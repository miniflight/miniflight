import numpy as np
import random
from enum import Enum, auto
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from gymnasium import Env
from gymnasium.spaces import Box
import torch
import torch.nn as nn, torch.nn.functional as F
import torch.optim as optim

from torch.distributions import MultivariateNormal
from torch.utils.data import DataLoader, TensorDataset


# Hyperparameters
state_dim = 4
action_dim = 2
max_episodes = 5000
max_timesteps = 3000
update_timestep = 4000
log_interval = 1
hidden_dim = 32
lr = 3e-4
gamma = 0.99
K_epochs = 3
eps_clip = 0.1
action_std = 0.5
gae_lambda = 0.95
ppo_loss_coef = 1.0
critic_loss_coef = 0.5
entropy_coef = 0.01
batch_size = 32

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim, dropout_rate=0.2):
        super(ActorCritic, self).__init__()
        # Actor network
        self.actor_fc1 = nn.Linear(state_dim, hidden_dim)
        self.actor_ln1 = nn.LayerNorm(hidden_dim)
        self.actor_fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.actor_ln2 = nn.LayerNorm(hidden_dim)
        self.actor_out = nn.Linear(hidden_dim, action_dim)
        self.actor_dropout = nn.Dropout(dropout_rate)
        
        # Critic network
        self.critic_fc1 = nn.Linear(state_dim, hidden_dim)
        self.critic_ln1 = nn.LayerNorm(hidden_dim)
        self.critic_fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.critic_ln2 = nn.LayerNorm(hidden_dim)
        self.critic_out = nn.Linear(hidden_dim, 1)
        self.critic_dropout = nn.Dropout(dropout_rate)

    def forward(self, state):
        # Actor network forward pass
        x = F.relu(self.actor_ln1(self.actor_fc1(state)))
        x = self.actor_dropout(x)
        x = F.relu(self.actor_ln2(self.actor_fc2(x)))
        x = self.actor_dropout(x)
        action_mean = torch.tanh(self.actor_out(x))  # Continuous action space

        # Rescale action_mean to match the action space
        action_mean_scaled = torch.clone(action_mean)  # Clone to avoid in-place modification
        action_mean_scaled[:, 0] = action_mean[:, 0] * 25 + 25  # Scaling for throttle (0 to 50)
        action_mean_scaled[:, 1] = action_mean[:, 1] * np.pi   # Scaling for steer angle (-π to π)
        
        # Critic network forward pass
        v = F.relu(self.critic_ln1(self.critic_fc1(state)))
        v = self.critic_dropout(v)
        v = F.relu(self.critic_ln2(self.critic_fc2(v)))
        v = self.critic_dropout(v)
        value = self.critic_out(v)
        
        return action_mean_scaled, value

class BatchPhase(Enum):
    COLLECTING = auto()
    READY_TO_UPDATE = auto()


class Memory:
    def __init__(self):
        self.states = []
        self.actions = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []

    def clear_memory(self):
        del self.states[:]
        del self.actions[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]

    def phase(self, batch_steps):
        return BatchPhase.READY_TO_UPDATE if len(self.rewards) >= batch_steps else BatchPhase.COLLECTING

class PPO:
    def __init__(self, actor_critic, lr, gamma, lamda, K_epochs, eps_clip, action_std, ppo_loss_coef, critic_loss_coef, entropy_coef, batch_size):
        self.actor_critic = actor_critic
        self.optimizer = optim.Adam(actor_critic.parameters(), lr=lr)
        self.gamma = gamma
        self.lamda = lamda
        self.K_epochs = K_epochs
        self.eps_clip = eps_clip
        self.action_std = action_std
        self.ppo_loss_coef = ppo_loss_coef
        self.critic_loss_coef = critic_loss_coef
        self.entropy_coef = entropy_coef
        self.mse_loss = nn.MSELoss()
        self.batch_size = batch_size

    def select_action(self, state, memory):
        state = torch.FloatTensor(state).unsqueeze(0)
        
        with torch.no_grad():
            action_mean, _ = self.actor_critic(state)
            action_var = torch.full((action_mean.size(-1),), self.action_std**2)
            cov_mat = torch.diag(action_var).unsqueeze(0)
            
            dist = MultivariateNormal(action_mean, covariance_matrix=cov_mat)
            action = dist.sample()
            action_logprob = dist.log_prob(action)
            
        memory.states.append(state)
        memory.actions.append(action)
        memory.logprobs.append(action_logprob)

        return action.detach().cpu().numpy().flatten()

    def rtg(self, rewards, is_terms):
        out = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(rewards), reversed(is_terms)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            out.insert(0, discounted_reward)
        return out

    def compute_advantages(self, rewards, state_values, is_terminals):
        advantages = []
        gae = 0
        state_values = torch.cat((state_values, torch.tensor([0.0])))  # Add a zero to handle the last state value

        for step in reversed(range(len(rewards))):
            delta = rewards[step] + self.gamma * state_values[step + 1] * (1 - is_terminals[step]) - state_values[step]
            gae = delta + self.gamma * self.lamda * (1 - is_terminals[step]) * gae
            advantages.insert(0, gae)

        advantages = torch.tensor(advantages, dtype=torch.float32)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        returns = advantages + state_values[:-1]

        return advantages, returns
    
    def update(self, memory):
        with torch.no_grad():
            rewards = torch.tensor(memory.rewards, dtype=torch.float32)
            is_terms = torch.tensor(memory.is_terminals, dtype=torch.float32)
            
            old_states = torch.cat(memory.states).detach()
            old_actions = torch.cat(memory.actions).detach()
            old_logprobs = torch.cat(memory.logprobs).detach()
            _, state_values = self.actor_critic(old_states)
            state_values = torch.squeeze(state_values)

            rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-8)
            advantages, returns = self.compute_advantages(rewards, state_values, is_terms)
        
        dataset = TensorDataset(old_states, old_actions, old_logprobs, advantages, returns)
        dataloader = DataLoader(dataset, batch_size=self.batch_size, shuffle=True)
        for _ in range(self.K_epochs):
            for batch in dataloader:
                batch_states, batch_actions, batch_logprobs, batch_advantages, batch_returns = batch
                
                # Forward pass
                action_means, state_values = self.actor_critic(batch_states)
                action_var = torch.full((action_means.size(-1),), self.action_std**2)
                cov_mat = torch.diag(action_var).unsqueeze(0)
                
                dist = MultivariateNormal(action_means, covariance_matrix=cov_mat)
                action_logprobs = dist.log_prob(batch_actions)
                dist_entropy = dist.entropy()
                state_values = torch.squeeze(state_values)
                
                # Compute ratios
                ratios = torch.exp(action_logprobs - batch_logprobs.detach())
                surr1 = ratios * batch_advantages
                surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * batch_advantages

                # PPO loss components
                actor_loss = -torch.min(surr1, surr2).mean()
                critic_loss = self.mse_loss(state_values, batch_returns)
                entropy_loss = dist_entropy.mean()

                # Total loss
                loss = self.ppo_loss_coef * actor_loss + self.critic_loss_coef * critic_loss - self.entropy_coef * entropy_loss

                # Backward pass and optimizer step with gradient clipping
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.actor_critic.parameters(), max_norm=0.5)
                self.optimizer.step()

def train(max_episodes, max_timesteps, update_timestep, log_interval, state_dim, action_dim, hidden_dim, lr, gamma, K_epochs, eps_clip, action_std, gae_lambda, ppo_loss_coef, critic_loss_coef, entropy_coef, batch_size):
    if update_timestep <= 0:
        raise ValueError("update_timestep must be positive")
    actor_critic = ActorCritic(state_dim, action_dim, hidden_dim)
    ppo = PPO(actor_critic, lr, gamma, gae_lambda, K_epochs, eps_clip, action_std, ppo_loss_coef, critic_loss_coef, entropy_coef, batch_size)
    memory = Memory()
    
    episode_returns = []
    running_avg_returns = []
    
    plt.ion()
    fig, ax = plt.subplots()
    
    # Initialize the environment once outside the loop
    env = ContinuousPathfindingEnv(num_obstacles=0, render_mode='human')
    
    for episode in range(1, max_episodes + 1):
        state, _ = env.reset()
        total_reward = 0
        for t in range(max_timesteps):
            action = ppo.select_action(state, memory)
            next_state, reward, done, trunc, _ = env.step(action)

            memory.rewards.append(reward)
            total_reward += reward
            memory.is_terminals.append(done or trunc)

            if memory.phase(update_timestep) is BatchPhase.READY_TO_UPDATE:
                ppo.update(memory)
                memory.clear_memory()

            state = next_state
            if done or trunc:
                break

        episode_returns.append(total_reward)
        running_avg = np.mean(episode_returns[-log_interval:]) if episode >= log_interval else np.mean(episode_returns)
        running_avg_returns.append(running_avg)
        
        if episode % log_interval == 0:
            print(f'ep {episode:6} return {total_reward:.2f}')

            # Dynamic plotting
            ax.clear()
            ax.plot(episode_returns, label='Returns')
            ax.plot(running_avg_returns, label='Running Average Returns')
            ax.axhline(y=max(episode_returns), color='r', linestyle='--', label='Max Return')
            ax.legend()
            ax.set_xlabel('Episode')
            ax.set_ylabel('Return')
            plt.pause(0.01)
    
    plt.ioff()
    plt.show()

class EpisodePhase(Enum):
    ACTIVE = auto()
    TERMINATED = auto()
    TRUNCATED = auto()


class ContinuousPathfindingEnv(Env):
    def __init__(self, grid_size=1000, num_obstacles=10, render_mode='human'):
        self.grid_size = grid_size
        self.num_obstacles = num_obstacles
        self.render_mode = render_mode

        # Initialize environment elements
        self.obstacles = self._generate_obstacles()
        self.start = self._generate_valid_point()
        self.end = np.array([self.grid_size / 2, self.grid_size / 2])  # Set end point to the origin

        self.current_position = np.array(self.start)
        self.phase = EpisodePhase.ACTIVE
        self.max_steps = 1000
        self.steps_taken = 0

        # Define action and observation spaces
        self.action_space = Box(low=np.array([0.0, -np.pi]), high=np.array([50.0, np.pi]), dtype=np.float32)
        self.observation_space = Box(low=0.0, high=self.grid_size, shape=(4,), dtype=np.float32)

        self.positions = []

        # Initialize rendering
        if self.render_mode == 'human':
            self._initialize_rendering()

    def _initialize_rendering(self):
        if not hasattr(self, 'fig') or not plt.fignum_exists(self.fig.number):
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(0, self.grid_size)
        self.ax.set_ylim(0, self.grid_size)
        self.ax.set_facecolor('white')
        self.particle = self.ax.arrow(self.current_position[0], self.current_position[1], 0, 0, 
                                    head_width=10, head_length=20, fc='blue', ec='blue', label='Particle')
        self.ax.scatter(*self.start, c='green', marker='o', label='Start')
        self.ax.scatter(*self.end, c='red', marker='x', label='End')
        for (lower_left, width, height) in self.obstacles:
            rect = patches.Rectangle(lower_left, width, height, linewidth=1, edgecolor='r', facecolor='none')
            self.ax.add_patch(rect)
        self.ax.legend()
        plt.ion()  # Enable interactive mode
        plt.show(block=False)  # Prevent blocking the code execution

    def _reset_rendering(self):
        if not hasattr(self, 'fig') or not plt.fignum_exists(self.fig.number):
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.clear()
        self.ax.set_xlim(0, self.grid_size)
        self.ax.set_ylim(0, self.grid_size)
        self.ax.set_facecolor('white')
        self.particle = self.ax.arrow(self.current_position[0], self.current_position[1], 0, 0, 
                                    head_width=10, head_length=20, fc='blue', ec='blue', label='Particle')
        self.ax.scatter(*self.start, c='green', marker='o', label='Start')
        self.ax.scatter(*self.end, c='red', marker='x', label='End')
        for (lower_left, width, height) in self.obstacles:
            rect = patches.Rectangle(lower_left, width, height, linewidth=1, edgecolor='r', facecolor='none')
            self.ax.add_patch(rect)
        self.ax.legend()
        plt.draw()

    def _generate_random_point(self):
        return (random.uniform(0, self.grid_size), random.uniform(0, self.grid_size))

    def _generate_obstacles(self):
        obstacles = []
        for _ in range(self.num_obstacles):
            lower_left = self._generate_random_point()
            width = random.uniform(0.01 * self.grid_size, 0.1 * self.grid_size)
            height = random.uniform(0.01 * self.grid_size, 0.1 * self.grid_size)
            obstacles.append((lower_left, width, height))
        return obstacles

    def _point_inside_obstacles(self, point):
        for (lower_left, width, height) in self.obstacles:
            if (lower_left[0] <= point[0] <= lower_left[0] + width and
                lower_left[1] <= point[1] <= lower_left[1] + height):
                return True
        return False

    def _point_close_to_obstacles(self, point, threshold=50.0):
        for (lower_left, width, height) in self.obstacles:
            center = np.array([lower_left[0] + width / 2, lower_left[1] + height / 2])
            if np.linalg.norm(point - center) < threshold:
                return True
        return False

    def _generate_valid_point(self):
        point = self._generate_random_point()
        while self._point_inside_obstacles(point):
            point = self._generate_random_point()
        return point

    @property
    def done(self):
        return self.phase is EpisodePhase.TERMINATED

    @property
    def trunc(self):
        return self.phase is EpisodePhase.TRUNCATED

    def reset(self):
        self.obstacles = self._generate_obstacles()
        self.start = self._generate_valid_point()
        self.end = np.array([self.grid_size / 2, self.grid_size / 2])  # Set end point to the origin
        self.current_position = np.array(self.start)
        self.phase = EpisodePhase.ACTIVE
        self.steps_taken = 0
        self.positions = [self.current_position.copy()]

        if self.render_mode == 'human':
            self._reset_rendering()

        state = np.concatenate((self.current_position, self.end))
        return state, {}

    def step(self, action):
        if self.phase is not EpisodePhase.ACTIVE:
            return np.concatenate((self.current_position, self.end)), 0.0, self.done, self.trunc, {}

        # Apply the action to update the current position
        throttle, steer_angle = action
        move = throttle * np.array([np.cos(steer_angle), np.sin(steer_angle)])
        self.current_position += move
        self.steps_taken += 1

        # Check for termination conditions
        self._check_termination()

        # Calculate reward
        reward = self._calculate_reward()

        # Record the position for animation
        self.positions.append(self.current_position.copy())

        if self.render_mode == 'human':
            self._update_rendering(move)

        state = np.concatenate((self.current_position, self.end))
        info = {}
        return state, reward, self.done, self.trunc, info

    def _check_termination(self):
        if not (0 <= self.current_position[0] <= self.grid_size and 0 <= self.current_position[1] <= self.grid_size):
            self.current_position = np.clip(self.current_position, 0, self.grid_size)
            self.phase = EpisodePhase.TERMINATED
        elif self._point_inside_obstacles(self.current_position):
            self.phase = EpisodePhase.TERMINATED
        elif np.linalg.norm(self.current_position - self.end) < 20.0:
            self.phase = EpisodePhase.TERMINATED
        elif self.steps_taken >= self.max_steps:
            self.phase = EpisodePhase.TRUNCATED

    def _calculate_reward(self):
        reward = 0 # Penalty for each time step
        distance_to_end = np.linalg.norm(self.current_position - self.end)
        previous_distance_to_end = np.linalg.norm(self.positions[-2] - self.end) if len(self.positions) > 1 else distance_to_end
        
        # Reward for moving closer to the end point
        if distance_to_end < previous_distance_to_end:
            reward += 1
        else:
            reward -= 1  # Penalty for moving away from the end point

        if distance_to_end < 20.0:
            reward += 100  # Large reward for reaching the end point

        if self._point_inside_obstacles(self.current_position):
            reward -= 100  # Large penalty for crashing into obstacles
        elif self._point_close_to_obstacles(self.current_position):
            reward -= 10  # Penalty for coming close to obstacles

        # Penalty for exiting environment boundaries
        if not (0 <= self.current_position[0] <= self.grid_size and 0 <= self.current_position[1] <= self.grid_size):
            reward -= 100  # Large penalty for exiting the boundaries

        return reward

    def _update_rendering(self, move):
        dx, dy = move
        self.particle.remove()
        self.particle = self.ax.arrow(self.current_position[0], self.current_position[1], dx, dy, 
                                      head_width=10, head_length=20, fc='blue', ec='blue')
        self.render()

    def render(self):
        if self.render_mode == 'human':
            plt.pause(0.01)
            plt.draw()


if __name__ == '__main__':

    train(max_episodes, max_timesteps, update_timestep, log_interval, state_dim, action_dim, hidden_dim, lr, gamma, K_epochs, eps_clip, action_std, gae_lambda, ppo_loss_coef, critic_loss_coef, entropy_coef, batch_size)


# Controller that points towards the end
def simple_controller(state):
    current_position = state[:2]
    end_position = state[2:]
    direction_vector = end_position - current_position
    steer_angle = np.arctan2(direction_vector[1], direction_vector[0])
    throttle = 1.0  # Full speed towards the end point
    return np.array([throttle, steer_angle])


