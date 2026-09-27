"""Run the R1 position baseline using the simulator's published course."""

from target.aigp.simulator import AIGPSimulator
from target.aigp.controllers.r1_gates import Controller


if __name__ == "__main__":
    AIGPSimulator(Controller(), "vq1.r1").rollout()
