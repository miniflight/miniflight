-- Read native race state independently of the MAVLink clock prediction.
local race_path = os.getenv("MINIFLIGHT_RACE_STATUS_FILE")
if race_path then
    local output = assert(io.open(race_path, "w"))
    local function observe_race()
        ExecuteInGameThread(function()
            local game = find_object("GameStateRaceBase")
            local player = controller()
            local state = valid(player) and player.PlayerState or nil
            if valid(game) and valid(state) then
                output:write(string.format(
                    '{"started":%s,"valid":%s,"completed":%s,"time_seconds":%.9f}\n',
                    tostring(game:HasRaceStarted()), tostring(state:IsRaceValid()),
                    tostring(state.bRaceCompleted), game:GetRaceTime()))
                output:flush()
            end
            ExecuteWithDelay(50, observe_race)
        end)
    end
    observe_race()
end
