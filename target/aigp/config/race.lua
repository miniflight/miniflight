-- Read native race state independently of the MAVLink clock prediction.
local race_path = os.getenv("MINIFLIGHT_RACE_STATUS_FILE")
if race_path then
    local output = assert(io.open(race_path, "w"))
    local centers_loaded = false
    local game, player
    local function observe_race()
        ExecuteInGameThread(function()
            if not valid(game) then game = find_object("GameStateRaceBase") end
            if not valid(player) then player = controller() end
            local state = valid(player) and player.PlayerState or nil
            if valid(game) and valid(state) then
                if not centers_loaded and track_ready() then
                    local track = game:GetCurrentTrack()
                    local origin = track:GetStartingGrid().PlayerStart0:K2_GetComponentLocation()
                    local system = StaticFindObject("/Script/Engine.Default__KismetSystemLibrary")
                    local centers = {}
                    for index = 0, track:GetNumberOfSequentialGates() - 1 do
                        local center = {}
                        system:GetComponentBounds(track:GetGateBySequentialId(index).GateMesh, center, {}, {})
                        centers[#centers + 1] = string.format("[%.9f,%.9f,%.9f]",
                            (center.X - origin.X) / 100, (center.Y - origin.Y) / 100, -(center.Z - origin.Z) / 100)
                    end
                    output:write('{"gate_centers":[' .. table.concat(centers, ",") .. ']}\n')
                    centers_loaded = true
                end
                output:write(string.format(
                    '{"started":%s,"valid":%s,"completed":%s,"time_seconds":%.9f,"active_gate_index":%d,"finish_time_seconds":%.9f}\n',
                    tostring(game:HasRaceStarted()), tostring(state:IsRaceValid()),
                    tostring(state.bRaceCompleted), game:GetRaceTime(),
                    state.ActiveGateSequentialId, state:GetCompletedRaceTotalTime()))
                output:flush()
            end
            ExecuteWithDelay(50, observe_race)
        end)
    end
    observe_race()
end
