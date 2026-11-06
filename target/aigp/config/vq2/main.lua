local ARENAS = {
    r1 = {
        master = "/Game/levelsMaster/MAP_anduril_master",
        world = "MAP_anduril_master",
        environment = "/Game/levels/MAP_anduril_world",
        environment_name = "MAP_anduril_world",
        lighting = "/Game/levels/MAP_anduril_lighting_DAY",
        lighting_name = "MAP_anduril_lighting_DAY",
        track = "/Game/levels/MAP_anduril_track01",
        track_name = "MAP_anduril_track01",
    },
    r2 = {
        master = "/Game/levelsMaster/MAP_arsenal_master",
        world = "MAP_arsenal_master",
        environment = "/Game/levels/MAP_arsenal_world",
        environment_name = "MAP_arsenal_world",
        lighting = "/Game/levels/MAP_arsenal_lighting_DAY",
        lighting_name = "MAP_arsenal_lighting_DAY",
        track = "/Game/levels/MAP_arsenal_track01",
        track_name = "MAP_arsenal_track01",
    },
}
local MASTER_OPTIONS = "game=/Script/DCGame.GameModeRaceBase"
local requested_mode = os and os.getenv and os.getenv("MINIFLIGHT_VQ2_MODE")
local arena = ARENAS[requested_mode]

if not arena then
    error("MINIFLIGHT_VQ2_MODE must be r1 or r2")
end

local stage = "load_master"

local function valid(object)
    return object and object:IsValid()
end

local function find_object(class_name, name_fragment)
    local objects = FindAllOf(class_name)

    if not objects then
        return nil
    end

    for _, object in ipairs(objects) do
        if valid(object) then
            local name = object:GetFullName()

            if not string.find(name, "Default__", 1, true)
                and (not name_fragment
                    or string.find(name, name_fragment, 1, true))
            then
                return object
            end
        end
    end
end

local function world()
    return find_object("World", arena.world)
end

local function controller()
    return find_object("DronePlayerController")
end

local function level_loaded(name)
    return valid(find_object("Level", name))
end

local function gameplay_statics()
    return StaticFindObject("/Script/Engine.Default__GameplayStatics")
end

local function stream_level(path)
    gameplay_statics():LoadStreamLevel(
        world(),
        FName(path),
        true,
        true,
        {}
    )
end

local function track_ready()
    local game_state = find_object("GameStateFlightBase")

    if not valid(game_state) or not game_state:IsTrackLevelLoaded() then
        return false
    end

    local track = game_state:GetCurrentTrack()

    return valid(track)
        and track:IsTrackValid()
        and valid(track:GetStartingGrid())
end

local function race_started()
    local game_state = find_object("GameStateRaceBase")
    return valid(game_state) and game_state:HasRaceStarted()
end

local function advance()
    if stage == "load_master" or stage == "wait_for_master" then
        local loaded_world = world()

        if valid(loaded_world) and valid(controller()) then
            print("[DirectVQ2] mode=" .. requested_mode .. "\n")
            print("[DirectVQ2] environment_present="
                .. tostring(level_loaded(arena.environment_name)) .. "\n")
            stage = "load_lighting"
            return
        end

        if stage == "load_master" then
            local current_world = find_object("World", "/Game/")

            if valid(current_world) and valid(gameplay_statics()) then
                gameplay_statics():OpenLevel(
                    current_world,
                    FName(arena.master),
                    true,
                    MASTER_OPTIONS
                )
                stage = "wait_for_master"
            end
        end
        return
    end

    if stage == "load_lighting" or stage == "wait_for_lighting" then
        if level_loaded(arena.lighting_name) then
            print("[DirectVQ2] lighting_loaded=true\n")
            stage = "notify_lighting_loaded"
        elseif stage == "load_lighting" and valid(world()) then
            print("[DirectVQ2] loading_lighting=" .. arena.lighting .. "\n")
            stream_level(arena.lighting)
            stage = "wait_for_lighting"
        end
        return
    end

    if stage == "notify_lighting_loaded" then
        local manager = find_object("MapLoadingManager")

        if valid(manager) then
            manager:OnLightingLevelLoaded()
            print("[DirectVQ2] lighting_callback=true\n")
            stage = "load_track"
        end
        return
    end

    if stage == "load_track" or stage == "wait_for_track" then
        if level_loaded(arena.track_name) then
            print("[DirectVQ2] track_loaded=true\n")
            stage = "notify_track_loaded"
        elseif stage == "load_track" and valid(world()) then
            print("[DirectVQ2] loading_track=" .. arena.track .. "\n")
            stream_level(arena.track)
            stage = "wait_for_track"
        end
        return
    end

    if stage == "notify_track_loaded" or stage == "wait_for_track_ready" then
        local manager = find_object("MapLoadingManager")

        if stage == "notify_track_loaded" and valid(manager) then
            manager:OnTrackLevelLoaded()
            print("[DirectVQ2] track_callback=true\n")
            stage = "wait_for_track_ready"
        elseif stage == "wait_for_track_ready" and track_ready() then
            print("[DirectVQ2] track_ready=true\n")
            stage = "assign_grid"
        end
        return
    end

    if stage == "assign_grid" then
        local game_state = find_object("GameStateRaceBase")
        local player_state = find_object("PlayerStateRaceBase")

        if valid(game_state) and valid(player_state) then
            if player_state.StartingGridSlot < 0 then
                local slot = game_state:FindAvailableStartingGridSlot()

                if slot >= 0 then
                    player_state.StartingGridSlot = slot
                end
            end

            if player_state.StartingGridSlot >= 0 then
                print("[DirectVQ2] grid_slot="
                    .. tostring(player_state.StartingGridSlot) .. "\n")
                stage = "confirm_opponents"
            end
        end
        return
    end

    if stage == "confirm_opponents" then
        local game_mode = find_object("GameModeRaceBase")

        if valid(game_mode) and valid(game_mode.OpponentsPopup) then
            game_mode.OpponentsPopup:ConfirmPopup()
            print("[DirectVQ2] opponents_confirmed=true\n")
            stage = "start_race"
        end
        return
    end

    if stage == "ready" or stage == "failed" then return end
    if race_started() then
        print("[DirectVQ2] physics_ready=true mode=" .. requested_mode .. "\n")
        stage = "ready"
    elseif stage == "start_race" and valid(controller()) then
        controller():ServerForceStartRaceWithoutRaceVerification()
        print("[DirectVQ2] force_start_requested=true\n")
        stage = "wait_for_go"
    end
end

local function run()
    ExecuteInGameThread(function()
        local ok, message = pcall(advance)

        if not ok then
            print("[DirectVQ2] failed: " .. tostring(message) .. "\n")
            stage = "failed"
        end

        if stage ~= "ready" and stage ~= "failed" then
            ExecuteWithDelay(250, run)
        end
    end)
end

print("[DirectVQ2] loaded requested_mode=" .. requested_mode .. "\n")
run()
