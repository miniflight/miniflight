local MASTER_LEVEL = "/Game/levelsMaster/MAP_anduril_master"
local MASTER_OPTIONS = "game=/Script/DCGame.GameModeRaceBase"
local LIGHTING_LEVEL = "/Game/levels/MAP_anduril_lighting_DAY"
local TRACK_LEVEL = "/Game/levels/MAP_anduril_track01"

local stage = 1
local running = true
local master_requested = false
local lighting_requested = false
local track_requested = false
local track_finished = false
local start_requested = false

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
    return find_object("World", "MAP_anduril_master")
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
    if stage == 1 then
        if valid(world()) and valid(controller()) then
            stage = 2
            return
        end

        if not master_requested then
            local current_world = find_object("World", "/Game/")

            if valid(current_world) and valid(gameplay_statics()) then
                gameplay_statics():OpenLevel(
                    current_world,
                    FName(MASTER_LEVEL),
                    true,
                    MASTER_OPTIONS
                )
                master_requested = true
            end
        end
        return
    end

    if stage == 2 then
        if level_loaded("MAP_anduril_lighting_DAY") then
            stage = 3
        elseif not lighting_requested and valid(world()) then
            stream_level(LIGHTING_LEVEL)
            lighting_requested = true
        end
        return
    end

    if stage == 3 then
        local manager = find_object("MapLoadingManager")

        if valid(manager) then
            manager:OnLightingLevelLoaded()
            stage = 4
        end
        return
    end

    if stage == 4 then
        if level_loaded("MAP_anduril_track01") then
            stage = 5
        elseif not track_requested and valid(world()) then
            stream_level(TRACK_LEVEL)
            track_requested = true
        end
        return
    end

    if stage == 5 then
        local manager = find_object("MapLoadingManager")

        if not track_finished and valid(manager) then
            manager:OnTrackLevelLoaded()
            track_finished = true
        elseif track_finished and track_ready() then
            stage = 6
        end
        return
    end

    if stage == 6 then
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
                stage = 7
            end
        end
        return
    end

    if stage == 7 then
        local game_mode = find_object("GameModeRaceBase")

        if valid(game_mode) and valid(game_mode.OpponentsPopup) then
            game_mode.OpponentsPopup:ConfirmPopup()
            stage = 8
        end
        return
    end

    if race_started() then
        print("[DirectVQ1] physics_ready=true\n")
        running = false
    elseif not start_requested and valid(controller()) then
        controller():ServerForceStartRaceWithoutRaceVerification()
        start_requested = true
    end
end

local function run()
    ExecuteInGameThread(function()
        local ok, message = pcall(advance)

        if not ok then
            print("[DirectVQ1] failed: " .. tostring(message) .. "\n")
            running = false
        end
    end)

    if running then
        ExecuteWithDelay(250, run)
    end
end

print("[DirectVQ1] loaded\n")
run()
