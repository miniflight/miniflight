local MASTER_LEVEL = "/Game/levelsMaster/MAP_anduril_master"
local MASTER_OPTIONS = "game=/Script/DCGame.GameModeRaceBase"
local LIGHTING_LEVEL = "/Game/levels/MAP_anduril_lighting_DAY"
local TRACK_LEVEL = "/Game/levels/MAP_anduril_track01"

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
    if stage == "load_master" or stage == "wait_for_master" then
        if valid(world()) and valid(controller()) then
            stage = "load_lighting"
            return
        end

        if stage == "load_master" then
            local current_world = find_object("World", "/Game/")

            if valid(current_world) and valid(gameplay_statics()) then
                gameplay_statics():OpenLevel(
                    current_world,
                    FName(MASTER_LEVEL),
                    true,
                    MASTER_OPTIONS
                )
                stage = "wait_for_master"
            end
        end
        return
    end

    if stage == "load_lighting" or stage == "wait_for_lighting" then
        if level_loaded("MAP_anduril_lighting_DAY") then
            stage = "notify_lighting_loaded"
        elseif stage == "load_lighting" and valid(world()) then
            stream_level(LIGHTING_LEVEL)
            stage = "wait_for_lighting"
        end
        return
    end

    if stage == "notify_lighting_loaded" then
        local manager = find_object("MapLoadingManager")

        if valid(manager) then
            manager:OnLightingLevelLoaded()
            stage = "load_track"
        end
        return
    end

    if stage == "load_track" or stage == "wait_for_track" then
        if level_loaded("MAP_anduril_track01") then
            stage = "notify_track_loaded"
        elseif stage == "load_track" and valid(world()) then
            stream_level(TRACK_LEVEL)
            stage = "wait_for_track"
        end
        return
    end

    if stage == "notify_track_loaded" or stage == "wait_for_track_ready" then
        local manager = find_object("MapLoadingManager")

        if stage == "notify_track_loaded" and valid(manager) then
            manager:OnTrackLevelLoaded()
            stage = "wait_for_track_ready"
        elseif stage == "wait_for_track_ready" and track_ready() then
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
                stage = "confirm_opponents"
            end
        end
        return
    end

    if stage == "confirm_opponents" then
        local game_mode = find_object("GameModeRaceBase")

        if valid(game_mode) and valid(game_mode.OpponentsPopup) then
            local system = StaticFindObject("/Script/Engine.Default__KismetSystemLibrary")
            local vsync = system:GetConsoleVariableIntValue("r.VSync")
            system:ExecuteConsoleCommand(world(), "r.VSync 0", controller())
            print("[DirectVQ1] vsync=" .. tostring(vsync) .. "->" .. tostring(system:GetConsoleVariableIntValue("r.VSync")) .. "\n")
            game_mode.OpponentsPopup:ConfirmPopup()
            local results = controller():GetHUD().ResultsScreen
            results.ResultGrid:SetIsEnabled(false) -- Pending grid selections block MAVLink reset.
            results.ResultGridAlt:SetIsEnabled(false)
            print("[DirectVQ1] results_grids_enabled=" ..
                tostring(results.ResultGrid:GetIsEnabled() or results.ResultGridAlt:GetIsEnabled()) .. "\n")
            stage = "start_race"
        end
        return
    end

    if stage == "ready" or stage == "failed" then return end
    if race_started() then
        print("[DirectVQ1] physics_ready=true\n")
        stage = "ready"
    elseif stage == "start_race" and valid(controller()) then
        controller():ServerForceStartRaceWithoutRaceVerification()
        stage = "wait_for_go"
    end
end

local function run()
    ExecuteInGameThread(function()
        local ok, message = pcall(advance)

        if not ok then
            print("[DirectVQ1] failed: " .. tostring(message) .. "\n")
            stage = "failed"
        end
        if stage ~= "ready" and stage ~= "failed" then
            ExecuteWithDelay(250, run)
        end
    end)
end

print("[DirectVQ1] loaded\n")
run()
