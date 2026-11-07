local capture_identified = false
LoopAsync(250, function()
    ExecuteInGameThread(function()
        if capture_identified or stage ~= "ready" then return end
        for _, obj in ipairs(FindAllOf("SceneCaptureComponent2D") or {}) do
            if obj:IsValid() and string.find(obj:GetFullName(), "BP_DronePawn_C_", 1, true) then
                local fps = os.getenv("MINIFLIGHT_CAPTURE_MAXFPS")
                local actual = "unchanged"
                if fps then
                    local system = StaticFindObject("/Script/Engine.Default__KismetSystemLibrary")
                    system:ExecuteConsoleCommand(world(), "t.MaxFPS " .. fps, controller())
                    actual = tostring(system:GetConsoleVariableIntValue("t.MaxFPS"))
                end
                local f = assert(io.open(os.getenv("MINIFLIGHT_CAPTURE_OBJECT"), "w"))
                f:write(string.format("%x\nmaxfps=%s\n", obj:GetAddress(), actual))
                f:close()
                capture_identified = true
                return
            end
        end
    end)
    return capture_identified
end)
