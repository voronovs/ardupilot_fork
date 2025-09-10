local last_print_ms = 0     -- GCS update timer
local interval_ms = 1000    -- update every 1s, ms
local current = 0           -- current from 12S1P, A
local voltage = 0           -- voltage from 12S1P, V
local watts = 0             -- watts from 12S1P, W
local capacity_used = 0     -- consumed capacity, mAh
local batt_capacity = battery:pack_capacity_mah(0) -- battcapacity 12S1P from parameters, mAh
local state_with_payload = true
local state_empty = false
local change_params_flag = false
local update_user = false -- flag of update messages to GCS
local brd_serial_number = param:get("BRD_SERIAL_NUM")


gcs:send_text(7, string.format("Serial number - FLUAVANT24X9PYYYYMMМ%d", brd_serial_number)) -- вставить серийный номер
gcs:send_text(7, string.format("FW version - ANT24X9Pxxx_4.5.7.2")) -- вставить название полетного контроллера
gcs:send_text(7, string.format("ARS version - NONE"))
gcs:send_text(7, string.format("Parameters version - 20250910"))
gcs:send_text(7, string.format("Script version - 20250910"))

function updateParameters ()
    if state_with_payload and change_params_flag then
        param:set_and_save("ATC_RAT_PIT_D", 0.006749)
        param:set_and_save("ATC_RAT_PIT_I", 0.1689983)
        param:set_and_save("ATC_RAT_PIT_P", 0.1689983)
        param:set_and_save("ATC_RAT_RLL_D", 0.0087827)
        param:set_and_save("ATC_RAT_RLL_I", 0.1907172)
        param:set_and_save("ATC_RAT_RLL_P", 0.1907172)
        param:set_and_save("LAND_SPEED_HIGH", 60)
        param:set_and_save("PSC_ACCZ_P", 0.23)
        param:set_and_save("PSC_ACCZ_I", 0.46)
        change_params_flag = false
        gcs:send_text(7, string.format("Payload params activated"))
    elseif state_empty and change_params_flag then
        param:set_and_save("ATC_RAT_PIT_D", 0.0084362)
        param:set_and_save("ATC_RAT_PIT_I", 0.1906413)
        param:set_and_save("ATC_RAT_PIT_P", 0.1906413)
        param:set_and_save("ATC_RAT_RLL_D", 0.0059036)
        param:set_and_save("ATC_RAT_RLL_I", 0.1794221)
        param:set_and_save("ATC_RAT_RLL_P", 0.1794221)
        param:set_and_save("LAND_SPEED_HIGH", 0)
        param:set_and_save("PSC_ACCZ_P", 0.14)
        param:set_and_save("PSC_ACCZ_I", 0.28)
        change_params_flag = false
        gcs:send_text(7, string.format("Empty params activated"))
    end
end

function updateBatteryInfo()
    current = battery:current_amps(0) -- current from 12S1P, A
    voltage = battery:voltage(0) -- voltage from 12S1P, V
    watts = current*voltage -- watts from 12S1P, W
    capacity_used = battery:consumed_mah(0)   -- consumed capacity, mAh

    if (update_user) then
        -- print to GCS consumed capacity from 12S1P
        if math.floor(capacity_used) >= math.floor(batt_capacity)*0.85 then
            gcs:send_text(3, string.format("batt_used: %d / %d mAh", math.floor(capacity_used), math.floor(batt_capacity)))
        elseif math.floor(capacity_used) < math.floor(batt_capacity)*0.85 and math.floor(capacity_used) >= math.floor(batt_capacity)*0.45 then
            gcs:send_text(5, string.format("batt_used: %d / %d mAh", math.floor(capacity_used), math.floor(batt_capacity)))
        elseif math.floor(capacity_used) < math.floor(batt_capacity)*0.45 then
            gcs:send_text(7, string.format("batt_used: %d / %d mAh", math.floor(capacity_used), math.floor(batt_capacity)))
        end
    end

    if math.floor(watts) > 3000 and state_empty then
        change_params_flag = true
        state_with_payload = true
        state_empty = false
        updateParameters()
    elseif math.floor(watts) <= 3000 and state_with_payload then
        change_params_flag = true
        state_with_payload = false
        state_empty = true
        updateParameters()
    end
end

-- main
function update ()
    local now_ms = millis() -- get the time since boot

    if (now_ms - last_print_ms > 5000) then -- update every 5s, ms
        last_print_ms = now_ms
        update_user = true
    end

    if arming:is_armed() then
        updateBatteryInfo()
    end

    return update, interval_ms
end

return update()