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

gcs:send_text(7, string.format("Serial number - FLUAVTRМ500202509М%d", brd_serial_number))
gcs:send_text(7, string.format("FW version - TRM500X11CO+_4.5.7.1"))
gcs:send_text(7, string.format("ARS version - NONE"))
gcs:send_text(7, string.format("Parameters version - 20250903"))
gcs:send_text(7, string.format("Script version - 20250903"))

function updateParameters ()
    if state_with_payload and change_params_flag then
        param:set_and_save("ATC_RAT_PIT_D", 0.0056259)
        param:set_and_save("ATC_RAT_PIT_I", 0.1913947)
        param:set_and_save("ATC_RAT_PIT_P", 0.1913947)
        param:set_and_save("ATC_RAT_RLL_D", 0.0052129)
        param:set_and_save("ATC_RAT_RLL_I", 0.1612214)
        param:set_and_save("ATC_RAT_RLL_P", 0.1612214)
        param:set_and_save("ATC_RAT_YAW_I", 0.0512)
        param:set_and_save("ATC_RAT_YAW_P", 0.512)
        param:set_and_save("ANGLE_MAX", 1800)
        param:set_and_save("PILOT_ACCEL_Z", 50)
        param:set_and_save("PILOT_SPEED_DN", 100)
        param:set_and_save("PSC_ACCZ_P", 0.21)
        param:set_and_save("PSC_ACCZ_I", 0.42)
        change_params_flag = false
        gcs:send_text(7, string.format("Payload params activated"))
    elseif state_empty and change_params_flag then
        param:set_and_save("ATC_RAT_PIT_D", 0.0056627)
        param:set_and_save("ATC_RAT_PIT_I", 0.1735554)
        param:set_and_save("ATC_RAT_PIT_P", 0.1735554)
        param:set_and_save("ATC_RAT_RLL_D", 0.0065161)
        param:set_and_save("ATC_RAT_RLL_I", 0.2015268)
        param:set_and_save("ATC_RAT_RLL_P", 0.2015268)
        param:set_and_save("ATC_RAT_YAW_I", 0.064)
        param:set_and_save("ATC_RAT_YAW_P", 0.64)
        param:set_and_save("ANGLE_MAX", 2250)
        param:set_and_save("PILOT_ACCEL_Z", 100)
        param:set_and_save("PILOT_SPEED_DN", 0)
        param:set_and_save("PSC_ACCZ_P", 0.15)
        param:set_and_save("PSC_ACCZ_I", 0.3)
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

    if math.floor(watts) > 3600 and state_empty then
        change_params_flag = true
        state_with_payload = true
        state_empty = false
        updateParameters()
    elseif math.floor(watts) <= 3600 and state_with_payload then
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