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
local watts_sum = 0
local sample_count = 0
local avg_watts = 0 -- average power consumption

gcs:send_text(7, string.format("Serial number - FLUAVTRМ500202511М%d", brd_serial_number)) -- вставить серийный номер
gcs:send_text(7, string.format("FW version - TRM500X11CO_4.6.3.1")) -- вставить название полетного контроллера
gcs:send_text(7, string.format("Parameters version - 20260213"))
gcs:send_text(7, string.format("Script version - 20260213"))

notify:play_tune(
        'MFT100' ..
        'O3L4F#F#F#L8D.L16A' ..
        'L4F#L8D.L16AL2F#' ..
        'O4L4C#C#C#L8D.O3L16A' ..
        'L4FL8D.L16AL2F#')

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
        param:set_and_save("ANGLE_MAX", 2000)
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
        param:set_and_save("ANGLE_MAX", 2500)
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
    voltage = 2*battery:voltage(0) -- voltage from 12S1P, V
    capacity_used = battery:consumed_mah(0)   -- consumed capacity, mAh

    if current ~= nil and voltage ~= nil then
        watts = current*voltage -- watts from 12S1P, W
        watts_sum = watts_sum + watts
        sample_count = sample_count + 1
    end

    if (update_user) then
        -- print to GCS consumed capacity from 12S1P
        if math.floor(capacity_used) >= math.floor(batt_capacity)*0.85 then
            gcs:send_text(3, string.format("%d / %d mAh, %s A, %s V", math.floor(capacity_used), math.floor(batt_capacity), tostring(current*3), tostring(voltage)))
        elseif math.floor(capacity_used) < math.floor(batt_capacity)*0.85 and math.floor(capacity_used) >= math.floor(batt_capacity)*0.45 then
            gcs:send_text(5, string.format("%d / %d mAh, %s A, %s V", math.floor(capacity_used), math.floor(batt_capacity), tostring(current*3), tostring(voltage)))
        elseif math.floor(capacity_used) < math.floor(batt_capacity)*0.45 then
            gcs:send_text(7, string.format("%d / %d mAh, %s A, %s V", math.floor(capacity_used), math.floor(batt_capacity), tostring(current*3), tostring(voltage)))
        end

        if sample_count > 0 then
            avg_watts = watts_sum / sample_count

            if math.floor(avg_watts) > 4600 and state_empty then
                change_params_flag = true
                state_with_payload = true
                state_empty = false
                updateParameters()
            elseif math.floor(avg_watts) <= 4600 and state_with_payload then
                change_params_flag = true
                state_with_payload = false
                state_empty = true
                updateParameters()
            end
        end
        -- Сброс накопителей
        watts_sum = 0
        sample_count = 0
    end
end

-- main
function update ()
    local now_ms = millis() -- get the time since boot
    update_user = false

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