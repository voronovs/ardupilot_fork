local last_print_ms = 0     -- GCS update timer
local interval_ms = 1000    -- update every 1s, ms
local current = 0           -- current from 6S2P, A
local voltage = 0           -- voltage from 6S2P, V
local watts = 0             -- watts from 6S2P, W
local capacity_used = 0     -- consumed capacity, mAh
local batt_capacity = battery:pack_capacity_mah(0) -- battcapacity 6S2P from parameters, mAh
local state_with_payload = true
local state_empty = false
local change_params_flag = false
local update_user = false -- flag of update messages to GCS
local brd_serial_number = param:get("BRD_SERIAL_NUM")
local watts_sum = 0
local sample_count = 0
local avg_watts = 0 -- average power consumption

gcs:send_text(7, string.format("Serial number - FLUAVFLY202505М%d", brd_serial_number))
gcs:send_text(7, string.format("FW version - FLYCO_4.6.3.2"))
gcs:send_text(7, string.format("Parameters version - 20251205"))
gcs:send_text(7, string.format("Script version - 20251205"))

notify:play_tune(
        'MFT100' ..
        'O3L4F#F#F#L8D.L16A' ..
        'L4F#L8D.L16AL2F#' ..
        'O4L4C#C#C#L8D.O3L16A' ..
        'L4FL8D.L16AL2F#')

function updateParameters ()
    if state_with_payload and change_params_flag then
        param:set_and_save("PSC_ACCZ_P", 0.37)
        param:set_and_save("PSC_ACCZ_I", 0.74)
        param:set_and_save("INS_HNTCH_REF", 0.3)
        change_params_flag = false
        gcs:send_text(7, string.format("Payload params activated"))
    elseif state_empty and change_params_flag then
        param:set_and_save("ATC_RAT_PIT_D", 0.00979057)
        param:set_and_save("ATC_RAT_PIT_I", 0.2153274)
        param:set_and_save("ATC_RAT_PIT_P", 0.2153274)
        param:set_and_save("ATC_RAT_RLL_D", 0.009428053)
        param:set_and_save("ATC_RAT_RLL_I", 0.179338)
        param:set_and_save("ATC_RAT_RLL_P", 0.179338)
        param:set_and_save("INS_HNTCH_REF", 0.2)
        param:set_and_save("PSC_ACCZ_P", 0.225)
        param:set_and_save("PSC_ACCZ_I", 0.45)
        change_params_flag = false
        gcs:send_text(7, string.format("Empty params activated"))
    end
end

-- Чтение значений тока и напряжения с датчиков
function updateBatteryInfo()
    current = battery:current_amps(0) -- current from 6S2P, A
    voltage = battery:voltage(0) -- voltage from 6S2P, V
    capacity_used = battery:consumed_mah(0)   -- consumed capacity, mAh

    if current ~= nil and  voltage ~= nil then
        watts = current*voltage -- watts from 6S2P, W
        watts_sum = watts_sum + watts
        sample_count = sample_count + 1
    end

    if (update_user) then
        -- print to GCS consumed capacity from 6S2P
        if math.floor(capacity_used) >= math.floor(batt_capacity)*0.85 then
            gcs:send_text(3, string.format("%d / %d mAh, %s A, %s V", math.floor(capacity_used), math.floor(batt_capacity), tostring(current), tostring(voltage)))
        elseif math.floor(capacity_used) < math.floor(batt_capacity)*0.85 and math.floor(capacity_used) >= math.floor(batt_capacity)*0.45 then
            gcs:send_text(5, string.format("%d / %d mAh, %s A, %s V", math.floor(capacity_used), math.floor(batt_capacity), tostring(current), tostring(voltage)))
        elseif math.floor(capacity_used) < math.floor(batt_capacity)*0.5 then
            gcs:send_text(7, string.format("%d / %d mAh, %s A, %s V", math.floor(capacity_used), math.floor(batt_capacity), tostring(current), tostring(voltage)))
        end

        if sample_count > 0 then
            avg_watts = watts_sum / sample_count

            if math.floor(avg_watts) > 1500 and state_empty then
                change_params_flag = true
                state_with_payload = true
                state_empty = false
                updateParameters()
            elseif math.floor(avg_watts) <= 1500 and state_with_payload then
                change_params_flag = true
                state_with_payload = false
                state_empty = true
                updateParameters()
            end
        end
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