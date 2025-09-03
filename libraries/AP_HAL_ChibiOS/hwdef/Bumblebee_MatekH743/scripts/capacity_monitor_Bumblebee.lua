local last_print_ms = 0     -- GCS update timer
local interval_ms = 1000    -- update every 1s, ms
local current = 0           -- current from 12S1P, A
local voltage = 0           -- voltage from 12S1P, V
local capacity_used = 0     -- consumed capacity, mAh
local batt_capacity = battery:pack_capacity_mah(0) -- battcapacity 12S1P from parameters, mAh
local update_user = false -- flag of update messages to GCS

gcs:send_text(7, string.format("Serial number - 1"))
gcs:send_text(7, string.format("FW version - Bumblebee_v4.5.7.1"))
gcs:send_text(7, string.format("ARS version - NONE"))
gcs:send_text(7, string.format("Parameters version - 20250903"))
gcs:send_text(7, string.format("Script version - 20250903"))

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