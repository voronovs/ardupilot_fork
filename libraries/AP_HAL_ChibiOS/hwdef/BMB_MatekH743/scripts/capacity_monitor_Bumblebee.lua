local last_print_ms = 0     -- GCS update timer
local interval_ms = 1000    -- update every 1s, ms
local capacity_used = 0     -- consumed capacity, mAh
local batt_capacity = battery:pack_capacity_mah(0) -- battcapacity PDB from parameters, mAh
local update_user = false -- flag of update messages to GCS
local brd_serial_number = param:get("BRD_SERIAL_NUM")

gcs:send_text(7, string.format("Serial number - FLUAVBLB202510М%d", brd_serial_number)) -- вставить серийный номер
gcs:send_text(7, string.format("FW version - BMB_MatekH743_4.6.2.1")) -- вставить название полетного контроллера
gcs:send_text(7, string.format("ARS version - NONE"))
gcs:send_text(7, string.format("Parameters version - 20251014"))
gcs:send_text(7, string.format("Script version - 20251014"))

notify:play_tune(
        'MFT100' ..
        'O3L4F#F#F#L8D.L16A' ..
        'L4F#L8D.L16AL2F#' ..
        'O4L4C#C#C#L8D.O3L16A' ..
        'L4FL8D.L16AL2F#')

function updateBatteryInfo()
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
    update_user = false

    if (now_ms - last_print_ms > 5000) then -- update every 5s, ms
        last_print_ms = now_ms
        update_user = true
    end

    if arming:is_armed() then
        if (update_user) then
            updateBatteryInfo()
        end
    end

    return update, interval_ms
end

return update()