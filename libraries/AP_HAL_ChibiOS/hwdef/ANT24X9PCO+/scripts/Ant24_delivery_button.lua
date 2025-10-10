-- === Настройки ===
local BUTTON_ID = 1          -- кнопка BTN_PIN1 на AUX4
local HOLD_TIME = 3000       -- удержание 3 сек
local TIMER_DURATION = 15000 -- длительность таймера 15 сек
local AUTO_DELAY = 3000      -- задержка 3 сек после ARM

-- === Переменные ===
local pressed_time = 0
local state_guided = false
local timer_active = false
local start_time = 0
local last_beep = 0
local elapsed_seconds = 0
local arm_time = 0
local awaiting_auto = false
local gps_bad = false               -- true if GPS is failing checks

gcs:send_text(7, string.format("Script Button - 20250924"))

-- === Функция писка ===
local function beep(short)
    if short then
        notify:play_tune("MFT200L8a")   -- короткий сигнал
    else
        notify:play_tune("MFT800L4a")   -- длинный сигнал
    end
end

-- === Проверки условий ===
local function conditions_ok()
    if arming:is_armed() then
        gcs:send_text(0, "UAV already Armed")
        return false
    end
    if vehicle:get_mode() ~= 4 then -- 4 = GUIDED
        gcs:send_text(0, "Not in GUIDED mode")
        return false
    end
    -- check GPS
    local gps_speed_acc = gps:speed_accuracy(gps:primary_sensor())
    if gps_speed_acc == nil then
        gps_speed_acc = 99
    end
    local gps_speed_acc_bad = (gps_speed_acc > 0.8)
    local gps_num_sat = gps:num_sats(gps:primary_sensor())
    local gps_num_sat_bad = ((gps_num_sat == nil) or (gps:num_sats(gps:primary_sensor()) < 6))
    if gps_bad then
        -- GPS is bad, check for recovery
        if (not gps_speed_acc_bad and not gps_num_sat_bad) then
            gps_bad = false
        end
    else
        -- GPS is good, check for GPS going bad
        if (gps_speed_acc_bad or gps_num_sat_bad) then
            gps_bad = true
        end
    end
    if gps_bad then
        gcs:send_text(0, "No good GNSS fix")
        return false
    end
    if not rc:has_valid_input() then
        gcs:send_text(0, "No RC input")
        return false
    end
    return true
end

-- === Армирование ===
local function try_arm()
    if not arming:is_armed() then
        gcs:send_text(0, "Arming drone...")
        vehicle:arm()
        arm_time = millis()
        awaiting_auto = true
    else
        gcs:send_text(0, "Already armed")
    end
end

-- === Основной цикл ===
function update()
    local now = millis()
    local btn = not button:get_button_state(BUTTON_ID)

    -- удержание кнопки работает только если дрон disarm
    if not arming:is_armed() then
        if btn then
            if pressed_time == 0 then
                pressed_time = now
            elseif (now - pressed_time >= HOLD_TIME) then
                gcs:send_text(0, "Button held 3s: switching to GUIDED")
                if vehicle:set_mode(4) then -- 4 = GUIDED
                    state_guided = true
                else
                    gcs:send_text(0, "Failed to switch to GUIDED")
                end
            end
        else
            pressed_time = 0
        end
    end

    -- если перешли в GUIDED и все условия выполнены → старт таймера
    if state_guided and not timer_active then
        if conditions_ok() then
            gcs:send_text(0, "Conditions OK, starting timer 15s")
            timer_active = true
            start_time = now
            last_beep = now
            elapsed_seconds = 0
            beep(true)
        else
            state_guided = false -- сброс, если условия не прошли
        end
    end

    -- логика таймера
    if timer_active then
        if now - last_beep >= 1000 then
            elapsed_seconds = elapsed_seconds + 1
            last_beep = now
            if elapsed_seconds < (TIMER_DURATION // 1000) then
                beep(true)
            else
                timer_active = false
                gcs:send_text(0, "Timer finished, arming!")
                beep(false)
                try_arm()
            end
        end
    end

    -- после ARM ждём 3 сек и включаем AUTO
    if awaiting_auto and arming:is_armed() then
        if now - arm_time >= AUTO_DELAY then
            gcs:send_text(0, "Switching to AUTO")
            vehicle:set_mode(3) -- 3 = AUTO
            awaiting_auto = false
        end
    end

    return update, 100
end

return update, 100
