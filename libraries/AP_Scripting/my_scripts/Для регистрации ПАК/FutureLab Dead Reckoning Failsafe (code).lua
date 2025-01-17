---@diagnostic disable: param-type-mismatch
---@diagnostic disable: cast-local-type

-- create and initialise parameters
local PARAM_TABLE_KEY = 86  -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "DR_", 4), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add DR_ENABLE param')   -- 1 = enabled, 0 = disabled
assert(param:add_param(PARAM_TABLE_KEY, 2, 'FLY_TIMEOUT', 180), 'could not add DR_FLY_TIMEOUT param')-- deadreckoning timeout (in seconds)
assert(param:add_param(PARAM_TABLE_KEY, 3, 'NEXT_MODE', 2), 'could not add DR_NEXT_MODE param')     -- mode to switch to after GPS recovers or timeout elapses
assert(param:add_param(PARAM_TABLE_KEY, 4, 'ENABLE_ALT', 15), 'could not add DR_ENABLE_ALT param')   -- altitude from home (in meters) beyond which the dead reckoning will be enabled

-- bind parameters to variables
--[[
  // @Param: DR_ENABLE
  // @DisplayName: Deadreckoning Enable
  // @Description: Deadreckoning Enable
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local enable = Parameter("DR_ENABLE")                  -- 1 = enabled, 0 = disabled

--[[
  // @Param: DR_ENABLE_DIST
  // @DisplayName: Deadreckoning Enable Distance
  // @Description: Distance from home (in meters) beyond which the dead reckoning will be enabled
  // @Units: m
  // @User: Standard
--]]
local enable_alt = Parameter("DR_ENABLE_ALT")        -- distance from home (in meters) beyond which the dead reckoning will be enabled

--[[
  // @Param: DR_FLY_TIMEOUT
  // @DisplayName: Deadreckoning flight timeout
  // @Description: Copter will attempt to switch to NEXT_MODE after this many seconds of deadreckoning.  If it cannot switch modes it will continue in Guided_NoGPS.  Set to 0 to disable timeout
  // @Units: s
  // @User: Standard
--]]
local fly_timeoout = Parameter("DR_FLY_TIMEOUT")       -- deadreckoning timeout (in seconds)

--[[
  // @Param: DR_NEXT_MODE
  // @DisplayName: Deadreckoning Next Mode
  // @Description: Copter switch to this mode after GPS recovers or DR_FLY_TIMEOUT has elapsed.  Default is 6/RTL.  Set to -1 to return to mode used before deadreckoning was triggered
  // @Values: 2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,16:PosHold,17:Brake,20:Guided_NoGPS,21:Smart_RTL,27:Auto RTL
  // @User: Standard
--]]
local next_mode = Parameter("DR_NEXT_MODE")            -- mode to switch to after GPS recovers or timeout elapses

-- modes deadreckoning may be activated from
-- comment out lines below to remove protection from these modes
local protected_mode_array = {
         0, -- STAB
         2, -- ALTHOLD
         3, -- AUTO
         4, -- GUIDED
         5, -- LOITER
         6, -- RTL
         7, -- CIRCLE
         9, -- LAND
         16, -- POSHOLD
         17, -- BRAKE
         21, -- SMART_RTL
         27, -- AUTO_RTL
        }
function is_protected_mode()
   local curr_mode = vehicle:get_mode()
   for i = 1, #protected_mode_array do
     if curr_mode == protected_mode_array[i] then
       return true
     end
   end
   return false
end

local copter_guided_nogps_mode = 20 -- Guided_NoGPS is mode 20 on Copter
local copter_RTL_mode = 6           -- RTL is mode 6 on Copter
local recovery_delay_ms = 3000      -- switch to NEXT_MODE happens this many milliseconds after RC and other failsafes recover

local rc_bad = false                -- true if RC is failing checks
local rc_or_something_bad = true    -- true if RC and/or something is bad, true once both have recovered

local flight_stage = 0  -- 0. wait for good RC, 1=wait for bad RC or something, 2=level vehicle, 3=deadreckon home
local recovery_start_time_ms = 0-- system time RC quality and other failsafes recovered (0 if not recovered)

local curr_alt_below_home = 0      -- altitude from home in meters

local target_yaw = 0    -- deg
local climb_rate = 0    -- m/s
local current_roll = 0  -- deg
local current_pitch = 0 -- deg
local current_yaw = 0   -- deg

local stage1_flight_mode = nil  -- flight mode vehicle was in during stage1 (may be used during recovery)
local time_left = 0
local stage1_start_time_ms  -- system time stage1 started (DR start)
local stage2_start_time_ms  -- system time stage2 started (level vehicle)
local stage3_start_time_ms  -- system time stage3 started (deadreckon home)
local last_print_ms = 0     -- pilot update timer
local interval_ms = 500     -- update at 2hz

local roll_data = {}
local pitch_data = {}
local yaw_data = {}

function update () -- periodic function that will be called

  -- exit immediately if not enabled
  if (enable:get() < 1) then
    return update, 1000
  end

  -- determine if progress update should be sent to user every 5 second
  local now_ms = millis() -- get the time since boot
  local update_user = false
  if (now_ms - last_print_ms > 5000) then
    last_print_ms = now_ms
    update_user = true
  end

  -- check RC failsafe
  local rc_loss = not rc:has_valid_input()
  if rc_bad ~= rc_loss then
    rc_bad = rc_loss
  end

  -- check for RC and/or something going bad
  if not rc_or_something_bad and rc_bad then
    rc_or_something_bad = true
    gcs:send_text(0, "DR: RC and/or something bad")
  end

  -- check for RC and/or something recovery
  if rc_or_something_bad and not rc_bad then
    -- start recovery timer
    if recovery_start_time_ms == 0 then
      recovery_start_time_ms = now_ms
    end
    if (now_ms - recovery_start_time_ms > recovery_delay_ms) then
      rc_or_something_bad = false
      recovery_start_time_ms = 0
      gcs:send_text(0, "DR: RC and/or something recovered")
    end
  end

  -- reset flight_stage when disarmed
  if not arming:is_armed() then
    -- reset buffer when disarmed
    roll_data = {}
    pitch_data = {}
    yaw_data = {}

    flight_stage = 0
    transition_start_time_ms = 0 -- error
    return update, interval_ms
  end

  -- flight_stage 0: wait for
  curr_alt_below_home = -ahrs:get_relative_position_D_home() --with negative
  if (flight_stage == 0) then

    -- wait for RC to be good
    if (rc_or_something_bad) then
      return update, interval_ms
    end
    
    -- wait for altitude from home to pass DR_ENABLE_ALT
    if (curr_alt_below_home >= enable_alt:get()) then
      gcs:send_text(5, "DR: enabled")
      flight_stage = 1
      stage1_start_time_ms = now_ms
    elseif (update_user) then
      gcs:send_text(5, "DR: waiting for alt:" .. tostring(math.floor(curr_alt_below_home)) .. " need:" .. tostring(math.floor(enable_alt:get())))
    end
    return update, interval_ms

  end

  -- flight_stage 1: wait for RC loss
  if (flight_stage == 1) then

    current_roll = math.deg(ahrs:get_roll())
    current_pitch = math.deg(ahrs:get_pitch())
    current_yaw = math.deg(ahrs:get_yaw())

    roll_data[#roll_data + 1] = current_roll
    pitch_data[#pitch_data + 1] = current_pitch
    yaw_data[#yaw_data + 1] = current_yaw
    
    if (rc_or_something_bad and is_protected_mode()) then

      -- change to Guided_NoGPS and initialise stage2
      if (vehicle:set_mode(copter_guided_nogps_mode)) then
        flight_stage = 2
        target_yaw = current_yaw
        stage2_start_time_ms = now_ms
      else
        -- warn user of unexpected failure
        if (update_user) then
          gcs:send_text(5, "DR: failed to change to Guided_NoGPS mode")
        end
      end
    else
      -- store flight mode (may be used during recovery)
      stage1_flight_mode = vehicle:get_mode()
    end
    return update, interval_ms
  end

  -- flight_stage 2: level vehicle for 5 seconds
  if (flight_stage == 2) then
    
    -- allow pilot to retake control
    if (vehicle:get_mode() ~= copter_guided_nogps_mode) then
      gcs:send_text(5, "DR: pilot retook control")
      flight_stage = 1
      return update, interval_ms
    end

    -- level vehicle for 5 seconds
    climb_rate = 0
    vehicle:set_target_angle_and_climbrate(0, 0, target_yaw, 0, false, 0)
    if ((now_ms - stage2_start_time_ms) >= 5000) then
      flight_stage = 3
      stage3_start_time_ms = now_ms
      gcs:send_text(5, "DR: flying back with last known yaw")
    end
    if (update_user) then
      gcs:send_text(5, "DR: leveling vehicle")
    end
    return update, interval_ms
  end

  -- flight_stage 3: deadreckon towards home
  if (flight_stage == 3) then

    -- allow pilot to retake control
    if (vehicle:get_mode() ~= copter_guided_nogps_mode) then
      gcs:send_text(5, "DR: pilot retook control")
      flight_stage = 1
      return update, interval_ms
    end

    -- check for timeout
    local time_elapsed_ms = now_ms - stage3_start_time_ms
    local timeout = (time_elapsed_ms >= (stage2_start_time_ms-stage1_start_time_ms + time_left))

    -- set angle target to roll 0, pitch to lean angle (note: negative is forward), yaw towards home
    lastIndexIn_roll_data = #roll_data
    lastIndexIn_pitch_data = #pitch_data
    lastIndexIn_yaw_data = #yaw_data

    if (lastIndexIn_roll_data == 1) then
      target_roll = math.deg(0)
      target_pitch = math.deg(0)
      target_yaw = target_yaw
    else
      target_roll = roll_data[lastIndexIn_roll_data]
      target_pitch = pitch_data[lastIndexIn_pitch_data]
      target_yaw = yaw_data[lastIndexIn_yaw_data]
    end    

    if (curr_alt_below_home <= 200) then
      climb_rate = 1
    elseif (200 < curr_alt_below_home and curr_alt_below_home < 500) then
      climb_rate = 0.1
    else
      climb_rate = 0
    end

    if (vehicle:set_target_angle_and_climbrate(target_roll, -target_pitch, target_yaw, climb_rate, false, 0)) then
      if (update_user) then
        local time_left_str = ""
        if (not timeout and (fly_timeoout:get() > 0)) then
          time_left_str = " t:" .. tostring(math.max(0, (stage2_start_time_ms - stage1_start_time_ms + time_left - time_elapsed_ms) / 1000))
        end
        gcs:send_text(5, "DR: fly home roll:" .. tostring(math.floor(target_roll)) .. " pit:" .. tostring(-math.floor(target_pitch)) .. " yaw:" .. tostring(math.floor(target_yaw)) .." cr:" .. tostring(math.floor(climb_rate*10)/10) .. time_left_str)
      end
    elseif (update_user) then
      gcs:send_text(0, "DR: failed to set attitude target")
    end

    if (lastIndexIn_roll_data ~= 1)then
      roll_data[lastIndexIn_roll_data] = nil
      pitch_data[lastIndexIn_pitch_data] = nil
      yaw_data[lastIndexIn_yaw_data] = nil
    end   

    -- if RC and something recover or timeout switch to next mode
    if (not rc_or_something_bad) or timeout then

      if not timeout then
        time_left = stage2_start_time_ms - stage1_start_time_ms + time_left - time_elapsed_ms
      end

      local recovery_mode = stage1_flight_mode
      if (next_mode:get() >= 0) then
        recovery_mode = next_mode:get()
      end
      if (recovery_mode == nil) or rc_or_something_bad then
        recovery_mode = copter_RTL_mode
        gcs:send_text(0, "DR: NEXT_MODE=-1 or timeout with rc_or_something_bad but fallingback to RTL")
      end
      -- change to DR_NEXT_MODE
      if (vehicle:set_mode(recovery_mode)) then
        flight_stage = 0
      else
        -- warn user of unexpected failure     
        gcs:send_text(0, "DR: failed to change to mode " .. tostring(recovery_mode))
        recovery_mode = next_mode:get()
        vehicle:set_mode(recovery_mode)
        gcs:send_text(0, "DR: NEXT_MODE AltHold")
        flight_stage = 0
      end
    end
    return update, interval_ms
  end

  -- we should never get here but just in case
  return update, interval_ms  
end

return update()--, 1000   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded
