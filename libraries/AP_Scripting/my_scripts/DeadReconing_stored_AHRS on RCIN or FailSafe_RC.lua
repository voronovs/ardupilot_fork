-- Copter attempts to fly home using dead reckoning if the GPS quality deteriorates or an EKF failsafe triggers
--
-- CAUTION: This script only works for Copter 4.3 (and higher)
-- this script checks for low GPS quality and/or an EKF failsafe and if either occurs, flies in the last known direction towards home
--
-- DR_ENABLE : 1 = enabled, 0 = disabled
-- DR_ENABLE_DIST : distance from home (in meters) beyond which the dead reckoning will be enabled
-- DR_GPS_SACC_MAX : GPS speed accuracy maximum, above which deadreckoning home will begin (default is 0.8).  Lower values trigger with good GPS quality, higher values will allow poorer GPS before triggering. Set to 0 to disable use of GPS speed accuracy.
-- DR_GPS_SAT_MIN : GPS satellite count threshold below which deadreckoning home will begin (default is 6).  Higher values trigger with good GPS quality, Lower values trigger with worse GPS quality. Set to 0 to disable use of GPS satellite count.
-- DR_GPS_TRIGG_SEC : GPS checks must fail for this many seconds before dead reckoning will be triggered.
-- DR_FLY_ANGLE : lean angle (in degrees) during deadreckoning.  Most vehicles reach maximum speed at 22deg
-- DR_FLY_ALT_MIN : min alt (above home in meters) during deadreckoning. zero to return at current alt
-- DR_FLY_TIMEOUT : timeout (in seconds).  Vehicle will attempt to switch to NEXT_MODE after this many seconds of deadreckoning.  If it cannot switch modes it will continue in Guided_NoGPS.  Set to 0 to disable timeout
-- DR_NEXT_MODE : flight mode vehicle will change to when GPS / EKF recovers or DR_FLY_TIMEOUT expires.  Default is 6=RTL, see FLTMODE1 parameter description for list of flight mode number.  Set to -1 to return to mode used before deadreckoning was triggered

-- How to use:
--   1. set SCR_ENABLE = 1 to enable scripting (and reboot the autopilot)
--   2. set SCR_HEAP_SIZE to 80000 or higher to allocate enough memory for this script
--   3. set DR_ENABLE = 1 to enable dead reckoning
--   4. optionally set DR_GPS_SACC_MAX and/or DR_GPS_SAT_MIN parameters to adjust how bad the GPS quality must be before triggering
--   5. confirm "DR: waiting for dist (Xm < 50m)" message is displayed on ground station (so you know script is working)
--   6. arm and takeoff to a safe altitude
--   7. fly at least DR_ENABLE_DIST meters from home and confirm "DR: activated!" is displayed on ground station
--
--   If this script senses low GPS quality or an EKF failsafe triggers:
--       - vehicle will change to Guided_NoGPS mode
--       - vehicle will lean in the last known direction of home (see DR_FLY_ANGLE)
--       - if GPS recovers or EKF failsafe is cleared the vehicle will switch to DR_NEXT_MODE (if -1 then it will switch back to the mode in use before the GPS/EKF failure)
--       - if the timeout is surpassed (see DR_FLY_TIMEOUT) the vehicle will try to switch to DR_NEXT_MODE.  If it fails to change it will continue in Guided_NoGPS but keep trying to change mode
--       - the pilot can retake control by switching to an "unprotected" mode like AltHold, Loiter (see "protected_mode_array" below)
--
-- Testing in SITL:
--   a. set map setshowsimpos 1 (to allow seeing where vehicle really is in simulator even with GPS disabled)
--   b. set SIM_GPS_DISABLE = 1 to disable GPS (confirm dead reckoning begins)
--   c. set SIM_GPS_DISABLE = 0 to re-enable GPS
--   d. set SIM_GPS_NUMSAT = 3 to lower simulated satellite count to confirm script triggers
--   e. set DR_GPS_SACC_MAX = 0.01 to lower the threshold and trigger below the simulator value which is 0.04 (remember to set this back after testing!)
--
-- Test on a real vehicle:
--   A. set DR_FLY_TIMEOUT to a low value (e.g. 5 seconds)
--   B. fly the vehicle at least DR_DIST_MIN meters from home and confirm the "DR: activated!" message is displayed
--   C. set GPS1_TYPE = 0 to disable GPS and confirm the vehicle begins deadreckoning after a few seconds
--   D. restore GPS1_TYPE to its original value (normally 1) and confirm the vehicle switches to DR_NEXT_MODE
--   E. restore DR_FLY_TIMEOUT to a higher value for real-world use
-- Note: Instaed of setting GPS1_TYPE, an auxiliary function switch can be setup to disable the GPS (e.g. RC9_OPTION = 65/"Disable GPS")
--
-- Testing that it does not require RC (in SITL):
--   a. set FS_OPTIONS's "Continue if in Guided on RC failsafe" bit
--   b. set FS_GCS_ENABLE = 1 (to enable GCS failsafe otherwise RC failsafe will trigger anyway)
--   c. optionally set SYSID_MYGCS = 77 (or almost any other unused system id) to trick the above check so that GCS failsafe can really be disabled
--   d. set SIM_RC_FAIL = 1 (to simulate RC failure, note vehicle keeps flying)
--   e. set SIM_RC_FAIL = 0 (to simulate RC recovery)
--
-- Test with wind (in SITL)
--   a. SIM_WIND_DIR <-- sets direction wind is coming from
--   b. SIM_WIND_SPD <-- sets wind speed in m/s
--

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: cast-local-type

-- create and initialise parameters
local PARAM_TABLE_KEY = 86  -- parameter table key must be used by only one script on a particular flight controller
assert(param:add_table(PARAM_TABLE_KEY, "DR_", 10), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add DR_ENABLE param')   -- 1 = enabled, 0 = disabled
assert(param:add_param(PARAM_TABLE_KEY, 2, 'ENABLE_DIST', 50), 'could not add DR_ENABLE_DIST param')   -- distance from home (in meters) beyond which the dead reckoning will be enabled
assert(param:add_param(PARAM_TABLE_KEY, 3, 'GPS_SACC_MAX', 0.8), 'could not add DR_GPS_SACC_MAX param') -- GPS speed accuracy max threshold
assert(param:add_param(PARAM_TABLE_KEY, 4, 'GPS_SAT_MIN', 6), 'could not add DR_GPS_SAT_MIN param')  -- GPS satellite count min threshold
assert(param:add_param(PARAM_TABLE_KEY, 5, 'GPS_TRIGG_SEC', 3), 'could not add DR_GPS_TRIGG_SEC parameter') -- GPS checks must fail for this many seconds before dead reckoning will be triggered

assert(param:add_param(PARAM_TABLE_KEY, 6, 'FLY_ANGLE', 15), 'could not add DR_FLY_ANGLE param')    -- lean angle (in degrees) during deadreckoning
assert(param:add_param(PARAM_TABLE_KEY, 7, 'FLY_ALT_MIN', 80), 'could not add DR_FLY_ALT_MIN param') -- min alt above home (in meters) during deadreckoning. zero to return at current alt
assert(param:add_param(PARAM_TABLE_KEY, 8, 'FLY_TIMEOUT', 180), 'could not add DR_FLY_TIMEOUT param')-- deadreckoning timeout (in seconds)
assert(param:add_param(PARAM_TABLE_KEY, 9, 'NEXT_MODE', 2), 'could not add DR_NEXT_MODE param')     -- mode to switch to after GPS recovers or timeout elapses
assert(param:add_param(PARAM_TABLE_KEY, 10, 'ENABLE_ALT', 15), 'could not add DR_ENABLE_ALT param')   -- altitude from home (in meters) beyond which the dead reckoning will be enabled

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
local enable_dist = Parameter("DR_ENABLE_DIST")        -- distance from home (in meters) beyond which the dead reckoning will be enabled

--[[
  // @Param: DR_ENABLE_DIST
  // @DisplayName: Deadreckoning Enable Distance
  // @Description: Distance from home (in meters) beyond which the dead reckoning will be enabled
  // @Units: m
  // @User: Standard
--]]
local enable_alt = Parameter("DR_ENABLE_ALT")        -- distance from home (in meters) beyond which the dead reckoning will be enabled

--[[
  // @Param: DR_GPS_SACC_MAX
  // @DisplayName: Deadreckoning GPS speed accuracy maximum threshold
  // @Description: GPS speed accuracy maximum, above which deadreckoning home will begin (default is 0.8).  Lower values trigger with good GPS quality, higher values will allow poorer GPS before triggering. Set to 0 to disable use of GPS speed accuracy
  // @Range: 0 10
  // @User: Standard
--]]
local gps_speed_acc_max = Parameter("DR_GPS_SACC_MAX") -- GPS speed accuracy max threshold

--[[
  // @Param: DR_GPS_SAT_MIN
  // @DisplayName: Deadreckoning GPS satellite count min threshold
  // @Description: GPS satellite count threshold below which deadreckoning home will begin (default is 6).  Higher values trigger with good GPS quality, Lower values trigger with worse GPS quality. Set to 0 to disable use of GPS satellite count
  // @Range: 0 30
  // @User: Standard
--]]
local gps_sat_count_min = Parameter("DR_GPS_SAT_MIN")  -- GPS satellite count min threshold

--[[
  // @Param: DR_GPS_TRIGG_SEC
  // @DisplayName: Deadreckoning GPS check trigger seconds
  // @Description: GPS checks must fail for this many seconds before dead reckoning will be triggered
  // @Units: s
  // @User: Standard
--]]
local gps_trigger_sec = Parameter("DR_GPS_TRIGG_SEC")  -- GPS checks must fail for this many seconds before dead reckoning will be triggered

--[[
  // @Param: DR_FLY_ANGLE
  // @DisplayName: Deadreckoning Lean Angle
  // @Description: lean angle (in degrees) during deadreckoning
  // @Units: deg
  // @Range: 0 45
  // @User: Standard
--]]
local fly_angle = Parameter("DR_FLY_ANGLE")            -- lean angle (in degrees) during deadreckoning

--[[
  // @Param: DR_FLY_ALT_MIN
  // @DisplayName: Deadreckoning Altitude Min
  // @Description: Copter will fly at at least this altitude (in meters) above home during deadreckoning
  // @Units: m
  // @Range: 0 1000
  // @User: Standard
--]]
local fly_alt_min = Parameter("DR_FLY_ALT_MIN")        -- min alt above home (in meters) during deadreckoning

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
local wpnav_speedup = Parameter("WPNAV_SPEED_UP")      -- maximum climb rate from WPNAV_SPEED_UP
local wpnav_accel_z = Parameter("WPNAV_ACCEL_Z")       -- maximum vertical acceleration from WPNAV_ACCEL_Z

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

local gps_bad = false               -- true if GPS is failing checks
local ekf_bad = false               -- true if EKF failsafe has triggered
local gps_or_ekf_bad = true         -- true if GPS and/or EKF is bad, true once both have recovered
local rc_bad = false                -- true if RC is failing checks
local rc_or_something_bad = true    -- true if RC and/or something is bad, true once both have recovered

local flight_stage = 0  -- 0. wait for good RC, 1=wait for bad RC or something, 2=level vehicle, 3=deadreckon home
local gps_bad_start_time_ms = 0 -- system time GPS quality went bad (0 if not bad)
local rc_bad_start_time_ms = 0 -- system time RC quality went bad (0 if not bad)
local recovery_start_time_ms = 0-- system time RC quality and other failsafes recovered (0 if not recovered)

local home_dist = 0     -- distance to home in meters
local home_yaw = 0      -- direction to home in degrees
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
local interval_ms = 100     -- update at 10hz

local roll_data = {}
local pitch_data = {}
local yaw_data = {}

--local filePath = "RCIN_record.txt"
--local file_clear = io.open(filePath, "w")
--if file_clear then
  --file_clear:close()
  --gcs:send_text(5, "file removed")
--else
  --gcs:send_text(5, "impossible to remove file")
--end

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
  pwm8 = rc:get_pwm(8)
  local rc_loss = not rc:has_valid_input()
  if rc_bad ~= rc_loss then
    rc_bad = rc_loss
  end

  -- check for RC and/or something going bad
  if not rc_or_something_bad and (rc_bad or pwm8 > 1600) then
    rc_or_something_bad = true
    gcs:send_text(0, "DR: RC and/or something bad")
  end

  -- check for RC and/or something recovery
  if rc_or_something_bad and (not rc_bad and pwm8 < 1600) then
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

    -- array of current orientation
    --local orientation_current = {current_roll, current_pitch, current_yaw}
    --local orientation_current_str = table.concat(orientation_current, " ")
    
    --local file = io.open(filePath, "a")

    --if file then
   
      --file:write(orientation_current_str .. "\n")

      --file:close()
    --else
      --gcs:send_text(5, "DR: impossible to open file")
    --end

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

  -- local target_yaw = math.deg(ahrs:get_yaw()) -- fetch the current yaw of the vehicle deg
  -- gcs:send_text(0, "UAV FLY current YAW is " .. target_yaw)
  -- return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
end

return update()--, 1000   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded