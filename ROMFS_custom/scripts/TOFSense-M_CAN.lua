--[[
   Driver for NoopLoop TOFSense-M CAN Version. Can be used as a 1-D RangeFidner or 3-D proximity sensor. Upto 3 CAN devices supported in this script although its easy to extend.
--]]

local update_rate_ms    = 10  -- update rate (in ms) of the driver. 10ms was found to be appropriate

-- Global variables (DO NOT CHANGE)
local param_num_lua_driver_backend = 36         -- parameter number for lua rangefinder
local param_num_lua_prx_backend = 15            -- parameter number for lua proximity
local sensor_setup_done = false

-- Table contains the following info for 3 sensors. If more sensors are needed, this table will need to be increased
-- approportate scritping backend from rngfnd/prx library, true if backend exists, index parsed last from sensor, minimum distance found since index was 0, Param to decide which rngfnd/prx backednd will match to this sensor, param to decide CAN ID of this sensor 
local backend_driver = {
  {lua_driver_backend = nil, sensor_driver_found = false, last_index = 0, min_distance = 0, INSTANCE, CAN_ID},
  {lua_driver_backend = nil, sensor_driver_found = false, last_index = 0, min_distance = 0, INSTANCE, CAN_ID},
  {lua_driver_backend = nil, sensor_driver_found = false, last_index = 0, min_distance = 0, INSTANCE, CAN_ID}
}

local PARAM_TABLE_KEY = 104
local PARAM_TABLE_PREFIX = "TOFSENSE_"

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: TOFSENSE_PRX
  // @DisplayName: TOFSENSE-M to be used as Proximity sensor
  // @Description: Set 0 if sensor is to be used as a 1-D rangefinder (minimum of all distances will be sent, typically used for height detection). Set 1 if it should be used as a 3-D proximity device (Eg. Obstacle Avoidance)
  // @Values: 0:Set as Rangefinder, 1:Set as Proximity sensor
  // @User: Standard
--]]
SET_PRX = bind_add_param('PRX', 1, 1)

--[[
  // @Param: TOFSENSE_NO
  // @DisplayName: TOFSENSE-M Connected
  // @Description: Number of TOFSENSE-M CAN sensors connected
  // @Range: 1 3
  // @User: Standard
--]]
MAX_SENSORS = bind_add_param('NO', 2, 3)

--[[
  // @Param: TOFSENSE_MODE
  // @DisplayName: TOFSENSE-M mode to be used
  // @Description: TOFSENSE-M mode to be used. 0 for 8x8 mode. 1 for 4x4 mode
  // @Values: 0: 8x8 mode, 1: 4x4 mode
  // @User: Standard
--]]
MODE = bind_add_param('MODE', 3, 0)

-- first sensor
--[[
  // @Param: TOFSENSE_INST1
  // @DisplayName: TOFSENSE-M First Instance
  // @Description: First TOFSENSE-M sensors backend Instance. Setting this to 1 will pick the first backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)
  // @Range: 1 3
  // @User: Standard
--]]
backend_driver[1].INSTANCE = bind_add_param('INST1', 4, 1)

--[[
  // @Param: TOFSENSE_ID1
  // @DisplayName: TOFSENSE-M First ID
  // @Description: First TOFSENSE-M sensor ID. Leave this at 0 to accept all IDs and if only one sensor is present. You can change ID of sensor from NAssistant Software
  // @Range: 1 255
  // @User: Standard
--]]
backend_driver[1].CAN_ID = bind_add_param('ID1', 5, 1)

-- second sensor
--[[
  // @Param: TOFSENSE_INST2
  // @DisplayName: TOFSENSE-M Second Instance
  // @Description: Second TOFSENSE-M sensors backend Instance. Setting this to 2 will pick the second backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)
  // @Range: 1 3
  // @User: Standard
--]]
backend_driver[2].INSTANCE = bind_add_param('INST2', 6, 2)

--[[
  // @Param: TOFSENSE_ID2
  // @DisplayName: TOFSENSE-M Second ID
  // @Description: Second TOFSENSE-M sensor ID. This cannot be 0. You can change ID of sensor from NAssistant Software
  // @Range: 1 255
  // @User: Standard
--]]
backend_driver[2].CAN_ID = bind_add_param('ID2', 7, 2)

--third sensor
--[[
  // @Param: TOFSENSE_INST3
  // @DisplayName: TOFSENSE-M Third Instance
  // @Description: Third TOFSENSE-M sensors backend Instance. Setting this to 3 will pick the second backend from PRX_ or RNG_ Parameters (Depending on TOFSENSE_PRX)
  // @Range: 1 3
  // @User: Standard
--]]
backend_driver[3].INSTANCE = bind_add_param('INST3', 8, 3)    --标记错误

--[[
  // @Param: TOFSENSE_ID3
  // @DisplayName: TOFSENSE-M Thir ID
  // @Description: Third TOFSENSE-M sensor ID. This cannot be 0. You can change ID of sensor from NAssistant Software
  // @Range: 1 255
  // @User: Standard
--]]
backend_driver[3].CAN_ID = bind_add_param('ID3', 9, 3)


-- check both CAN device for scripting backend. CAN Buffer length set to fixed 5
local driver = CAN:get_device(5)
if not driver then
  driver = CAN:get_device2(5)
end
if not driver then
  error("No scripting CAN interfaces found")
  return
end

function setup_sensor(sensor, param_num)
  local sensor_count = sensor:num_sensors() -- number of sensors connected
  -- gcs:send_text(0, "sensor_count:"..tostring(sensor_count))
  -- gcs:send_text(0, "MAX_SENSORS:get():"..tostring(MAX_SENSORS:get()))

  if  MAX_SENSORS:get() > 3 then
    error("TOFSENSE: Only 3 devices supported")
  end

  for i = 1, MAX_SENSORS:get() do
    local backends_found = 0
    local sensor_driver_found = false
    local lua_driver_backend
    if (MODE:get() == 0) then  --如果MODE == 0则每个传感器要采集64个数据
      -- 8x8 mode
      backend_driver[i].last_index = 64;
    elseif (MODE:get() == 1) then
      -- 4x4 mode
      backend_driver[i].last_index = 16;
    end

    for j = 0, sensor_count -1 do
      local device = sensor:get_backend(j)
      
      if ((not sensor_driver_found) and  device and (device:type() == param_num)) then --检测是否有device
        -- this is a lua driver
        backends_found = backends_found + 1
        -- gcs:send_text(0, "backends_found:"..tostring(backends_found))
        -- gcs:send_text(0, "backend_driver[i].INSTANCE:get():"..tostring(backend_driver[i].INSTANCE:get()))
        if backends_found == backend_driver[i].INSTANCE:get() then
          -- get the correct instance as we may have multile scripting backends doing different things
          sensor_driver_found = true
          lua_driver_backend = device
          break; -- exit the loop 找到对应ID了
        end
      end
    end
    if not sensor_driver_found then
      -- We can't use this script if user hasn't setup a lua backend
      error(string.format("TOFSENSE: Could not find SCR Backend ".. tostring(i)))
      return
    end
    backend_driver[i].sensor_driver_found = true
    backend_driver[i].lua_driver_backend = lua_driver_backend  --把对应的设备ID给到后端
    -- gcs:send_text(0, "i: ".. tostring(lua_driver_backend))
  end

end

-- get yaw and pitch of the pixel based message index.
function convert_to_angle(index,instance) --更改程序为135°范围
  -- The distances are sent in either a 4x4 or 8x8 grid. The horizontal and vertical FOV are 45 degrees so we can work out the angles
  local index_row_max = 8
  if (MODE:get() ~= 0) then
    index_row_max = 4
  end
  local angle_division = 45/index_row_max --45°划分成4或8份
  local horizontal_index = (index) % index_row_max --计算水平索引
  local vertical_index = math.floor(index / index_row_max) --计算垂直索引
  local yaw = -22.5 + (horizontal_index*angle_division) 
  local pitch = -22.5 + (vertical_index*angle_division)         
  ---- CANID :    2   1   3  角度分布
  if instance == 2 then
    -- flip the yaw for the second sensor
    yaw = yaw - 45;
  end
  if instance == 3 then
    -- flip the yaw for the third sensor
    yaw = yaw + 45
  end
  return yaw, pitch
end

-- send the message down to proximity library. This needs to be a 3D vector
function sent_prx_message(prx_backend, dist, yaw_deg, pitch_deg, push_to_boundary)
  if (dist > 0) then
    prx_backend:set_distance_min_max(0,4)
    prx_backend:handle_script_distance_msg(dist, yaw_deg, pitch_deg, push_to_boundary)
  end
end

-- send the message down to proximity library. This needs to be a single distance
function send_rfnd_message(rfnd_backend, dist)
    if dist > 0 and (SET_PRX:get() == 0) then
      local sent_successfully = rfnd_backend:handle_script_msg(dist)
      if not sent_successfully then
        -- This should never happen as we already checked for a valid configured lua backend above
        gcs:send_text(0, string.format("RFND Lua Script Error"))
      end
  end
end

-- get the correct instace from parameters according to the CAN ID received
function get_instance_from_CAN_ID(frame)
  for i = 1, MAX_SENSORS:get() do
    if ((uint32_t(frame:id() - 0x200)) ==  uint32_t(backend_driver[i].CAN_ID:get())) then
       return i
    end
  end
  return 0
end

-- this is the loop which periodically runs
function update()

  -- setup the sensor according to user preference of using proximity library or rangefinder
  -- gcs:send_text(0, "SET_PRX:get()"..tostring(SET_PRX:get()));
  if not sensor_setup_done then
    if SET_PRX:get() == 0 then
      setup_sensor(rangefinder, param_num_lua_driver_backend)
    else
      setup_sensor(proximity, param_num_lua_prx_backend)
    end
    sensor_setup_done = true
  end

  -- read frame if available
  local frame = driver:read_frame()
  if not frame then
    return
  end

  local instance = 0
  if ((backend_driver[1].CAN_ID:get() ~= 0)) then
    instance = get_instance_from_CAN_ID(frame)
    if (instance == 0) then
      -- wrong ID
      return
    end
  else
    -- Simply accept any ID
    instance = 1
  end

  -- Correct ID, so parse the data
  local distance = ((frame:data(0) | frame:data(1)<<8 | frame:data(2)<<16)) / 1000
  gcs:send_text(0, "frame:data(0):"..tostring(frame:data(0)).." frame:data(1):"..tostring(frame:data(1)).." frame:data(2):"..tostring(frame:data(2)))
  local status = frame:data(3)
  local index = frame:data(6)
  local update_rfnd = false
  gcs:send_text(0, "Distance:"..tostring(distance).." Instance:"..tostring(instance))
  
  if (index < backend_driver[instance].last_index) then
    -- One cycle of data has come. Lets update all backends involved
    if SET_PRX:get() == 1 then
      backend_driver[instance].lua_driver_backend:
      ()
    else
      update_rfnd = true
    end
  end
  -- status定位状态定义如下：
  -- 0  测量数据可用
  -- 1  信号强度过低
  -- 2  阶段目标
  -- 3  目标噪声估值过高
  -- 4  目标一致性检测失败
  -- 5  测量数据未更新
  -- 6  未执行环绕操作 (通常为第一次测量)
  -- 7  速率不一致
  -- 8  当前目标信号强度低
  -- 9  大脉冲有效范围（可能是由于合并的目标）
  -- 10  测量数据可用，但在之前的检测中未检测到目标
  -- 11  测量结果不一致
  -- 12  目标被模糊
  -- 13  检测到目标但数据不一致，通常发生在次要目标存在时
  -- 255  未检测到目标

  if status == 0 then
    -- Status is healthy
    if (SET_PRX:get() == 1) then
      -- Send 3D data to Proximity Library
      local yaw, pitch =  convert_to_angle(index,instance)
      sent_prx_message(backend_driver[instance].lua_driver_backend, distance, yaw, pitch, false)
    end
    if (backend_driver[instance].min_distance == 0 or distance < backend_driver[instance].min_distance) then
      -- store min data incase user wants to use it as a 1-D RangeFinder
      backend_driver[instance].min_distance = distance
    end
  end

  if (update_rfnd) then
    send_rfnd_message(backend_driver[instance].lua_driver_backend, backend_driver[instance].min_distance)
    -- reset
    backend_driver[instance].min_distance = 0
  end
end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()

  local success, err = pcall(update)
  if not success then
      gcs:send_text(0, "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
  end
  return protected_wrapper, update_rate_ms
end

-- start running update loop
gcs:send_text(0, "TOFSENSE-M CAN Driver started")
return protected_wrapper()
