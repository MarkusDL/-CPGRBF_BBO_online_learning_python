--[[
Callback function for receiving motor positions over ROS
--]]

-- Constants
local socket = require 'socket'
math.randomseed(os.time())

local function roundToNthDecimal(num, n)
    local mult = 10^(n or 0)
    return math.floor(num * mult + 0.5) / mult
end

function mean( t )
    local sum = 0
    local count= 0
    for k,v in pairs(t) do
        if type(v) == 'number' then
            sum = sum + v
            count = count + 1
        end
    end
    return (sum / count)
end

function standardDeviation( t )
    local m
    local vm
    local sum = 0
    local count = 0
    local result
    m = mean( t )
    for k,v in pairs(t) do
        if type(v) == 'number' then
            vm = v - m
            sum = sum + (vm * vm)
            count = count + 1
        end
    end
    result = math.sqrt(sum / (count-1))
    return result
end

function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) * math.cos(2 * math.pi * math.random()) + mean
end

function simGetJointVelocity (jointHandle)
    res,velocity=simGetObjectFloatParameter(jointHandle,2012)
    return  velocity
end

function absmean (prev_avg, x, n)
    return ((prev_avg * n + math.abs(x)) / (n + 1));
end

function absmean_arr (numlist)
    if type(numlist) ~= 'table' then
        print("Error in absmean_arr")
        return numlist
    end
    num = 0
    for numCount = 1, #numlist do
  	num=num+math.abs(numlist[numCount])
    end
    
    return num / #numlist
end

function slip_detector( object_handle, vel_threshold )
    index=0
    objectsInContact,contactPt,forceDirectionAndAmplitude=sim.getContactInfo(sim.handle_all,object_handle,index)

    linearVelocity, angularVelocity = sim.getVelocity(object_handle)
    absLinearVelocity = math.sqrt((linearVelocity[1]*linearVelocity[1]) + (linearVelocity[2]*linearVelocity[2]))

    if objectsInContact then
        if absLinearVelocity > vel_threshold then return 1 end
        return 0
    else return -1
    end
end

function setMotorPositions_cb(msg)
    data = msg.data

    sim.setJointTargetPosition(TC_motor0,data[2])
    sim.setJointTargetPosition(CF_motor0,data[4])
    sim.setJointTargetPosition(FT_motor0,data[6])

    sim.setJointTargetPosition(TC_motor1,data[8])
    sim.setJointTargetPosition(CF_motor1,data[10])
    sim.setJointTargetPosition(FT_motor1,data[12])

    sim.setJointTargetPosition(TC_motor2,data[14])
    sim.setJointTargetPosition(CF_motor2,data[16])
    sim.setJointTargetPosition(FT_motor2,data[18])

    sim.setJointTargetPosition(TC_motor3,data[20])
    sim.setJointTargetPosition(CF_motor3,data[22])
    sim.setJointTargetPosition(FT_motor3,data[24])

    sim.setJointTargetPosition(TC_motor4,data[26])
    sim.setJointTargetPosition(CF_motor4,data[28])
    sim.setJointTargetPosition(FT_motor4,data[30])

    sim.setJointTargetPosition(TC_motor5,data[32])
    sim.setJointTargetPosition(CF_motor5,data[34])
    sim.setJointTargetPosition(FT_motor5,data[36])
end

function TableConcat(t1,t2)
    for i=1,#t2 do
        t1[#t1+1] = t2[i]
    end
    return t1
end


function readFroceSensors(handles)

    local forces = {}
    for i, handle in pairs(handles) do
        result,  forceVector, torqueVector = simReadForceSensor(handle)

        force = math.sqrt(forceVector[1] * forceVector[1] + forceVector[2] * forceVector[2]+ forceVector[3] * forceVector[3])
        forces = TableConcat(forces, {force})
    end
    return forces

end
function resetObjectives_cb(msg)
    local bool = msg.data

    print("reset objectives called")
    
    stepCounter     = 0
    mean_vel        = 0
    mean_jtor       = 0
    mean_jvel       = 0
    mean_jpower     = 0
    mean_pan        = 0
    mean_tilt       = 0
    mean_roll       = 0
    mean_height     = 0
    mean_slip       = 0
    offset_pan      = 0
    update_count    = 0
    bodyfloor_collisions = 0
    leg_collisions = 0
    collisionLast1 = false
    collisionLast2 = false
    collisionLast3 = false
    collisionLast4 = false
    collisionLastBF = false
    boolswitch     = true
    height_arr = {}
    oriX_arr = {}
    oriY_arr = {}
    orientation_arr = {}
    circlebreak = false;
    testParameters = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    largest_dist = 0
    collisions_max = 0
    robotStartPosition = ypos
    
end

--[[
Initialization: Called once at the start of a simulation
--]]
if (sim_call_type==sim.childscriptcall_initialization) then

    nameAddOn=sim.getIntegerSignal("simulationID")

    print("************")
    print("MORF: "..nameAddOn)
    print("************")

    stepCounter     = 0
    mean_vel        = 0
    mean_jtor       = 0
    mean_jvel       = 0
    mean_jpower     = 0
    mean_pan        = 0
    mean_tilt       = 0
    mean_roll       = 0
    mean_height     = 0
    mean_slip       = 0
    offset_pan      = 0
    update_count    = 0
    bodyfloor_collisions = 0
    leg_collisions = 0
    collisionLast1 = false
    collisionLast2 = false
    collisionLast3 = false
    collisionLast4 = false
    collisionLastBF = false
    boolswitch     = true
    height_arr = {}
    oriX_arr = {}
    oriY_arr = {}
    orientation_arr = {}
    circlebreak = false;
    testParameters = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    largest_dist = 0
    collisions_max = 0
    robotStartPosition = 0
    ypos = 0

    -- Create all handles
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    if nameAddOn == nil or nameAddOn == '' then
        nameAddOnSpecial = ''
    else
        nameAddOnSpecial = "" --"#"..nameAddOn
    end

    TC_motor0=sim.getObjectHandle("TC0"..nameAddOnSpecial)    -- Handle of the TC motor
    TC_motor1=sim.getObjectHandle("TC1"..nameAddOnSpecial)    -- Handle of the TC motor
    TC_motor2=sim.getObjectHandle("TC2"..nameAddOnSpecial)    -- Handle of the TC motor
    TC_motor3=sim.getObjectHandle("TC3"..nameAddOnSpecial)    -- Handle of the TC motor
    TC_motor4=sim.getObjectHandle("TC4"..nameAddOnSpecial)    -- Handle of the TC motor
    TC_motor5=sim.getObjectHandle("TC5"..nameAddOnSpecial)    -- Handle of the TC motor

    CF_motor0=sim.getObjectHandle("CF0"..nameAddOnSpecial)    -- Handle of the CF motor
    CF_motor1=sim.getObjectHandle("CF1"..nameAddOnSpecial)    -- Handle of the CF motor
    CF_motor2=sim.getObjectHandle("CF2"..nameAddOnSpecial)    -- Handle of the CF motor
    CF_motor3=sim.getObjectHandle("CF3"..nameAddOnSpecial)    -- Handle of the CF motor
    CF_motor4=sim.getObjectHandle("CF4"..nameAddOnSpecial)    -- Handle of the CF motor
    CF_motor5=sim.getObjectHandle("CF5"..nameAddOnSpecial)    -- Handle of the CF motor

    FT_motor0=sim.getObjectHandle("FT0"..nameAddOnSpecial)    -- Handle of the FT motor
    FT_motor1=sim.getObjectHandle("FT1"..nameAddOnSpecial)    -- Handle of the FT motor
    FT_motor2=sim.getObjectHandle("FT2"..nameAddOnSpecial)    -- Handle of the FT motor
    FT_motor3=sim.getObjectHandle("FT3"..nameAddOnSpecial)    -- Handle of the FT motor
    FT_motor4=sim.getObjectHandle("FT4"..nameAddOnSpecial)    -- Handle of the FT motor
    FT_motor5=sim.getObjectHandle("FT5"..nameAddOnSpecial)    -- Handle of the FT motor

    tipHandles = { sim.getObjectHandle("tip_dyn0"..nameAddOnSpecial),
                   sim.getObjectHandle("tip_dyn1"..nameAddOnSpecial),
                   sim.getObjectHandle("tip_dyn2"..nameAddOnSpecial),
                   sim.getObjectHandle("tip_dyn3"..nameAddOnSpecial),
                   sim.getObjectHandle("tip_dyn4"..nameAddOnSpecial),
                   sim.getObjectHandle("tip_dyn5"..nameAddOnSpecial)}

    tibiaHandles ={sim.getObjectHandle("tibia_dyn0"..nameAddOnSpecial),
                   sim.getObjectHandle("tibia_dyn1"..nameAddOnSpecial),
                   sim.getObjectHandle("tibia_dyn2"..nameAddOnSpecial),
                   sim.getObjectHandle("tibia_dyn3"..nameAddOnSpecial),
                   sim.getObjectHandle("tibia_dyn4"..nameAddOnSpecial),
                   sim.getObjectHandle("tibia_dyn5"..nameAddOnSpecial)}

    distanceSegment12=sim.addDrawingObject(sim.drawing_lines,4,0,-1,1,{0,1,0})
    distanceSegment23=sim.addDrawingObject(sim.drawing_lines,4,0,-1,1,{0,1,0})

    distanceSegment45=sim.addDrawingObject(sim.drawing_lines,4,0,-1,1,{0,1,0})
    distanceSegment56=sim.addDrawingObject(sim.drawing_lines,4,0,-1,1,{0,1,0})

    force_leg0 = sim.getObjectHandle("3D_force0"..nameAddOnSpecial)
    force_leg1 = sim.getObjectHandle("3D_force1"..nameAddOnSpecial)
    force_leg2 = sim.getObjectHandle("3D_force2"..nameAddOnSpecial)
    force_leg3 = sim.getObjectHandle("3D_force3"..nameAddOnSpecial)
    force_leg4 = sim.getObjectHandle("3D_force4"..nameAddOnSpecial)
    force_leg5 = sim.getObjectHandle("3D_force5"..nameAddOnSpecial)

    IMU=sim.getObjectHandle("Imu"..nameAddOnSpecial)
    --distHandle_leg01=sim.getDistanceHandle("leg01"..nameAddOnSpecial)
    --distHandle_leg12=sim.getDistanceHandle("leg12"..nameAddOnSpecial)
    --distHandle_leg34=sim.getDistanceHandle("leg34"..nameAddOnSpecial)
    --distHandle_leg45=sim.getDistanceHandle("leg45"..nameAddOnSpecial)
--
    --distHandle_leg0s=sim.getDistanceHandle("legs0"..nameAddOnSpecial)
    --distHandle_leg1s=sim.getDistanceHandle("legs1"..nameAddOnSpecial)
    --distHandle_leg2s=sim.getDistanceHandle("legs2"..nameAddOnSpecial)
    --distHandle_leg3s=sim.getDistanceHandle("legs3"..nameAddOnSpecial)
    --distHandle_leg4s=sim.getDistanceHandle("legs4"..nameAddOnSpecial)
    --distHandle_leg5s=sim.getDistanceHandle("legs5"..nameAddOnSpecial)
--
    --distHandle_BF   =sim.getDistanceHandle("bodyfloor"..nameAddOnSpecial)

    previousTime=0

    morfHexapod=sim.getObjectHandle("morfHexapod"..nameAddOnSpecial)

    -- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    if simROS then
    	pluginNotFound = false
    	print("simROS plugin found in controller")
    end
    --while moduleName do
    --    moduleName,moduleVersion=sim.getModuleName(index)
    --    if (moduleName=='RosInterface') then
    --        pluginNotFound=false
    --   end
    --    index=index+1
    --end
    
    if (pluginNotFound) then
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        printToConsole('[ERROR] The RosInterface was not found.')
    end

    -- If found then start the subscribers and publishers
    if (not pluginNotFound) then
        local simulationTimeTopicName='sim_control'..nameAddOn..'/simulationTime'
        local startSimulationName='sim_control'..nameAddOn..'/startSimulation'
        local pauseSimulationName='sim_control'..nameAddOn..'/pauseSimulation'
        local stopSimulationName='sim_control'..nameAddOn..'/stopSimulation'
        local enableSyncModeName='sim_control'..nameAddOn..'/enableSyncMode'
        local triggerNextStepName='sim_control'..nameAddOn..'/triggerNextStep'
        local simulationStepDoneName='sim_control'..nameAddOn..'/simulationStepDone'
        local simulationStateName='sim_control'..nameAddOn..'/simulationState'
        local resetObjectivesName = 'sim_control'..nameAddOn..'/resetObjectives'

        local terminateControllerName='sim_control'..nameAddOn..'/terminateController'
        local MotorTopicName='morf_sim'..nameAddOn..'/multi_joint_command'
        local jointPositionsName='morf_sim'..nameAddOn..'/joint_positions'
        local jointTorquesName='morf_sim'..nameAddOn..'/joint_torques'
        local jointVelocitiesName='morf_sim'..nameAddOn..'/joint_velocities'
        local legForceTopicName = 'morf_sim'..nameAddOn..'/leg_forces'
        local testParametersName='sim_control'..nameAddOn..'/testParameters'
        
        local objectPositionHeadingName='morf_sim'..nameAddOn..'/position_heading'

        local addonName='a_girl_has_no_name'
        --print(nameAddOn)
        if nameAddOn ~= nil and nameAddOn ~= '' then
            addonName = nameAddOn
        end

        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..MotorTopicName,'std_msgs/Float32MultiArray','setMotorPositions_cb')
        ResetObjectivesSub=simROS.subscribe('/'..resetObjectivesName,'std_msgs/Bool','resetObjectives_cb')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        jointPositionsPub=simROS.advertise('/'..jointPositionsName,'std_msgs/Float32MultiArray')
        jointTorquesPub=simROS.advertise('/'..jointTorquesName,'std_msgs/Float32MultiArray')
        jointVelocitiesPub=simROS.advertise('/'..jointVelocitiesName,'std_msgs/Float32MultiArray')
        imuEulerPub=simROS.advertise('/morf_sim'..nameAddOn..'/euler','geometry_msgs/Vector3')
        testParametersPub=simROS.advertise('/'..testParametersName,'std_msgs/Float32MultiArray')
        objectPositionHeadingPub=simROS.advertise('/'..objectPositionHeadingName,'std_msgs/Float32MultiArray')
        legForcePub=simROS.advertise('/'..legForceTopicName,'std_msgs/Float32MultiArray')

        if nameAddOn == nil or nameAddOn == '' then
            clockPub=simROS.advertise('/clock','rosgraph_msgs/Clock')
        end

        local rollout=sim.getIntegerSignal("rollout")
        if rollout == nil or rollout == '' then
            rollout = 0
        end

        -- Start the client application (c++ node)
	--CPGRBFN_path = sim.getStringParameter(sim.stringparam_scene_path)
        --os.execute(CPGRBFN_path.."/../interfaces/morf/sim/build_dir/bin/morf_controller".." "..MotorTopicName.." "..simulationTimeTopicName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..jointPositionsName.." "..jointTorquesName.." "..jointVelocitiesName.." "..testParametersName.." "..addonName.." "..rollout.." "..CPGRBFN_path.." ".."&")
        --if (result==false) then
        --    sim.displayDialog('Error','External ROS-Node not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        --    printToConsole('[ ERROR] External ROS-Node not found')
        --else
        --    print('[ INFO] C++ controller started')
        --end
    end

    -- WAIT FOR C++ CONTROLLER TO LAUNCH
    --print("Waiting for c++ controller to launch")
    --socket.sleep(1.5)
    --simROS.publish(terminateControllerPub,{data=false})

    printToConsole('[ INFO] Initialized simulation')
end

--[[
Actuation: This part will be executed in each simulation step
--]]
if (sim_call_type==sim.childscriptcall_actuation) then
    -- Publish
    simROS.publish(terminateControllerPub,{data=false})
    if nameAddOn == nil or nameAddOn == '' then
        simROS.publish(clockPub,{clock=simGetSimulationTime()})
    end
end

--[[
Sensing: This part will be executed in each simulation step
--]]
if (sim_call_type==sim.childscriptcall_sensing) then
    -- Publish
    position_array  ={  simGetJointPosition(TC_motor0),simGetJointPosition(CF_motor0),simGetJointPosition(FT_motor0),
                        simGetJointPosition(TC_motor1),simGetJointPosition(CF_motor1),simGetJointPosition(FT_motor1),
                        simGetJointPosition(TC_motor2),simGetJointPosition(CF_motor2),simGetJointPosition(FT_motor2),
                        simGetJointPosition(TC_motor3),simGetJointPosition(CF_motor3),simGetJointPosition(FT_motor3),
                        simGetJointPosition(TC_motor4),simGetJointPosition(CF_motor4),simGetJointPosition(FT_motor4),
                        simGetJointPosition(TC_motor5),simGetJointPosition(CF_motor5),simGetJointPosition(FT_motor5) }

    velocity_array  ={  simGetJointVelocity(TC_motor0),simGetJointVelocity(CF_motor0),simGetJointVelocity(FT_motor0),
                        simGetJointVelocity(TC_motor1),simGetJointVelocity(CF_motor1),simGetJointVelocity(FT_motor1),
                        simGetJointVelocity(TC_motor2),simGetJointVelocity(CF_motor2),simGetJointVelocity(FT_motor2),
                        simGetJointVelocity(TC_motor3),simGetJointVelocity(CF_motor3),simGetJointVelocity(FT_motor3),
                        simGetJointVelocity(TC_motor4),simGetJointVelocity(CF_motor4),simGetJointVelocity(FT_motor4),
                        simGetJointVelocity(TC_motor5),simGetJointVelocity(CF_motor5),simGetJointVelocity(FT_motor5) }

    torque_array    ={  simGetJointForce(TC_motor0),simGetJointForce(CF_motor0),simGetJointForce(FT_motor0),
                        simGetJointForce(TC_motor1),simGetJointForce(CF_motor1),simGetJointForce(FT_motor1),
                        simGetJointForce(TC_motor2),simGetJointForce(CF_motor2),simGetJointForce(FT_motor2),
                        simGetJointForce(TC_motor3),simGetJointForce(CF_motor3),simGetJointForce(FT_motor3),
                        simGetJointForce(TC_motor4),simGetJointForce(CF_motor4),simGetJointForce(FT_motor4),
                        simGetJointForce(TC_motor5),simGetJointForce(CF_motor5),simGetJointForce(FT_motor5) }


    force_array     = readFroceSensors({force_leg0, force_leg1, force_leg2, force_leg3, force_leg4, force_leg5})--{  simReadForceSensor(force_leg0),simReadForceSensor(force_leg1), simReadForceSensor(force_leg2),
                     --   simReadForceSensor(force_leg3),simReadForceSensor(force_leg4), simReadForceSensor(force_leg5)}

    -- **************** --
    -- Fitness feedback --
    -- **************** --
    linearVelocity, aVelocity=sim.getObjectVelocity(IMU) -- m/s
    objectPosition = sim.getObjectPosition(IMU,-1)
    objectOrientation = sim.getObjectOrientation(IMU,-1)

    ---- Mean velocity of robot
    mean_vel = absmean(mean_vel, linearVelocity[1], update_count)

    ---- Mean power
    mean_jtor   = absmean(mean_jtor, absmean_arr(torque_array), update_count)
    mean_jvel   = absmean(mean_jvel, absmean_arr(velocity_array), update_count)
    mean_jpower = absmean(mean_jpower, mean_jtor * mean_jvel, update_count)

    ---- Orientation / Stability
    mean_roll   = absmean(mean_roll, objectOrientation[1], update_count)
    mean_tilt   = absmean(mean_tilt, objectOrientation[2], update_count)
    mean_pan    = absmean(mean_pan, objectOrientation[3]-offset_pan, update_count)

    ---- Position
    table.insert(height_arr, objectPosition[3])

    ---- Remove transient period
    if simGetSimulationTime() < 1 then
        mean_roll=0
        mean_tilt=0
        mean_pan =0
        mean_height=0
        offset_pan = objectOrientation[3]
    elseif boolswitch then
        boolswitch = false
        -- Release the robot
        simSetObjectInt32Parameter(morfHexapod, sim_shapeintparam_static, 0.0)
    end

    ---- Distance between legs
    --max_detect_interleg     = 0.1
    --max_detect_intraleg     = 0.005
    --max_detect_bodyfloor    = 0.03
    --collisionState = {max_detect_interleg,max_detect_interleg,max_detect_interleg,max_detect_interleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_intraleg,max_detect_bodyfloor}
----

    ------ inter leg.
    --result, distance_leg01=sim.handleDistance(distHandle_leg01) -- m
    --if distance_leg01 == nil then collisionState[1]=max_detect_interleg else collisionState[1] = distance_leg01 end
    --result, distance_leg12=sim.handleDistance(distHandle_leg12) -- m
    --if distance_leg12 == nil then collisionState[2]=max_detect_interleg else collisionState[2] = distance_leg12 end
    --result, distance_leg34=sim.handleDistance(distHandle_leg34) -- m
    --if distance_leg34 == nil then collisionState[3]=max_detect_interleg else collisionState[3] = distance_leg34 end
    --result, distance_leg45=sim.handleDistance(distHandle_leg45) -- m
    --if distance_leg45 == nil then collisionState[4]=max_detect_interleg else collisionState[4] = distance_leg45 end
--
    --print(distance_leg01)
----
    ------ intra leg.
    --result, distance_leg0s=sim.handleDistance(distHandle_leg0s) -- m
    --if distance_leg0s == nil then collisionState[5]=max_detect_intraleg else collisionState[5] = distance_leg0s end
    --result, distance_leg1s=sim.handleDistance(distHandle_leg1s) -- m
    --if distance_leg1s == nil then collisionState[6]=max_detect_intraleg else collisionState[6] = distance_leg1s end
    --result, distance_leg2s=sim.handleDistance(distHandle_leg2s) -- m
    --if distance_leg2s == nil then collisionState[7]=max_detect_intraleg else collisionState[7] = distance_leg2s end
    --result, distance_leg3s=sim.handleDistance(distHandle_leg3s) -- m
    --if distance_leg3s == nil then collisionState[8]=max_detect_intraleg else collisionState[8] = distance_leg3s end
    --result, distance_leg4s=sim.handleDistance(distHandle_leg4s) -- m
    --if distance_leg4s == nil then collisionState[9]=max_detect_intraleg else collisionState[9] = distance_leg4s end
    --result, distance_leg5s=sim.handleDistance(distHandle_leg5s) -- m
    --if distance_leg5s == nil then collisionState[10]=max_detect_intraleg else collisionState[10] = distance_leg5s end
----
    ------ body floor.
    --result, distance_BF=sim.handleDistance(distHandle_BF) -- m
    --if distance_BF == nil then collisionState[11]=max_detect_bodyfloor else collisionState[8] = distance_BF end
----
    --collisionState[1] = 1 - (collisionState[1])   / ( max_detect_interleg ) -- leg01
    --collisionState[2] = 1 - (collisionState[2])   / ( max_detect_interleg ) -- leg12
    --collisionState[3] = 1 - (collisionState[3])   / ( max_detect_interleg ) -- leg34
    --collisionState[4] = 1 - (collisionState[4])   / ( max_detect_interleg ) -- leg45
    --collisionState[5] = 1 - (collisionState[5])   / ( max_detect_intraleg ) -- leg0s
    --collisionState[6] = 1 - (collisionState[6])   / ( max_detect_intraleg ) -- leg1s
    --collisionState[7] = 1 - (collisionState[7])   / ( max_detect_intraleg ) -- leg2s
    --collisionState[8] = 1 - (collisionState[8])   / ( max_detect_intraleg ) -- leg3s
    --collisionState[9] = 1 - (collisionState[9])   / ( max_detect_intraleg ) -- leg4s
    --collisionState[10] = 1 - (collisionState[10]) / ( max_detect_intraleg ) -- leg5s
    --collisionState[11] = 1 - (collisionState[11]) / ( max_detect_bodyfloor) -- bodyfloor
----
    ------print(collisionState)
    --max_dist = math.max(unpack(collisionState))
    --if max_dist > 1 then print("[ ERROR]: Please set max dist correctly") end
    --if max_dist > collisions_max then collisions_max = max_dist end

    positionRobot=sim.getObjectPosition(morfHexapod, -1)
    ypos = positionRobot[2]
    distance = -ypos+robotStartPosition -- use negative world y axis

    slip_results = {}
    for i = 1, table.getn(tipHandles), 1 do
        table.insert(slip_results, slip_detector( tipHandles[i], 0.025))
    end
    max_slip = math.max(unpack(slip_results))

    if max_slip ~= -1 then
        mean_slip    = absmean(mean_slip, max_slip, update_count)
    end

    testParameters[2]  = mean_slip -- x orientation
    testParameters[3]  = mean_tilt -- x orientation
    testParameters[4]  = mean_roll -- y orientation
    testParameters[5]  = mean_pan -- Heading = z orientation
    testParameters[6]  = collisions_max
    testParameters[7]  = 0.0 -- was body floor collisions (now included in collisions_max)
    testParameters[8]  = mean_jpower
    testParameters[9]  = mean_vel
    testParameters[10] = distance
    testParameters[11] = standardDeviation(height_arr)

    -- Step counters for sync with controller and internal functions
    update_count = update_count + 1
    stepCounter = stepCounter + 1
    testParameters[12] = stepCounter

    simROS.publish(jointPositionsPub,{data=position_array})
    simROS.publish(jointVelocitiesPub,{data=velocity_array})
    simROS.publish(jointTorquesPub,{data=torque_array})
    simROS.publish(testParametersPub,{data=testParameters})
    simROS.publish(legForcePub,{data=force_array})

    local objetPositionHeading = {unpack(positionRobot)}
    for I = 1,#objectOrientation do
        objetPositionHeading[#positionRobot+I] = objectOrientation[I]
    end

    simROS.publish(objectPositionHeadingPub,{data=objetPositionHeading})

    
end

--[[
Clean up: This part will be executed one time just before a simulation ends
--]]
if (sim_call_type==sim.childscriptcall_cleanup) then

    print("+====Objectives====+")
    print("Mean tilt:\t"    .. roundToNthDecimal(mean_tilt,4))
    print("Mean roll:\t"    .. roundToNthDecimal(mean_roll,4))
    print("Mean heading:\t" .. roundToNthDecimal(mean_pan,4))
    print("Mean Height:\t"  .. roundToNthDecimal(mean_height, 4))
    print("Mean power:\t"   .. roundToNthDecimal(mean_jpower,4))
    print("Robot Coll..:\t" .. roundToNthDecimal(collisions_max,5))
    print("Slipping:\t"     .. roundToNthDecimal(mean_slip,4))
    print("Distance:\t"     .. roundToNthDecimal(distance,4))
    print("+=================+")

    -- Terminate Controller
    --simROS.publish(terminateControllerPub,{data=true})
    --sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,true)
    --simSetObjectInt32Parameter(morfHexapod, sim_shapeintparam_static, 1)

    -- Wait for the signal to reach the node
    waitTimer=0
    while( waitTimer < 1000 ) do
        waitTimer = waitTimer+1
        if nameAddOn == nil or nameAddOn == '' then
            simROS.publish(clockPub,{clock=simGetSimulationTime()+waitTimer})
        end
        simROS.publish(terminateControllerPub,{data=true})
    end

    -- Terminate remaining local notes
    simROS.shutdownSubscriber(MotorSub)
    if nameAddOn == nil or nameAddOn == '' then
        simROS.shutdownPublisher(clockPub)
    end
    simROS.shutdownPublisher(jointTorquesPub)
    simROS.shutdownPublisher(jointVelocitiesPub)
    simROS.shutdownPublisher(jointPositionsPub)
    simROS.shutdownPublisher(testParametersPub)
    simROS.shutdownPublisher(terminateControllerPub)
    simROS.shutdownPublisher(imuEulerPub)
    
    simROS.shutdownPublisher(objectPositionHeadingPub)

    printToConsole('[ INFO] Lua child script stopped')
end
