---
--- Created by mat.
--- DateTime: 5/22/19 1:14 PM
---

function sysCall_init()
    -- Check if the RosInterface is available:
    moduleName=0
    moduleVersion=0
    index=0
    pluginFound=false
    print("Checking modules")
    
    if simROS then
        pluginFound=true
    end
    
    --while moduleName do
    --    moduleName,moduleVersion=sim.getModuleName(index)
    --    print(moduleName)
    --    if (moduleName=='RosInterface') then
    --        pluginFound=true
    --        print("Plugin found")
    --    end
    --    index=index+1
    --end


    -- Find name add on
    file = "simulationID.txt"
    local f = assert(io.open(file, "rb"))
    local content = tonumber(f:read("*all"))
    f:close()

    print("--SIMULATION ID--")
    print(content)
    result=sim.setIntegerSignal("simulationID", content)

    nameAddOn = content

    if pluginFound then
        startSub=simROS.subscribe('/sim_control'..nameAddOn..'/startSimulation', 'std_msgs/Bool', 'startSimulation_callback')
        pauseSub=simROS.subscribe('/sim_control'..nameAddOn..'/pauseSimulation', 'std_msgs/Bool', 'pauseSimulation_callback')
        stopSub=simROS.subscribe('/sim_control'..nameAddOn..'/stopSimulation', 'std_msgs/Bool', 'stopSimulation_callback')
        enableSynModeSub=simROS.subscribe('/sim_control'..nameAddOn..'/enableSyncMode', 'std_msgs/Bool', 'enableSyncMode_callback')
        triggerNextStepSub=simROS.subscribe('/sim_control'..nameAddOn..'/triggerNextStep', 'std_msgs/Bool', 'triggerNextStep_callback')
        blackoutSub=simROS.subscribe('/sim_control'..nameAddOn..'/blackout', 'std_msgs/Bool', 'blackout_callback')

        simStepDonePub=simROS.advertise('/sim_control'..nameAddOn..'/simulationStepDone', 'std_msgs/Bool')
        simStatePub=simROS.advertise('/sim_control'..nameAddOn..'/simulationState','std_msgs/Int32')
        simTimePub=simROS.advertise('/sim_control'..nameAddOn..'/simulationTime','std_msgs/Float32')
        auxPub=simROS.advertise('/sim_control'..nameAddOn..'/privateMsgAux', 'std_msgs/Bool')
        auxSub=simROS.subscribe('/sim_control'..nameAddOn..'/privateMsgAux', 'std_msgs/Bool', 'aux_callback')

        rosInterfaceSynModeEnabled=false
    else
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end
    
    print("Started pubs and subs")
end

function startSimulation_callback(msg)
    -- Send signal here!
    --result=sim.setIntegerSignal("rollout",msg.data)
    sim.startSimulation()
end

function blackout_callback(msg)
    data=msg.data
    sim.setBoolParameter(sim.boolparam_display_enabled,data)
end

function pauseSimulation_callback(msg)
    sim.stopSimulation()
    os.execute("sleep " .. tonumber(0.3)) -- SIM TAKES SOME STEPS TO STOP
    --result=sim.setIntegerSignal("rollout",msg.data)
    sim.startSimulation()
end

function stopSimulation_callback(msg)
    sim.stopSimulation()
end

function enableSyncMode_callback(msg)
    rosInterfaceSynModeEnabled=msg.data
    sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,rosInterfaceSynModeEnabled)
end

function triggerNextStep_callback(msg)
    data=msg.data
    if data==true then
        sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,false)
    end
end

function aux_callback(msg)
    simROS.publish(simStepDonePub,{data=true})
end

function publishSimState()
    local state=0 -- simulation not running
    local s=sim.getSimulationState()
    if s==sim.simulation_paused then
        state=2 -- simulation paused
    elseif s==sim.simulation_stopped then
        state=0 -- simulation stopped
    else
        state=1 -- simulation running
    end
    simROS.publish(simStatePub,{data=state})
end


function sysCall_nonSimulation()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_actuation()
    if pluginFound then
        publishSimState()
        simROS.publish(simTimePub,{data=sim.getSimulationTime()})
    end
end

function sysCall_sensing()
    if pluginFound then
        simROS.publish(auxPub,{data=true})
        sim.setBoolParameter(sim.boolparam_rosinterface_donotrunmainscript,rosInterfaceSynModeEnabled)
    end
end

function sysCall_suspended()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_afterSimulation()
    if pluginFound then
        publishSimState()
    end
end

function sysCall_cleanup()
    if pluginFound then
        simROS.shutdownSubscriber(startSub)
        simROS.shutdownSubscriber(pauseSub)
        simROS.shutdownSubscriber(stopSub)
        simROS.shutdownSubscriber(enableSynModeSub)
        simROS.shutdownSubscriber(triggerNextStepSub)
        simROS.shutdownSubscriber(auxSub)
        simROS.shutdownPublisher(auxPub)
        simROS.shutdownPublisher(simStepDonePub)
        simROS.shutdownPublisher(simStatePub)
        simROS.shutdownPublisher(simTimePub)
    end
end
