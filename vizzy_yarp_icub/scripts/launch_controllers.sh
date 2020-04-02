#!/bin/bash

yarpidl_rosmsg --name /typ@/yarpidl &
vizzyUpperBodyJointsSim &
cartesianControllerServer --from server_right_arm_sim.ini &
cartesianSolver --from solver_right_arm_sim.ini &
cartesianRosBridge --from --context vizzyMotorRosInterface --from right_arm_cartesian.ini&
vizzy_iKinGazeCtrl --from config_640x480_Sim.ini &
VizzyArmRoutines --robot vizzySim --arm right &
VizzyArmRoutines --robot vizzySim --arm left &
torsoMotorInterfaceSim &
fixationPointStatusBridge &
sleep 1
fixationPointActionBridge &
sleep 1
gazeSimulationBridge
