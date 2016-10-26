set(appname vizzyMotorRosInterface)

file(GLOB configFiles ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

#message(${YARP_DIR})
yarp_install(FILES ${configFiles} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})
