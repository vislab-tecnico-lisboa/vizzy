
set(appname vizzyYogaDemo)

set(configFiles config.ini head.txt left_shoulder_arm.txt right_shoulder_arm.txt torso.txt)

#message(${YARP_DIR})
yarp_install(FILES ${configFiles} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/${appname})

