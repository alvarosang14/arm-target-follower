#yarp_install(DIRECTORY contexts/sensors DESTINATION ${ROBOTICSLAB-VISION_CONTEXTS_INSTALL_DIR})

# applications/ymanager.ini
yarp_install(FILES applications/pruebas.xml
             DESTINATION ${ROBOTICSLAB-TFG_APPLICATIONS_INSTALL_DIR})

yarp_install(DIRECTORY contexts/
                 DESTINATION ${ROBOTICSLAB-TFG_CONTEXTS_INSTALL_DIR})
