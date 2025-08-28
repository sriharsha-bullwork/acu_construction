install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
scripts/waypoint_commander.py
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config