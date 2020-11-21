cmake_minimum_required(VERSION 2.8.3)
project(cabot_site_cmu)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY i18n/ DESTINATION share/${PROJECT_NAME}/i18n/)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps/)
install(DIRECTORY menu/ DESTINATION share/${PROJECT_NAME}/menu/)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds/)

