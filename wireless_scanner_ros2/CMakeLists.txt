cmake_minimum_required(VERSION 3.5)
project(wireless_scanner_ros)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY script/scanner/cabot DESTINATION lib/${PROJECT_NAME}/)

install(PROGRAMS
  script/receiver/ble_scan_converter.py
  script/receiver/esp32_wifi_scan_converter.py
  script/scanner/cabot_serial.py
  script/scanner/dbus_ibeacon_scanner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
