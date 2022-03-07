# cabot_ui package (ROS1)

user interface related code and i18n files

## i18n

internationalization files

## menu

- menu.yaml - not used

## src

- cabot_ble.py
  - BLE central server
  - Speak service server
- cabot_force.py - not used
- cabot_gamepad.py
  - map game button to cabot event
- cabot_keyboard.py
  - map key input to cabot event
- cabot_ui_manager.py
  - main UI code for handling event and navigation
- navcog_global_planner.cpp/h - not used
- navcog_map.py
  - show all nodes and links on MapService server on rviz
- navcodnode.py - not used
- tts_node.py - not used
  - Speak service server using IBM watson TTS service to speak
