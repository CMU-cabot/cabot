# mf_localization_msgs package (ROS1)

## msg

- **MFGlobalPosition**: global (lat, lng, floor) position
- **MFLocalPosition**: ROS local position
- **StatusResponse**: service response

## srv

- **ConvertLocalToGlobal**: to convert local position to global position
- **FloorChange**: to change floor for gazebo simulation
- **MFSetInt**: for set_current_floor service
- **MFTrigger**: for services to toggle status 
- **RestartLocalization**: to restart localization
- **StartLocalization**: to start localization
- **StopLocalization**: to stop localization