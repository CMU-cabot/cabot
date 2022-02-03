# cabot_debug package (ROS1)

debug utilities for logging output of command to check CPU/GPU status like `top`, `nvidia-smi dmon`

## command_logger.py

publish the output of the specified command to the specified string msg

- topic: publishing topic name
- command: command to be executed
