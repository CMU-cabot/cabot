# cabot_explore package (ROS1)

Provide exploration function.

## How to run in exploration mode
Launch cabot with "-e" option.
For example, run following command in simulation mode.
```
./launch.sh -s -e
```

## How to replay exploration function from bag file

Lauch replay exploration function from ROS bag file by following command, and 
start replay by pressing space button in launched xterm window.
```
./launch_explore_replay.sh -b <ROS bag file path>
```
