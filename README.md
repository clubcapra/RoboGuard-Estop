This ROS2 node monitors the `/heartbeat` topic and controls a GPIO pin on Roboguard's raspberry pi to raise the E-stop.

It checks for a new heartbeat every 10ms, and if left without one for 150ms, raises the triggered status and sets the GPIO pin to UP.

Tested and working on Roboguard's PI. Name of topic (`/heartbeat`) and GPIO pin number (`27`) are to be changed to the correct ones.

`lgpio` Python package required.

To easily test this node out, start a new terminal w/ `ros2 topic pub -r 10 /heartbeat std_msgs/msg/Bool "data: true"` to start an artifical roboguard heartbeat. Optionally, run `ros2 topic echo /heartbeat` in a new terminal to monitor the heartbeat manually. Trigger the GPIO pin by killing the terminal running the publisher.
