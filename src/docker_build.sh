sudo docker build -t josedallatorre/sjtu_drone:latest .
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch pitchside_tracker yolo_ball_tracker.launch.py