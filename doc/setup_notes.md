ROV setup:

- `git clone https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git`
- `cd cabrillo_rov_2023`
- `git submodule init`
- `git submodule update`
- `rosdep init` (as root)
- `rosdep update` (not as root)
- `rosdep install -i --from-path src/ --rosdistro humble -y`
- `source /opt/ros/humble/setup.bash`
- `make`
- `source install/setup.bash`
- `ros2 launch launch/rov.launch.py`

loop:
- `make`
- `source install/setup.bash`
- `ros2 launch`

user ubuntu needs to be added to dialout group