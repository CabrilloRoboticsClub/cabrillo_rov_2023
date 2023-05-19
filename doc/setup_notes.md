# SETUP NOTES

ROV build code:

- `git clone https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git`
- `cd cabrillo_rov_2023`
- `git submodule init`
- `git submodule update`
- `sudo rosdep init`
- `rosdep update`
- `rosdep install -i --from-path src/ --rosdistro humble -y`
- `source /opt/ros/humble/setup.bash`
- `make`

loop:

- `rosdep update`
- `rosdep install -i --from-path src/ --rosdistro humble -y`
- `source /opt/ros/humble/setup.bash`
- `make`
- `source install/setup.bash`
- `ros2 launch`
