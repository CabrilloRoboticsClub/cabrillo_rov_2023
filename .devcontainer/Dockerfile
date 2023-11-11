FROM mcr.microsoft.com/devcontainers/base:jammy

# Setup packages needed for `make devbox-install`
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install git ansible python3-pip vim

# Install the latest devbox playbook
RUN cd /tmp \
    && git clone https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git \
    && make -C cabrillo_rov_2023 devbox-install 
        
# Put ros2 in the path
RUN echo '. /opt/ros/humble/setup.bash' >> /home/vscode/.bashrc 
