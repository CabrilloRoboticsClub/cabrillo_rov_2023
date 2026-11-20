FROM docker.io/ros:lyrical

ARG USERNAME=robot
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Setup packages needed for `make devbox-install`
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install git ansible python3-pip vim sudo 

# Delete user if it exists in container
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# For vulcanexus container... make a user. 
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -m -s /usr/bin/bash --uid $USER_UID --gid $USER_GID -G adm,sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME \
    && echo '. /opt/ros/lyrical/setup.bash' >> /home/${USERNAME}/.bashrc 

USER $USERNAME
