FROM osrf/ros:jazzy-desktop
ARG USERNAME=$USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG PLATFORM="LINUX"
ARG SIM_ENABLED=true
ENV SIM_ENABLED=$SIM_ENABLED

ARG DEPLOYABLE_BUILD=false

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y

# Unconditional installs
RUN apt-get install -y \
    python3-pip \
    ros-jazzy-rmw-cyclonedds-cpp 

## Simulation only
RUN if $SIM_ENABLED ; then \
        apt-get update && apt-get install -y \
            # External terminal
            gnome-terminal dbus-x11 \
            # Onshape-to-robot dependencies
            openscad ; fi

ENV SHELL=/bin/bash

CMD ["/bin/bash"]