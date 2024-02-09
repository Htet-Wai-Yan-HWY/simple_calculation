FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

COPY config/ /site_config/

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID


ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# ENV QT_DEBUG_PLUGINS=1
# ENV QT_QPA_PLATFORM=wayland
# ENV QT_QPA_PLATFORM_PLUGIN_PATH=/opt/Qt/${QT_VERSION}/gcc_64/plugins
# ENV QT_PLUGIN_PATH=/opt/Qt/${QT_VERSION}/gcc_64/plugins
ENV DISPLAY=:1

# RUN apt-get update && \
#     apt-get install -y libqt5gui5 && \
#     rm -rf /var/lib/apt/lists/*
# ENV QT_DEBUG_PLUGINS=1

# RUN apt-get install -y libxcb-util1
# RUN apt-get install -y libxcb-xinerama0
# RUN apt-get install -y libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-xinerama0 libxcb-xfixes0
#creating a non-root user 
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# USER ros

RUN apt-get update \
&& apt-get install -y sudo \
&& apt-get install -y x11vnc xvfb firefox \
# && echo ${USERNAME} ALL=\(root\) NOPASSSWORD:ALL > etc/sudoers.d/${USERNAME}\
# && chmod 0440 /etc/sudoers.d/${USERNAME} \
&& rm -rf /var/lib/apt/lists/* 

