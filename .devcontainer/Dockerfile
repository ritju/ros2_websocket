FROM ritju/ros2-turtlebot3:latest

# Install .NET SDK
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    # Install prerequisites
    && apt-get install -y --no-install-recommends \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-rosapi \
    python3-websockets \
    python3-pytest \
    # Cleanup
    # && apt-get autoremove -y \
    # && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
RUN pip install rosdepc \
    && rosdepc init
ENV DEBIAN_FRONTEND=dialog

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN git config --global --add safe.directory $WORKSPACE

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT []
CMD tail -f /dev/null
