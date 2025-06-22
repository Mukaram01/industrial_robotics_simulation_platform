FROM ros:humble-ros-base

# Create workspace
WORKDIR /opt/industrial_ws

# Copy repository contents
COPY . /opt/industrial_ws

# Install pip and Python dependencies
RUN apt-get update && \
    apt-get install -y python3-pip && \
    python3 -m pip install --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Install ROS dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src \
        --rosdistro humble \
        --skip-keys "ament_python flask-socketio pymodbus pyyaml moveit_commander onnxruntime flask paho-mqtt numpy asyncua"

# Build the workspace
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Default command
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch simulation_tools integrated_system_launch.py"]
