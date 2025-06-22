FROM ros:humble-ros-base

# Create workspace
WORKDIR /opt/industrial_ws

# Copy repository contents
COPY . /opt/industrial_ws

# Install system dependencies and Python requirements
RUN python3 -m pip install --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src -y --ignore-src \
        --rosdistro humble \
        --skip-keys "ament_python flask-socketio pymodbus pyyaml moveit_commander onnxruntime flask paho-mqtt numpy" && \
    . /opt/ros/humble/setup.sh && \
    colcon build

# Default command runs the integrated launch file
CMD bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch simulation_tools integrated_system_launch.py"
