# Custom ROS 2 Interfaces and Services

This project demonstrates a ROS 2 Python package with custom message and service types.

## Architecture Diagram

![image](https://github.com/user-attachments/assets/0251ca50-92a4-4cf3-bba5-d9a100b09cd5)


This diagram shows the communication model in the project:

- The **Battery node** acts as a client sending requests to the `/set_led` service.
- The **LED panel node** acts as a server processing requests to turn LEDs on/off.
- Communication happens via the `/set_led` service, while the server can publish status updates on the `/led_panel_state` topic.

---

## Package: `my_py_pkg`

### Nodes:
- **`battery.py`**: Publishes battery status to `/battery_status` using the custom `HardwareStatus.msg`
- **`led_panel.py`**: Provides a service `/set_led` using the custom `SetLed.srv`

### Custom Interfaces

- `LedStateArray.msg`
- `SetLed.srv`

---

## Build and Run

```bash
# Clone repo into your workspace src folder
cd ~/ros2_custom_ws/src
git clone git@github.com:nikolaslouka2005/custom_ros_interfaces_and_services.git

# Build the workspace
cd ~/ros2_custom_ws
colcon build
source install/setup.bash
