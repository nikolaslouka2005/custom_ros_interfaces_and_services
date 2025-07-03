# Custom ROS 2 Interfaces and Services

This project demonstrates a ROS 2 Python package with custom message and service types.

## Architecture Diagram

![image](https://github.com/user-attachments/assets/831f51d7-60d1-48bd-90c2-a9bf335671f6)


This diagram shows the communication model in the project:

- The **Battery node** acts as a client sending requests to the `/set_led` service.
- The **LED panel node** acts as a server processing requests to turn LEDs on/off.
- Communication happens via the `/set_led` service, while the server can publish status updates on the `/led_panel_state` topic.

---

## Package: `my_py_pkg`

### Nodes:

- **`battery.py`**: Defines the `BatteryNode` class which publishes battery status and acts as a client calling the `/set_led` service.
- **`led_panel.py`**: Defines the `LedPanel` class which acts as a service server for `/set_led` and publishes the LED states on the `/led_panel_state` topic.

### Custom Interfaces
---
-LedStateArray.msg

---

-SetLed.srv

---

## Build and Run

```bash
# Clone repo into your workspace src folder
cd ~/ros2_custom_ws/src
git clone git@github.com:nikolaslouka2005/custom_ros_interfaces_and_services.git
```
```bash
# Build the workspace
cd ~/ros2_custom_ws
colcon build
source install/setup.bash
```
```bash
#Run the battery node (publisher + client)
ros2 run my_py_pkg battery
```
```bash
# Run the LED panel node (service server + publisher)
ros2 run my_py_pkg led_panel
```
```bash
#Call the service to set LED state
ros2 service call /set_led my_py_pkg/srv/SetLed "{led_number: 1, state: true}"
```



