# Roboguard E-Stop System (ROS 2)

This package implements a distributed emergency stop (E-Stop) system for the Roboguard platform using ROS 2. It includes several nodes for collecting, monitoring, aggregating, and acting on E-Stop signals from multiple sources.

All nodes can be launched together using `estop_launch.py`.

---

## Nodes

### 1. `EstopHeartbeatUnifier`
**Purpose**: Aggregates heartbeats from multiple E-Stop sources and republishes a unified status.

- **Subscribed**:
  - `/estop_controller_h` (`std_msgs/Bool`) – Xbox controller estop status heartbeat
  - `/estop_ui_h` (`std_msgs/Bool`) – Foxglove estop status heartbeat 
  - `/estop_hardware_h` (`std_msgs/Bool`) – combined STM32 + Mushroom estop status heartbeat

- **Published**:
  - `/estop_general_h` (`std_msgs/Bool`) – `true` if **any** input is missing or `true`, else `false`

- **Behavior**:
  - Publishes at 100 Hz (every 10ms)
  - Triggers E-Stop (`true`) if:
    - Any heartbeat is `true`
    - Any heartbeat is missing for >150ms

---

### 2. `EstopHardwareMonitor`
**Purpose**: Monitors STM32 and mushroom heartbeat topics and republishes a hardware-level E-Stop status.

- **Subscribed**:
  - `/estop_stm32_status` (`std_msgs/Bool`) – STM32 current estop status heartbeat
  - `/estop_mushroom_status` (`std_msgs/Bool`) – Mushroom button current estop status heartbeat

- **Published**:
  - `/estop_hardware_h` (`std_msgs/Bool`) – combined hardware estop requirement

- **Behavior**:
  - Publishes at 100 Hz
  - `true` (E-Stop) if either source hasn’t published a boolean within 400ms
  - `false` otherwise no matter the value (we assume they will engage their own Estop and only care if the node is working)

---

### 3. `EstopManager`
**Purpose**: Controls a Roboguard Pi GPIO pin to enforce the E-Stop circuit.

- **Subscribed**:
  - `/estop_general_h` (`std_msgs/Bool`) – EStop general requirement heartbeat
  - `/estop_manual_s` (`std_msgs/Bool`) – Manual override

- **Published**: None

- **Behavior**:
  - GPIO 29 goes **HIGH** when:
    - General estop is `true`, or
    - Heartbeat is missing >150ms, or
    - Manual override has sent a `true` message
  - GPIO 29 goes **LOW** otherwise
    - General estop resumes AND
    - Manual override's last message was `false`

---

### 4. `PowerSupplyToggler`
**Purpose**: Controls a separate GPIO pin for toggling power supply modes.

- **Subscribed**:
  - `/powersupply_mode_s` (`std_msgs/Bool`)

- **Published**: None

- **Behavior**:
  - GPIO 31 is set HIGH if last message was `true`, LOW if `false`. Pin is not touched until first message is received.

---

## Topic Summary

| Topic Name               | Type            | Description                          |
|--------------------------|------------------|--------------------------------------|
| `/estop_stm32_status`   | `std_msgs/Bool`  | EStop status from STM32         |
| `/estop_mushroom_status`| `std_msgs/Bool`  | EStop status from Mushroom|
| `/estop_hardware_h`     | `std_msgs/Bool`  | Existing heartbeat from STM32 and Mushroom     |
| `/estop_controller_h`   | `std_msgs/Bool`  | Heartbeat from Xbox controller   |
| `/estop_ui_h`           | `std_msgs/Bool`  | Heartbeat from Foxglove        |
| `/estop_general_h`      | `std_msgs/Bool`  | Final unified E-Stop heartbeat        |
| `/estop_manual_s`       | `std_msgs/Bool`  | Manual override request              |
| `/powersupply_mode_s`   | `std_msgs/Bool`  | Toggles GPIO for power management    |

---

## Launching

All nodes can be launched together using:

```bash
ros2 launch your_package_name estop_launch.py