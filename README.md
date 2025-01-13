<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a id="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <h3 align="center">Ardupilot Simulation Example</h3>

  <p align="center">
    <em>Depth Estimation and Navigation Using Stereo Cameras</em>
  </p>
  
  <p align="center">
    This project provides a simulated environment for depth estimation and navigation using stereo cameras.  
    It utilizes <strong>ROS2</strong>, <strong>Ardupilot</strong>, and <strong>Gazebo</strong> for simulation.  
  </p>
  
  <hr />
  
  <p align="center">
    <strong>Presented at:</strong> <br />
    <span style="font-size: 1.1rem;">The Korean Society of Mechanical Engineers (KSME)</span> <br />
    <strong>Event:</strong> Robotics Engineering Workshop 2025 <br />
    <strong>Date:</strong> January 15, 2025 <br />
  </p>
</div>

<div style="border-top: 2px dashed #ccc; margin: 20px 0;"></div>


## **Environment**
- **Operating System**: Ubuntu 22.04  
- **Development Environment**:
  - **ROS Version**: ROS2 Humble  
  - **Simulation Tool**: Gazebo Sim (Harmonic Version)  
  - **Flight Controller**: ArduPilot  
  - **Programming Language**: Python 3.10.12  




## Getting Started

The pre-setting required to execute this code is as follows.

### 1. Git Clone
```bash
cd ~/ros2_ws
git clone https://github.com/arrl-ajou/ardupilot_simulation_example.git
```

### 2. File Setting
```bash
cd ~/ros2_ws/ardupilot_simulation_example
chmod +x setting.sh
./setting.sh ~/ros2_ws/src    # Your ROS2 workspace src (with Ardupilot)
```
### 3. Build
```bash
colcon build 
```

### 4. Run
> Gazebo Simulation
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_forest.launch.py rviz:=true use_gz_tf:=true
```
> MAVProxy
```bash
mavproxy.py --master udp:127.0.0.1:14550  --console
```
```bash
mode guided
arm throttle
takeoff 1.0
```


## MAVProxy Example
### 1. For Python
[MAVLink](https://ardupilot.org/dev/docs/mavlink-commands.html) 
```py
from pymavlink import mavutil
import time
import math

# MAVLink 연결 설정
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # 연결 포트 설정
master.wait_heartbeat()
print(f"Heartbeat received from system (system {master.target_system}, component {master.target_component})")

# GUIDED 모드로 전환
def set_guided_mode():
    master.set_mode("GUIDED")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == master.mode_mapping()["GUIDED"]:
            print("GUIDED mode set successfully.")
            break
        time.sleep(1)

# 모터 ARM (활성화)
def arm_drone():
    master.arducopter_arm()
    print("Arming motors...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Motors are armed.")
            break
        time.sleep(1)

# 드론 이륙 함수
def takeoff(altitude):
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Takeoff 명령
        0, 0, 0, 0, 0, 0, 0, altitude  # 목표 고도
```

### 2. For C++
[MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/quickstart.html)
#### Install
```bash
sudo apt remove mavsdk
wget https://github.com/mavlink/MAVSDK/releases/download/v2.14.1/libmavsdk-dev_2.14.1_ubuntu22.04_amd64.deb
sudo dpkg -i libmavsdk-dev_2.14.1_ubuntu22.04_amd64.deb
```

Code
```cpp
#include "mav_handler.h"
#include <iostream>
#include <thread>
#include <cmath>
#include <stdexcept>

using namespace mavsdk;

MavHandler::MavHandler(const std::string& connection_url, float takeoff_altitude)
    : mavsdk(Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}) {
    // 연결 설정
    auto connection_result = mavsdk.add_any_connection(connection_url);
    if (connection_result != mavsdk::ConnectionResult::Success) {
        throw std::runtime_error("Connection failed: " +
                                 std::to_string(static_cast<int>(connection_result)));
    }

    auto maybe_system = mavsdk.first_autopilot(3.0); // 3초 대기
    if (!maybe_system) {
        throw std::runtime_error("Timed out waiting for system.");
    }
    system = maybe_system.value();

    // Plugins 초기화
    telemetry = std::make_shared<mavsdk::Telemetry>(system);
    action = std::make_shared<mavsdk::Action>(system);
    offboard = std::make_shared<mavsdk::Offboard>(system);

    std::cout << "Waiting for the system to become armable..." << std::endl;
    while (!telemetry->health().is_armable) {
        std::cout << "System not armable yet. Waiting..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "System is now armable!" << std::endl;

    // 이륙 여부 확인
    auto current_altitude = telemetry->position().relative_altitude_m;

    if (current_altitude < takeoff_altitude * 0.6) {
        // 이륙 필요
        action->set_takeoff_altitude(takeoff_altitude);

        bool takeoff_success = false;
        while (!takeoff_success) {
            if (!telemetry->armed()) {
                // 드론 Arm
                if (action->arm() != mavsdk::Action::Result::Success) {
                    std::cerr << "Failed to arm the drone. Retrying..." << std::endl;
                }
                std::cout << "Drone armed successfully." << std::endl;
            } else {
                std::cout << "Drone is already armed." << std::endl;
            }

            if (action->takeoff() == mavsdk::Action::Result::Success) {
                takeoff_success = true;
                std::cout << "Takeoff initiated to " << takeoff_altitude << " meters." << std::endl;

                // 이륙 대기
                const float takeoff_threshold = takeoff_altitude * 0.6; // 목표 고도의 60% 이상 도달 기준
                while (telemetry->position().relative_altitude_m < takeoff_threshold) {
                    std::cout << "Current altitude: " << telemetry->position().relative_altitude_m
                            << " meters. Waiting for takeoff..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                std::cout << "Takeoff complete." << std::endl;
            } else {
                std::cerr << "Failed to take off. Retrying..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(5)); // 재시도 전 대기
            }
        }
    } else {
        std::cout << "Drone is already in the air at altitude: " << current_altitude << " meters." << std::endl;
    }


    // Offboard 초기화
    mavsdk::Offboard::VelocityBodyYawspeed stay{};
    if (offboard->set_velocity_body(stay) != mavsdk::Offboard::Result::Success) {
        throw std::runtime_error("Failed to set initial velocity.");
    }

    if (offboard->start() != mavsdk::Offboard::Result::Success) {
        throw std::runtime_error("Failed to start Offboard mode.");
    }
}

void MavHandler::mav_current_position() {
    auto position = telemetry->position_velocity_ned();
    current_x = position.position.east_m;
    current_y = position.position.north_m;
    current_z = -position.position.down_m;

    auto velocity = telemetry->velocity_ned();
    current_vel_x = velocity.east_m_s;
    current_vel_y = velocity.north_m_s;
    current_vel_z = velocity.down_m_s;
    current_speed = std::sqrt(current_vel_x * current_vel_x +
                              current_vel_y * current_vel_y +
                              current_vel_z * current_vel_z);

    mav_degree = telemetry->attitude_euler().yaw_deg ;

    current_degree = 90 - mav_degree;
}

void MavHandler::mav_command_send(float current_vel, float nav_degree) {
    mavsdk::Offboard::VelocityBodyYawspeed velocity_cmd{};
    mav_current_position();
    // float nav_radian = nav_degree * M_PI / 180.0;
    velocity_cmd.forward_m_s = current_vel;
    velocity_cmd.right_m_s = 0.0;
    velocity_cmd.down_m_s =  0.0; 
    velocity_cmd.yawspeed_deg_s = nav_degree; 


    if (offboard->set_velocity_body(velocity_cmd) != mavsdk::Offboard::Result::Success) {
        throw std::runtime_error("Failed to send velocity command.");
    }
}
```

Header
```hpp
#ifndef MAV_HANDLER_H
#define MAV_HANDLER_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <memory>
#include <string>

class MavHandler {
public:
    MavHandler(const std::string& connection_url, float takeoff_altitude);

    void mav_current_position();
    void mav_command_send(float current_vel, float nav_degree);

    // Getter functions for current position and speed
    float get_current_x() const { return current_x; }
    float get_current_y() const { return current_y; }
    float get_current_z() const { return current_z; }
    float get_current_speed() const { return current_speed; }
    double get_current_degree() const { return current_degree; }

private:
    mavsdk::Mavsdk mavsdk; // Mavsdk 객체
    std::shared_ptr<mavsdk::System> system;
    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::Offboard> offboard;

    float current_x = 0.0f, current_y = 0.0f, current_z = 0.0f;
    float current_vel_x = 0.0f, current_vel_y = 0.0f, current_vel_z = 0.0f, current_speed = 0.0f;
    float mav_degree = 0.0f;
    double current_degree = 0.0;
};

#endif // MAV_HANDLER_H
```