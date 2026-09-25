// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "webots_ros2_control/Ros2ControlSystem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <webots/device.h>
#include <webots/robot.h>

namespace webots_ros2_control {
  Ros2ControlSystem::Ros2ControlSystem() {
    mNode = NULL;
  }
  void Ros2ControlSystem::init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) {
    mNode = node;
    for (hardware_interface::ComponentInfo component : info.joints) {
      Joint joint;
      joint.name = component.name;

      WbDeviceTag device = wb_robot_get_device(joint.name.c_str());
      WbNodeType type = wb_device_get_node_type(device);
      joint.motor =
        (type == WB_NODE_LINEAR_MOTOR || type == WB_NODE_ROTATIONAL_MOTOR) ? device : wb_position_sensor_get_motor(device);
      device = (component.parameters.count("sensor") == 0) ? wb_robot_get_device(joint.name.c_str()) :
                                                             wb_robot_get_device(component.parameters.at("sensor").c_str());
      type = wb_device_get_node_type(device);
      joint.sensor = (type == WB_NODE_POSITION_SENSOR) ? device : wb_motor_get_position_sensor(device);

      if (joint.sensor)
        wb_position_sensor_enable(joint.sensor, wb_robot_get_basic_time_step());
      if (!joint.sensor && !joint.motor)
        throw std::runtime_error("Cannot find a Motor or PositionSensor with name " + joint.name);

      // Initialize the state. The state starts at 0 (not NAN): controllers such as the
      // tricycle_controller reject NAN feedback and joint_state_broadcaster would publish it.
      joint.controlPosition = false;
      joint.controlVelocity = false;
      joint.controlEffort = false;
      joint.positionCommand = NAN;
      joint.velocityCommand = NAN;
      joint.effortCommand = NAN;
      joint.position = 0.0;
      joint.velocity = 0.0;
      joint.acceleration = 0.0;
      joint.effort = 0.0;
      joint.positionCommandMin = NAN;
      joint.positionCommandMax = NAN;
      joint.velocityMotor = false;
      joint.motorMaxForce = NAN;
      joint.holdGain = 10.0;
      joint.holdMaxVelocity = 0.1;
      joint.holding = false;
      joint.holdPosition = NAN;
      joint.lowerLimit = -INFINITY;
      joint.upperLimit = INFINITY;
      joint.samples = 0;

      // Check if state interfaces have initial positions
      for (hardware_interface::InterfaceInfo stateInterface : component.state_interfaces) {
        if (stateInterface.name == "position" && !stateInterface.initial_value.empty()) {
          joint.position = std::stod(stateInterface.initial_value);
          wb_motor_set_position(joint.motor, std::stod(stateInterface.initial_value));
        }
      }

      // Configure the command interface
      for (hardware_interface::InterfaceInfo commandInterface : component.command_interfaces) {
        if (commandInterface.name == "position") {
          joint.controlPosition = true;
          if (!commandInterface.min.empty())
            joint.positionCommandMin = std::stod(commandInterface.min);
          if (!commandInterface.max.empty())
            joint.positionCommandMax = std::stod(commandInterface.max);
        } else if (commandInterface.name == "velocity")
          joint.controlVelocity = true;
        else if (commandInterface.name == "velocity_motor") {
          // exported as the standard velocity interface, see Joint::velocityMotor
          joint.controlVelocity = true;
          joint.velocityMotor = true;
        } else if (commandInterface.name == "effort")
          joint.controlEffort = true;
        else
          throw std::runtime_error("Invalid hardware info name `" + commandInterface.name + "`");
      }

      // Joint limits of the URDF (used by the velocity_motor hold)
#if HARDWARE_INTERFACE_VERSION_MAJOR > 4 || (HARDWARE_INTERFACE_VERSION_MAJOR == 4 && HARDWARE_INTERFACE_VERSION_MINOR >= 16)
      double urdfEffortLimit = NAN;
      const auto limits = info.limits.find(joint.name);
      if (limits != info.limits.end()) {
        if (limits->second.has_position_limits) {
          joint.lowerLimit = limits->second.min_position;
          joint.upperLimit = limits->second.max_position;
        }
        if (limits->second.has_effort_limits)
          urdfEffortLimit = limits->second.max_effort;
      }
#else
      const double urdfEffortLimit = NAN;
#endif
      if (joint.motor && std::isinf(joint.lowerLimit) && std::isinf(joint.upperLimit) &&
          wb_motor_get_min_position(joint.motor) != wb_motor_get_max_position(joint.motor)) {
        joint.lowerLimit = wb_motor_get_min_position(joint.motor);
        joint.upperLimit = wb_motor_get_max_position(joint.motor);
      }

      if (joint.velocityMotor) {
        if (component.parameters.count("motor_fmax"))
          joint.motorMaxForce = std::stod(component.parameters.at("motor_fmax"));
        else if (!std::isnan(urdfEffortLimit))
          joint.motorMaxForce = urdfEffortLimit;
        if (component.parameters.count("motor_hold_gain"))
          joint.holdGain = std::stod(component.parameters.at("motor_hold_gain"));
        if (component.parameters.count("motor_hold_max_velocity"))
          joint.holdMaxVelocity = std::stod(component.parameters.at("motor_hold_max_velocity"));
        if (joint.motor && !std::isnan(joint.motorMaxForce)) {
          // Webots refuses an available force above the motor's maxForce/maxTorque
          const double maxForce = std::min(joint.motorMaxForce, wb_motor_get_max_force(joint.motor));
          if (wb_device_get_node_type(joint.motor) == WB_NODE_LINEAR_MOTOR)
            wb_motor_set_available_force(joint.motor, maxForce);
          else
            wb_motor_set_available_torque(joint.motor, maxForce);
        }
        RCLCPP_INFO_STREAM(mNode->get_logger(), "Joint motor velocity control for joint " << joint.name << " (fmax = "
                                                                                          << joint.motorMaxForce << ", hold gain = "
                                                                                          << joint.holdGain << ", hold max velocity = "
                                                                                          << joint.holdMaxVelocity << ")");
      }

      if (joint.motor && joint.controlVelocity && !joint.controlPosition) {
        wb_motor_set_position(joint.motor, INFINITY);
        wb_motor_set_velocity(joint.motor, 0.0);
      }
      // effort state from the motor force/torque feedback (joint_state_broadcaster
      // otherwise publishes NAN efforts)
      if (joint.motor)
        wb_motor_enable_force_feedback(joint.motor, wb_robot_get_basic_time_step());

      mJoints.push_back(joint);
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#if HARDWARE_INTERFACE_VERSION_MAJOR > 5 || (HARDWARE_INTERFACE_VERSION_MAJOR == 5 && HARDWARE_INTERFACE_VERSION_MINOR >= 3)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) !=
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#endif

  std::vector<hardware_interface::StateInterface> Ros2ControlSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (Joint &joint : mJoints)
      if (joint.sensor) {
        interfaces.emplace_back(
          hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));
        interfaces.emplace_back(
          hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocity)));
        interfaces.emplace_back(
          hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_ACCELERATION, &(joint.acceleration)));
        if (joint.motor)
          interfaces.emplace_back(
            hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effort)));
      }

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Ros2ControlSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (Joint &joint : mJoints)
      if (joint.motor) {
        if (joint.controlPosition)
          interfaces.emplace_back(
            hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));
        if (joint.controlEffort)
          interfaces.emplace_back(
            hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));
        if (joint.controlVelocity)
          interfaces.emplace_back(
            hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));
      }
    return interfaces;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Ros2ControlSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    static double lastReadTime = 0;

    const double deltaTime = wb_robot_get_time() - lastReadTime;
    lastReadTime = wb_robot_get_time();

    for (Joint &joint : mJoints) {
      if (joint.motor) {
        const double effort = wb_device_get_node_type(joint.motor) == WB_NODE_LINEAR_MOTOR ?
                                wb_motor_get_force_feedback(joint.motor) :
                                wb_motor_get_torque_feedback(joint.motor);
        if (!std::isnan(effort))
          joint.effort = effort;
      }
      if (joint.sensor) {
        const double position = wb_position_sensor_get_value(joint.sensor);
        // not sampled yet, keep the previous (finite) state
        if (std::isnan(position))
          continue;
        // Finite differences need two (velocity) and three (acceleration) samples. Before,
        // report 0 instead of NAN - NAN in /joint_states aborts the system tests and the
        // tricycle_controller logs "Could not read feedback value".
        double velocity = 0.0;
        double acceleration = 0.0;
        if (deltaTime > 0.0 && joint.samples >= 1)
          velocity = (position - joint.position) / deltaTime;
        if (deltaTime > 0.0 && joint.samples >= 2)
          acceleration = (velocity - joint.velocity) / deltaTime;
        if (joint.samples < 2)
          joint.samples++;
        joint.acceleration = acceleration;
        joint.velocity = velocity;
        joint.position = position;
      }
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Ros2ControlSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    for (Joint &joint : mJoints) {
      if (joint.motor) {
        if (joint.controlPosition && !std::isnan(joint.positionCommand)) {
          double positionCommand = joint.positionCommand;
          if (!std::isnan(joint.positionCommandMin))
            positionCommand = std::max(positionCommand, joint.positionCommandMin);
          if (!std::isnan(joint.positionCommandMax))
            positionCommand = std::min(positionCommand, joint.positionCommandMax);
          wb_motor_set_position(joint.motor, positionCommand);
        }
        if (joint.velocityMotor) {
          // Joint motor: Webots drives the joint towards the commanded velocity with at most
          // motorMaxForce. A 0 (or not yet commanded) velocity latches the position reached
          // at that moment and holds it - the bare motor with velocity 0 still creeps under
          // load, so the hold corrects the position with a P term (like the pixel-robotics
          // gazebo_ros2_control fork, whose "no correction at a limit" is not needed here
          // because there is no hard stop at the limits).
          double velocity = std::isnan(joint.velocityCommand) ? 0.0 : joint.velocityCommand;
          // The URDF joint limits are enforced here, not by Webots joint stops: ODE applies
          // the full motor force as an impulse when a joint resting on a stop is driven away
          // from it, which launched the fork on every lift start. The proto places its
          // hard stops outside these limits.
          if ((velocity < 0.0 && joint.position <= joint.lowerLimit) || (velocity > 0.0 && joint.position >= joint.upperLimit))
            velocity = 0.0;
          if (velocity == 0.0) {
            if (!joint.holding) {
              joint.holdPosition = std::clamp(joint.position, joint.lowerLimit, joint.upperLimit);
              joint.holding = true;
            }
            velocity = std::clamp(joint.holdGain * (joint.holdPosition - joint.position), -joint.holdMaxVelocity,
                                  joint.holdMaxVelocity);
          } else
            joint.holding = false;
          const double maxVelocity = wb_motor_get_max_velocity(joint.motor);
          if (maxVelocity > 0.0)
            velocity = std::clamp(velocity, -maxVelocity, maxVelocity);
          wb_motor_set_velocity(joint.motor, velocity);
        } else if (joint.controlVelocity && !std::isnan(joint.velocityCommand)) {
          // In the position control mode the velocity cannot be negative.
          const double velocityCommand = joint.controlPosition ? abs(joint.velocityCommand) : joint.velocityCommand;
          wb_motor_set_velocity(joint.motor, velocityCommand);
        }
        if (joint.controlEffort && !std::isnan(joint.effortCommand))
          wb_motor_set_torque(joint.motor, joint.effortCommand);
      }
    }
    return hardware_interface::return_type::OK;
  }
}  // namespace webots_ros2_control

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2ControlSystem, webots_ros2_control::Ros2ControlSystemInterface)
