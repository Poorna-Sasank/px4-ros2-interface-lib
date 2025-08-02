#pragma once
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static const std::string kName = "NoseDiveControl";

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kName)
  , _attitude_setpoint(std::make_shared<px4_ros2::AttitudeSetpointType>(*this))
  , _peripheral_actuator_controls(std::make_shared<px4_ros2::PeripheralActuatorControls>(*this))
  , _vehicle_local_position(std::make_shared<px4_ros2::OdometryLocalPosition>(*this))
  , _nose_dive_pitch(-0.5410f) // -31 degrees in radians (nose down)
  , _nose_dive_thrust(0.70f)   // Thrust for rapid descent with attitude control
  , _yaw(1.574f)               // Fixed yaw (90 degrees)
  {
  }

  void onActivate() override 
  {
  }

  void onDeactivate() override 
  {
  }

  void updateSetpoint(float dt_s) override
  {
    // Nose dive: fixed pitch, zero roll, low thrust
    Eigen::Vector2f pitch_roll(0.0f, _nose_dive_pitch); // Roll = 0, Pitch = -31 degrees
    float thrust = _nose_dive_thrust;

    // Convert pitch, roll, yaw to quaternion
    const Eigen::Quaternionf qd = px4_ros2::eulerRpyToQuaternion(pitch_roll.x(), pitch_roll.y(), _yaw);
    const Eigen::Vector3f thrust_sp{0.0f, 0.0f, -thrust};

    _attitude_setpoint->update(qd, thrust_sp, 0.0f);
    _peripheral_actuator_controls->set(0.0f);
  }

private:
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
  std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  float _nose_dive_pitch; // Pitch angle for nose dive (radians)
  float _nose_dive_thrust; // Thrust for nose dive
  float _yaw; // Fixed yaw angle
};
