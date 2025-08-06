#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

class FlightModeTest : public px4_ros2::ModeBase
{
public:
    explicit FlightModeTest(rclcpp::Node& node);

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

private:
    void loadParameters();

    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_setpoint;
    std::shared_ptr<px4_ros2::PeripheralActuatorControls> _peripheral_actuator_controls;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

    float _param_nose_dive_pitch = {};
    float _param_nose_dive_thrust = {};
    float _param_yaw = {};
};