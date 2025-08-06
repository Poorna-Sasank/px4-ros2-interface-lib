#include "mode.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <Eigen/Core>

static const std::string kModeName = "NoseDiveControl";
static const bool kEnableDebugOutput = true;

using namespace std::chrono_literals;

FlightModeTest::FlightModeTest(rclcpp::Node& node)
    : ModeBase(node, kModeName)
    , _node(node)
    , _attitude_setpoint(std::make_shared<px4_ros2::AttitudeSetpointType>(*this))
    , _peripheral_actuator_controls(std::make_shared<px4_ros2::PeripheralActuatorControls>(*this))
    , _vehicle_local_position(std::make_shared<px4_ros2::OdometryLocalPosition>(*this))
{
    loadParameters();
}

void FlightModeTest::loadParameters()
{
    _node.declare_parameter<float>("nose_dive_pitch", 0.0);
    _node.declare_parameter<float>("nose_dive_thrust", 0.744);
    _node.declare_parameter<float>("yaw", 1.57);

    _node.get_parameter("nose_dive_pitch", _param_nose_dive_pitch);
    _node.get_parameter("nose_dive_thrust", _param_nose_dive_thrust);
    _node.get_parameter("yaw", _param_yaw);
}

void FlightModeTest::onActivate()
{
}

void FlightModeTest::onDeactivate()
{
}

void FlightModeTest::updateSetpoint(float dt_s)
{
    Eigen::Vector2f pitch_roll(0.0f, _param_nose_dive_pitch);
    float thrust = _param_nose_dive_thrust;

    const Eigen::Quaternionf qd = px4_ros2::eulerRpyToQuaternion(pitch_roll.x(), pitch_roll.y(), _param_yaw);
    const Eigen::Vector3f thrust_sp{0.0f, 0.0f, -thrust};

    _attitude_setpoint->update(qd, thrust_sp, 0.0f);
    _peripheral_actuator_controls->set(0.0f);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<FlightModeTest>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}