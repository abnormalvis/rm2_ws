#include "rm2_orientation_controller/rm2_orientation_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm2_orientation_controller
{

controller_interface::CallbackReturn OrientationController::on_init()
{
	try
	{
		auto_declare<std::string>("frame_source", "");
		auto_declare<std::string>("frame_target", "");
		auto_declare<std::string>("imu_name", "");
		auto_declare<std::string>("imu_frame_id", "");
		auto_declare<std::string>("imu_topic", "data");
	}
	catch (const std::exception & e)
	{
		RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
		return controller_interface::CallbackReturn::ERROR;
	}

	imu_rt_buffer_.writeFromNonRT(OrientationSample{});
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OrientationController::command_interface_configuration() const
{
	controller_interface::InterfaceConfiguration config;
	config.type = controller_interface::interface_configuration_type::NONE;
	return config;
}

controller_interface::InterfaceConfiguration OrientationController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration config;

	const auto imu_name = get_node()->get_parameter("imu_name").as_string();
	if (imu_name.empty())
	{
		config.type = controller_interface::interface_configuration_type::NONE;
		return config;
	}

	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	config.names.push_back(imu_name + "/orientation.x");
	config.names.push_back(imu_name + "/orientation.y");
	config.names.push_back(imu_name + "/orientation.z");
	config.names.push_back(imu_name + "/orientation.w");
	return config;
}

controller_interface::CallbackReturn OrientationController::on_configure(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	frame_source_ = get_node()->get_parameter("frame_source").as_string();
	frame_target_ = get_node()->get_parameter("frame_target").as_string();
	imu_name_ = get_node()->get_parameter("imu_name").as_string();
	imu_frame_id_ = get_node()->get_parameter("imu_frame_id").as_string();
	imu_topic_ = get_node()->get_parameter("imu_topic").as_string();

	if (frame_source_.empty() || frame_target_.empty())
	{
		RCLCPP_ERROR(
			get_node()->get_logger(),
			"Missing required params: frame_source/frame_target (got '%s' -> '%s')",
			frame_source_.c_str(), frame_target_.c_str());
		return controller_interface::CallbackReturn::ERROR;
	}

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

	if (!imu_topic_.empty())
	{
		imu_data_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
			imu_topic_, rclcpp::QoS(10),
			std::bind(&OrientationController::imuDataCallback, this, std::placeholders::_1));
	}

	source2target_msg_.header.frame_id = frame_source_;
	source2target_msg_.child_frame_id = frame_target_;
	source2target_msg_.transform.rotation.w = 1.0;

	last_imu_update_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OrientationController::on_activate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	receive_imu_msg_ = false;
	imu_orientation_x_ = nullptr;
	imu_orientation_y_ = nullptr;
	imu_orientation_z_ = nullptr;
	imu_orientation_w_ = nullptr;
	assignImuInterfaces(state_interfaces_);

	imu_rt_buffer_.writeFromNonRT(OrientationSample{});
	last_imu_update_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OrientationController::on_deactivate(
	const rclcpp_lifecycle::State & /*previous_state*/)
{
	imu_orientation_x_ = nullptr;
	imu_orientation_y_ = nullptr;
	imu_orientation_z_ = nullptr;
	imu_orientation_w_ = nullptr;
	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OrientationController::update(
	const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
	if (!tf_buffer_)
	{
		return controller_interface::return_type::OK;
	}

	double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
	bool has_orientation = false;

	if (imu_orientation_x_ && imu_orientation_y_ && imu_orientation_z_ && imu_orientation_w_)
	{
		x = imu_orientation_x_->get_optional().value_or(0.0);
		y = imu_orientation_y_->get_optional().value_or(0.0);
		z = imu_orientation_z_->get_optional().value_or(0.0);
		w = imu_orientation_w_->get_optional().value_or(1.0);
		has_orientation = true;
	}
	else
	{
		const auto sample = *imu_rt_buffer_.readFromRT();
		if (sample.valid && sample.stamp > last_imu_update_time_)
		{
			last_imu_update_time_ = sample.stamp;
			x = sample.x;
			y = sample.y;
			z = sample.z;
			w = sample.w;
			has_orientation = true;
		}
	}

	if (!has_orientation)
	{
		return controller_interface::return_type::OK;
	}

	if (imu_frame_id_.empty())
	{
		RCLCPP_WARN_THROTTLE(
			get_node()->get_logger(), *get_node()->get_clock(), 2000,
			"imu_frame_id is empty; cannot build transform chain");
		return controller_interface::return_type::OK;
	}

	geometry_msgs::msg::TransformStamped source2target;
	source2target.header.stamp = time;
	source2target.header.stamp.nanosec += 1;  // avoid redundant timestamp

	if (getTransform(source2target, x, y, z, w))
	{
		source2target_msg_ = source2target;
	}
	else
	{
		source2target = source2target_msg_;
		source2target.header.stamp = time;
		source2target.header.stamp.nanosec += 1;
	}

	if (tf_broadcaster_)
	{
		tf_broadcaster_->sendTransform(source2target);
	}
	return controller_interface::return_type::OK;
}

void OrientationController::assignImuInterfaces(std::vector<hardware_interface::LoanedStateInterface> & interfaces)
{
	if (imu_name_.empty())
	{
		return;
	}

	for (auto & interface : interfaces)
	{
		if (interface.get_prefix_name() != imu_name_)
		{
			continue;
		}

		if (interface.get_interface_name() == "orientation.x")
		{
			imu_orientation_x_ = &interface;
		}
		else if (interface.get_interface_name() == "orientation.y")
		{
			imu_orientation_y_ = &interface;
		}
		else if (interface.get_interface_name() == "orientation.z")
		{
			imu_orientation_z_ = &interface;
		}
		else if (interface.get_interface_name() == "orientation.w")
		{
			imu_orientation_w_ = &interface;
		}
	}
}

bool OrientationController::getTransform(
	geometry_msgs::msg::TransformStamped & source2target,
	const double x,
	const double y,
	const double z,
	const double w)
{
	source2target.header.frame_id = frame_source_;
	source2target.child_frame_id = frame_target_;
	source2target.transform.rotation.w = 1.0;

	tf2::Transform source2odom, odom2fixed, fixed2target;

	try
	{
		auto tf_msg = tf_buffer_->lookupTransform(frame_source_, "odom", tf2::TimePointZero);
		tf2::fromMsg(tf_msg.transform, source2odom);

		tf_msg = tf_buffer_->lookupTransform("odom", imu_frame_id_, tf2::TimePointZero);
		tf2::fromMsg(tf_msg.transform, odom2fixed);

		tf_msg = tf_buffer_->lookupTransform(imu_frame_id_, frame_target_, tf2::TimePointZero);
		tf2::fromMsg(tf_msg.transform, fixed2target);
	}
	catch (const tf2::TransformException & ex)
	{
		RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000, "%s", ex.what());
		return false;
	}

	tf2::Quaternion odom2fixed_quat(x, y, z, w);
	odom2fixed.setRotation(odom2fixed_quat);
	source2target.transform = tf2::toMsg(source2odom * odom2fixed * fixed2target);
	return true;
}

void OrientationController::imuDataCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
	receive_imu_msg_ = true;
	if (imu_frame_id_.empty() && !msg->header.frame_id.empty())
	{
		imu_frame_id_ = msg->header.frame_id;
	}

	OrientationSample sample;
	sample.stamp = msg->header.stamp;
	sample.x = msg->orientation.x;
	sample.y = msg->orientation.y;
	sample.z = msg->orientation.z;
	sample.w = msg->orientation.w;
	sample.valid = true;
	imu_rt_buffer_.writeFromNonRT(sample);
}

}  // namespace rm2_orientation_controller

PLUGINLIB_EXPORT_CLASS(
	rm2_orientation_controller::OrientationController,
	controller_interface::ControllerInterface)