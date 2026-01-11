//
// Created by ch on 2025/10/5.
//

#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Core>

/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
void quatToRPY(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);

double yawFromQuat(const geometry_msgs::msg::Quaternion& q);

tf2::Quaternion getAverageQuaternion(const std::vector<tf2::Quaternion>& quaternions,
                                     const std::vector<double>& weights);

tf2::Quaternion rotationMatrixToQuaternion(const Eigen::Map<Eigen::Matrix3d>& rot);
