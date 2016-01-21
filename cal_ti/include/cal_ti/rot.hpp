#ifndef ROT_H_
#define ROT_H_

#include "tiv_types.hpp"
#include <eigen3/Eigen/Core>


namespace tiv {
	// matrix equivalent to axis-angle: eaa = angle * axis (be careful when angle is close to 0).
	extern "C" void mat2eaa(const Eigen::Matrix3d& R, Eigen::Vector3d& eaa);
	extern "C" void eaa2mat(const Eigen::Vector3d& eaa, Eigen::Matrix3d& R);

	// in this conversion, rpy = {roll (->), pitch (->), yaw} are representing rotation w.r.t. z-axis, y-axis, and x-axis respectively.
	extern "C" void ti_mat2rpy(const Eigen::Matrix3d& R, Eigen::Vector3d& rpy);
	extern "C" void ti_rpy2mat(const Eigen::Vector3d& rpy, Eigen::Matrix3d& R);

	// in this coversion, hab = {heading (->), attitude (->), bank} are representing rotation w.r.t. y-axis, z-axis, and x-axis respectively.
	extern "C" void mat2hab(const Eigen::Matrix3d& R, Eigen::Vector3d& hab);
	extern "C" void hab2mat(const Eigen::Vector3d& hab, Eigen::Matrix3d& R);
}

#endif 
