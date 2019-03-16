/**
 * COPYRIGHT and PERMISSION NOTICE
 * Real Deal Robotics Software: Pose Estimation
 * Copyright (C) 2019 Real Deal Robotics, LLC
 * All rights Reserved
 */
 
#ifndef RDR_POSE_ESTIMATION_H
#define RDR_POSE_ESTIMATION_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
//#include "LinearMath/btMatrix3x3.h"
#include <rdr_spline_path/rdr_utilities.h>

struct RdrRpy {
	double roll;
	double pitch;
	double yaw;
};

struct RdrPose {
	
	Eigen::Quaterniond q;
	Eigen::Vector3d p;
	RdrRpy rpy;
};

class poseEstimation
{
public:
	poseEstimation(ros::NodeHandle& nh)
	{
		_nh = nh;
	}
	
	bool getVehiclePose(RdrPose& pose)
	{
		try {
			_listener.lookupTransform("/world", 
				"uav/imu",
				ros::Time(0), 
				_transform
				);
		} catch (tf::TransformException ex) {
			ROS_ERROR_THROTTLE(1.0, "%s", ex.what() );
			return false;
		}
		
		//! Convert the transform quaternion to an Eigen Quaternion:
		tf::quaternionTFToEigen(_transform.getRotation(), pose.q);
		
		//! Convert the quaternion to roll pitch yaw:				
		tf::Matrix3x3(_transform.getRotation()).getEulerYPR(
			pose.rpy.yaw, pose.rpy.pitch, pose.rpy.roll);
			
		//! Convert the translation from the world frame into Eigen Vector3:
		tf::vectorTFToEigen(_transform.getOrigin(), pose.p);
		
		return true;
	}
	
private:
	ros::NodeHandle _nh;
	tf::TransformListener _listener;
	tf::StampedTransform _transform;
};


#endif // RDR_POSE_ESTIMATION_H
