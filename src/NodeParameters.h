#ifndef NODE_PARAMETERS_H
#define NODE_PARAMETERS_H

#include <rclcpp/rclcpp.hpp>
#include <norlab_icp_mapper/Mapper.h>

class NodeParameters
{
private:
	typedef PointMatcher<float> PM;

	void declareParameters(rclcpp::Node& node);
	void retrieveParameters(rclcpp::Node& node);
	void validateParameters() const;
	void parseComplexParameters();
	void parseInitialRobotPose();

public:
	std::string odomFrame;
	std::string robotFrame;
	std::string mappingConfig;
	std::string initialMapFileName;
	std::string initialRobotPoseString;
	PM::TransformationParameters initialRobotPose;
	std::string finalMapFileName;
	std::string finalTrajectoryFileName;
	float mapPublishRate;
	float mapTfPublishRate;
	float maxIdleTime;
    bool is3D;
	bool isMapping;
	bool isOnline;
	bool saveMapCellsOnHardDrive;
	bool publishTfsBetweenRegistrations;
	bool deskew;

	NodeParameters(rclcpp::Node& node);
};

#endif
