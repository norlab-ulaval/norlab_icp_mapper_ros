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
	std::string initialMapFileName;
	std::string initialRobotPoseString;
	PM::TransformationParameters initialRobotPose;
	std::string finalMapFileName;
	std::string finalTrajectoryFileName;
	std::string configFilePath;
	float mapPublishRate;
	float mapTfPublishRate;
	float maxIdleTime;
	float sensorMaxRange;
	float priorDynamic;
	float thresholdDynamic;
	float beamHalfAngle;
	float epsilonA;
	float epsilonD;
	float alpha;
	float beta;
	bool is3D;
	bool isOnline;
	bool computeProbDynamic;
	bool isMapping;
	bool saveMapCellsOnHardDrive;
	bool publishTfsBetweenRegistrations;

	NodeParameters(rclcpp::Node& node);
};

#endif
