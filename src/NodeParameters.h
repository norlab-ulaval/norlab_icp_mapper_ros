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
    void parseImuToLidar();

public:
    std::string robotFrame;
    std::string initialMapFileName;
    std::string initialRobotPoseString;
    PM::TransformationParameters initialRobotPose;
    std::string finalMapFileName;
    std::string finalTrajectoryFileName;
    std::string icpConfig;
    std::string inputFiltersConfig;
    std::string mapPostFiltersConfig;
    std::string mapUpdateCondition;
    float mapUpdateOverlap;
    float mapUpdateDelay;
    float mapUpdateDistance;
    float mapPublishRate;
    float mapTfPublishRate;
    float maxIdleTime;
    float minDistNewPoint;
    float sensorMaxRange;
    float priorDynamic;
    float thresholdDynamic;
    float beamHalfAngle;
    float epsilonA;
    float epsilonD;
    float alpha;
    float beta;
    bool is3D;
    bool computeProbDynamic;
    bool isMapping;
    bool saveMapCellsOnHardDrive;
    bool publishTfsBetweenRegistrations;
    std::string imuToLidarString;
    PM::TransformationParameters imuToLidar;

    NodeParameters(rclcpp::Node& node);
};

#endif
