#ifndef NODE_PARAMETERS_H
#define NODE_PARAMETERS_H

#include <rclcpp/rclcpp.hpp>
#include <norlab_icp_mapper/Mapper.h>

class NodeParameters
{
private:
    typedef norlab_icp_mapper::PM PM;
    typedef norlab_icp_mapper::T T;

    void declareParameters(rclcpp::Node& node);
    void retrieveParameters(rclcpp::Node& node);
    void validateParameters();
    void parseComplexParameters();
    void parseInitialMapPose();

public:
    std::string odomFrame;
    std::string sensorFrame;
    std::string robotFrame;
    std::string initialMapFileName;
    std::string initialMapPoseString;
    PM::TransformationParameters initialMapPose;
    std::string finalMapFileName;
    std::string finalMapPoseFileName;
    std::string finalTrajectoryFileName;
    std::string icpConfig;
    std::string inputFiltersConfig;
    std::string mapPostFiltersConfig;
    std::string mapUpdateCondition;
    std::string meanResidualFileName;
    std::string finalTransformationFileName;
    std::string inertiaFileName;
    std::string scanDirectory;
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
    bool isOnline;
    bool computeProbDynamic;
    bool computeResidual;
    bool recordInertia;
    bool perpendicularResidual;
    bool pointToPlaneResidual;
    bool useCRVModel;
    bool useICRAModel;
    bool afterDeskewing;
    bool softUncertaintyThreshold;
    bool isMapping;
    int skewModel;
    float cornerPointUncertainty;
    float uncertaintyThreshold;
    float uncertaintyQuantile;
    float binaryUncertaintyThreshold;
    float scaleFactor;

    NodeParameters(rclcpp::Node& node);
};

#endif
