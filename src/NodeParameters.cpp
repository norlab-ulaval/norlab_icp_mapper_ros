#include "NodeParameters.h"
#include <fstream>

NodeParameters::NodeParameters(rclcpp::Node& node)
{
    declareParameters(node);
    retrieveParameters(node);
    parseComplexParameters();
    validateParameters();
}

void NodeParameters::declareParameters(rclcpp::Node& node)
{
    node.declare_parameter<std::string>("odom_frame", "odom");
    node.declare_parameter<std::string>("robot_frame", "base_link");
    node.declare_parameter<std::string>("mapping_config", "");
    node.declare_parameter<std::string>("initial_map_file_name", "");
    node.declare_parameter<std::string>("initial_robot_pose", "");
    node.declare_parameter<std::string>("final_map_file_name", "map.vtk");
    node.declare_parameter<std::string>("final_trajectory_file_name", "trajectory.vtk");
    node.declare_parameter<float>("map_publish_rate", 10);
    node.declare_parameter<float>("map_tf_publish_rate", 10);
    node.declare_parameter<float>("max_idle_time", 10);
    node.declare_parameter<bool>("is_3D", true);
    node.declare_parameter<bool>("is_mapping", true);
    node.declare_parameter<bool>("is_online", true);
    node.declare_parameter<bool>("save_map_cells_on_hard_drive", true);
    node.declare_parameter<bool>("publish_tfs_between_registrations", true);
    node.declare_parameter<bool>("deskew", true);
    node.declare_parameter<int>("expected_unique_deskewing_TF_number", 4000);
    node.declare_parameter<int>("deskewing_round_to_N_secs", 50000);
}

void NodeParameters::retrieveParameters(rclcpp::Node& node)
{
	node.get_parameter("odom_frame", odomFrame);
	node.get_parameter("robot_frame", robotFrame);
	node.get_parameter("mapping_config", mappingConfig);
	node.get_parameter("initial_map_file_name", initialMapFileName);
	node.get_parameter("initial_robot_pose", initialRobotPoseString);
	node.get_parameter("final_map_file_name", finalMapFileName);
	node.get_parameter("final_trajectory_file_name", finalTrajectoryFileName);
	node.get_parameter("map_publish_rate", mapPublishRate);
	node.get_parameter("map_tf_publish_rate", mapTfPublishRate);
	node.get_parameter("max_idle_time", maxIdleTime);
	node.get_parameter("is_3D", is3D);
	node.get_parameter("is_mapping", isMapping);
	node.get_parameter("is_online", isOnline);
	node.get_parameter("save_map_cells_on_hard_drive", saveMapCellsOnHardDrive);
	node.get_parameter("publish_tfs_between_registrations", publishTfsBetweenRegistrations);
	node.get_parameter("deskew", deskew);
	node.get_parameter("expected_unique_deskewing_TF_number", expectedUniqueDeskewingTFNumber);
	node.get_parameter("deskewing_round_to_N_secs", deskewingRoundToNSecs);
}

void NodeParameters::validateParameters() const
{
	if(!initialMapFileName.empty())
	{
		std::ifstream ifs(initialMapFileName.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid initial map file: " + initialMapFileName);
		}
		ifs.close();
	}

	if(!isOnline)
	{
		std::ofstream mapOfs(finalMapFileName.c_str(), std::ios_base::app);
		if(!mapOfs.good())
		{
			throw std::runtime_error("Invalid final map file: " + finalMapFileName);
		}
		mapOfs.close();

		std::ofstream trajectoryOfs(finalTrajectoryFileName.c_str(), std::ios_base::app);
		if(!trajectoryOfs.good())
		{
			throw std::runtime_error("Invalid final trajectory file: " + finalTrajectoryFileName);
		}
		trajectoryOfs.close();
	}

	if(!mappingConfig.empty())
	{
		std::ifstream ifs(mappingConfig.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid mapping config file: " + mappingConfig);
		}
		ifs.close();
	}

	if(mapPublishRate <= 0)
	{
		throw std::runtime_error("Invalid map publish rate: " + std::to_string(mapPublishRate));
	}

	if(publishTfsBetweenRegistrations)
	{
		if (mapTfPublishRate <= 0)
		{
			throw std::runtime_error("Invalid map tf publish rate: " + std::to_string(mapTfPublishRate));
		}
	}
	if(!isOnline)
	{
		if(maxIdleTime < 0)
		{
			throw std::runtime_error("Invalid max idle time: " + std::to_string(maxIdleTime));
		}
	}

	if(!isMapping && initialMapFileName.empty())
	{
		throw std::runtime_error("is mapping is set to false, but initial map file name was not specified.");
	}

	if (deskew)
	{
        if (expectedUniqueDeskewingTFNumber <= 0)
            {
                throw std::runtime_error("Expected unique deskewing TF value must be positive: " + std::to_string(expectedUniqueDeskewingTFNumber));
            }
        if (deskewingRoundToNSecs <= 0)
            {
                throw std::runtime_error("Deskewing round to nsecs value must be positive: " + std::to_string(deskewingRoundToNSecs));
            }
	}
}

void NodeParameters::parseComplexParameters()
{
	parseInitialRobotPose();
}

void NodeParameters::parseInitialRobotPose()
{
	if(!initialRobotPoseString.empty())
	{
		int homogeneousDim = is3D ? 4 : 3;
		initialRobotPose = PM::TransformationParameters::Identity(homogeneousDim, homogeneousDim);

		initialRobotPoseString.erase(std::remove(initialRobotPoseString.begin(), initialRobotPoseString.end(), '['), initialRobotPoseString.end());
		initialRobotPoseString.erase(std::remove(initialRobotPoseString.begin(), initialRobotPoseString.end(), ']'), initialRobotPoseString.end());
		std::replace(initialRobotPoseString.begin(), initialRobotPoseString.end(), ',', ' ');
		std::replace(initialRobotPoseString.begin(), initialRobotPoseString.end(), ';', ' ');

		float poseMatrix[homogeneousDim * homogeneousDim];
		std::stringstream poseStringStream(initialRobotPoseString);
		for(int i = 0; i < homogeneousDim * homogeneousDim; i++)
		{
			if(!(poseStringStream >> poseMatrix[i]))
			{
				throw std::runtime_error("An error occurred while trying to parse the initial robot pose.");
			}
		}

		float extraOutput = 0;
		if((poseStringStream >> extraOutput))
		{
			throw std::runtime_error("Invalid initial robot pose dimension.");
		}

		for(int i = 0; i < homogeneousDim * homogeneousDim; i++)
		{
			initialRobotPose(i / homogeneousDim, i % homogeneousDim) = poseMatrix[i];
		}
	}
}
