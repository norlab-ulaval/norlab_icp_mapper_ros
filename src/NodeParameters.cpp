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
    node.declare_parameter<std::string>("initial_map_file_name", "");
    node.declare_parameter<std::string>("initial_robot_pose", "");
    node.declare_parameter<std::string>("final_map_file_name", "map.vtk");
    node.declare_parameter<std::string>("final_trajectory_file_name", "trajectory.vtk");
    node.declare_parameter<std::string>("icp_config", "");
    node.declare_parameter<std::string>("input_filters_config", "");
    node.declare_parameter<std::string>("map_post_filters_config", "");
    node.declare_parameter<std::string>("map_update_condition", "overlap");
    node.declare_parameter<float>("map_update_overlap", 0.9);
    node.declare_parameter<float>("map_update_delay", 1);
    node.declare_parameter<float>("map_update_distance", 0.5);
    node.declare_parameter<float>("map_publish_rate", 10);
    node.declare_parameter<float>("map_tf_publish_rate", 10);
    node.declare_parameter<float>("max_idle_time", 10);
    node.declare_parameter<float>("min_dist_new_point", 0.03);
    node.declare_parameter<float>("sensor_max_range", 80);
    node.declare_parameter<float>("prior_dynamic", 0.6);
    node.declare_parameter<float>("threshold_dynamic", 0.9);
    node.declare_parameter<float>("beam_half_angle", 0.01);
    node.declare_parameter<float>("epsilon_a", 0.01);
    node.declare_parameter<float>("epsilon_d", 0.01);
    node.declare_parameter<float>("alpha", 0.8);
    node.declare_parameter<float>("beta", 0.99);
    node.declare_parameter<bool>("is_3D", true);
    node.declare_parameter<bool>("is_online", true);
    node.declare_parameter<bool>("compute_prob_dynamic", false);
    node.declare_parameter<bool>("is_mapping", true);
    node.declare_parameter<bool>("save_map_cells_on_hard_drive", true);
    node.declare_parameter<bool>("publish_tfs_between_registrations", true);
}

void NodeParameters::retrieveParameters(rclcpp::Node& node)
{
	node.get_parameter("odom_frame", odomFrame);
	node.get_parameter("robot_frame", robotFrame);
	node.get_parameter("initial_map_file_name", initialMapFileName);
	node.get_parameter("initial_robot_pose", initialRobotPoseString);
	node.get_parameter("final_map_file_name", finalMapFileName);
	node.get_parameter("final_trajectory_file_name", finalTrajectoryFileName);
	node.get_parameter("icp_config", icpConfig);
	node.get_parameter("input_filters_config", inputFiltersConfig);
	node.get_parameter("map_post_filters_config", mapPostFiltersConfig);
	node.get_parameter("map_update_condition", mapUpdateCondition);
	node.get_parameter("map_update_overlap", mapUpdateOverlap);
	node.get_parameter("map_update_delay", mapUpdateDelay);
	node.get_parameter("map_update_distance", mapUpdateDistance);
	node.get_parameter("map_publish_rate", mapPublishRate);
	node.get_parameter("map_tf_publish_rate", mapTfPublishRate);
	node.get_parameter("max_idle_time", maxIdleTime);
	node.get_parameter("min_dist_new_point", minDistNewPoint);
	node.get_parameter("sensor_max_range", sensorMaxRange);
	node.get_parameter("prior_dynamic", priorDynamic);
	node.get_parameter("threshold_dynamic", thresholdDynamic);
	node.get_parameter("beam_half_angle", beamHalfAngle);
	node.get_parameter("epsilon_a", epsilonA);
	node.get_parameter("epsilon_d", epsilonD);
	node.get_parameter("alpha", alpha);
	node.get_parameter("beta", beta);
	node.get_parameter("is_3D", is3D);
	node.get_parameter("is_online", isOnline);
	node.get_parameter("compute_prob_dynamic", computeProbDynamic);
	node.get_parameter("is_mapping", isMapping);
	node.get_parameter("save_map_cells_on_hard_drive", saveMapCellsOnHardDrive);
	node.get_parameter("publish_tfs_between_registrations", publishTfsBetweenRegistrations);
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

	if(!icpConfig.empty())
	{
		std::ifstream ifs(icpConfig.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid icp config file: " + icpConfig);
		}
		ifs.close();
	}

	if(!inputFiltersConfig.empty())
	{
		std::ifstream ifs(inputFiltersConfig.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid input filters config file: " + inputFiltersConfig);
		}
		ifs.close();
	}

	if(!mapPostFiltersConfig.empty())
	{
		std::ifstream ifs(mapPostFiltersConfig.c_str());
		if(!ifs.good())
		{
			throw std::runtime_error("Invalid map post filters config file: " + mapPostFiltersConfig);
		}
		ifs.close();
	}

	if(mapUpdateCondition != "overlap" && mapUpdateCondition != "delay" && mapUpdateCondition != "distance")
	{
		throw std::runtime_error("Invalid map update condition: " + mapUpdateCondition);
	}

	if(mapUpdateOverlap < 0 || mapUpdateOverlap > 1)
	{
		throw std::runtime_error("Invalid map update overlap: " + std::to_string(mapUpdateOverlap));
	}

	if(mapUpdateDelay < 0)
	{
		throw std::runtime_error("Invalid map update delay: " + std::to_string(mapUpdateDelay));
	}

	if(mapUpdateDistance < 0)
	{
		throw std::runtime_error("Invalid map update distance: " + std::to_string(mapUpdateDistance));
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

	if(minDistNewPoint < 0)
	{
		throw std::runtime_error("Invalid minimum distance of new point: " + std::to_string(minDistNewPoint));
	}

	if(sensorMaxRange < 0)
	{
		throw std::runtime_error("Invalid sensor max range: " + std::to_string(sensorMaxRange));
	}

	if(priorDynamic < 0 || priorDynamic > 1)
	{
		throw std::runtime_error("Invalid prior dynamic: " + std::to_string(priorDynamic));
	}

	if(thresholdDynamic < 0 || thresholdDynamic > 1)
	{
		throw std::runtime_error("Invalid threshold dynamic: " + std::to_string(thresholdDynamic));
	}

	if(beamHalfAngle < 0 || beamHalfAngle > M_PI_2)
	{
		throw std::runtime_error("Invalid beam half angle: " + std::to_string(beamHalfAngle));
	}

	if(epsilonA < 0)
	{
		throw std::runtime_error("Invalid epsilon a: " + std::to_string(epsilonA));
	}

	if(epsilonD < 0)
	{
		throw std::runtime_error("Invalid epsilon d: " + std::to_string(epsilonD));
	}

	if(alpha < 0 || alpha > 1)
	{
		throw std::runtime_error("Invalid alpha: " + std::to_string(alpha));
	}

	if(beta < 0 || beta > 1)
	{
		throw std::runtime_error("Invalid beta: " + std::to_string(beta));
	}

	if(!isMapping && initialMapFileName.empty())
	{
		throw std::runtime_error("is mapping is set to false, but initial map file name was not specified.");
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
