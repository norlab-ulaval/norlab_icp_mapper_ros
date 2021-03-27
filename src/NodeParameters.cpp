#include "NodeParameters.h"
#include <fstream>

NodeParameters::NodeParameters(ros::NodeHandle privateNodeHandle)
{
	retrieveParameters(privateNodeHandle);
	parseComplexParameters();
	validateParameters();
}

void NodeParameters::retrieveParameters(const ros::NodeHandle& nodeHandle)
{
	nodeHandle.param<std::string>("odom_frame", odomFrame, "odom");
	nodeHandle.param<std::string>("robot_frame", robotFrame, "base_link");
	nodeHandle.param<std::string>("initial_map_file_name", initialMapFileName, "");
	nodeHandle.param<std::string>("initial_robot_pose", initialRobotPoseString, "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]");
	nodeHandle.param<std::string>("final_map_file_name", finalMapFileName, "map.vtk");
	nodeHandle.param<std::string>("final_trajectory_file_name", finalTrajectoryFileName, "trajectory.vtk");
	nodeHandle.param<std::string>("icp_config", icpConfig, "");
	nodeHandle.param<std::string>("input_filters_config", inputFiltersConfig, "");
	nodeHandle.param<std::string>("map_post_filters_config", mapPostFiltersConfig, "");
	nodeHandle.param<std::string>("map_update_condition", mapUpdateCondition, "overlap");
	nodeHandle.param<float>("map_update_overlap", mapUpdateOverlap, 0.9);
	nodeHandle.param<float>("map_update_delay", mapUpdateDelay, 1);
	nodeHandle.param<float>("map_update_distance", mapUpdateDistance, 0.5);
	nodeHandle.param<float>("map_publish_rate", mapPublishRate, 10);
	nodeHandle.param<float>("map_tf_publish_rate", mapTfPublishRate, 10);
	nodeHandle.param<float>("max_idle_time", maxIdleTime, 10);
	nodeHandle.param<float>("min_dist_new_point", minDistNewPoint, 0.03);
	nodeHandle.param<float>("sensor_max_range", sensorMaxRange, 80);
	nodeHandle.param<float>("prior_dynamic", priorDynamic, 0.6);
	nodeHandle.param<float>("threshold_dynamic", thresholdDynamic, 0.9);
	nodeHandle.param<float>("beam_half_angle", beamHalfAngle, 0.01);
	nodeHandle.param<float>("epsilon_a", epsilonA, 0.01);
	nodeHandle.param<float>("epsilon_d", epsilonD, 0.01);
	nodeHandle.param<float>("alpha", alpha, 0.8);
	nodeHandle.param<float>("beta", beta, 0.99);
	nodeHandle.param<bool>("is_3D", is3D, true);
	nodeHandle.param<bool>("is_online", isOnline, true);
	nodeHandle.param<bool>("compute_prob_dynamic", computeProbDynamic, false);
	nodeHandle.param<bool>("is_mapping", isMapping, true);
	nodeHandle.param<bool>("save_map_cells_on_hard_drive", saveMapCellsOnHardDrive, true);
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

	if(mapTfPublishRate <= 0)
	{
		throw std::runtime_error("Invalid map tf publish rate: " + std::to_string(mapTfPublishRate));
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
