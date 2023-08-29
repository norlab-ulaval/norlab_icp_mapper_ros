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
    node.declare_parameter<std::string>("sensor_frame", "velodyne");
    node.declare_parameter<std::string>("robot_frame", "base_link");
    node.declare_parameter<std::string>("initial_map_file_name", "");
    node.declare_parameter<std::string>("initial_map_pose", "[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]");
    node.declare_parameter<std::string>("final_map_file_name", "map.vtk");
    node.declare_parameter<std::string>("final_map_pose_file_name", "final_map_pose.txt");
    node.declare_parameter<std::string>("final_trajectory_file_name", "trajectory.vtk");
    node.declare_parameter<std::string>("icp_config", "");
    node.declare_parameter<std::string>("input_filters_config", "");
    node.declare_parameter<std::string>("map_post_filters_config", "");
    node.declare_parameter<std::string>("map_update_condition", "overlap");
    node.declare_parameter<std::string>("mean_residual_file_name", "residual.csv");
    node.declare_parameter<std::string>("final_transformation_file_name", "final_transformation.txt");
    node.declare_parameter<std::string>("inertia_file_name", "inertia.csv");
    node.declare_parameter<std::string>("scan_directory", "");
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
    node.declare_parameter<bool>("compute_residual", false);
    node.declare_parameter<bool>("record_inertia", false);
    node.declare_parameter<bool>("perpendicular_residual", false);
    node.declare_parameter<bool>("point_to_plane_residual", true);
    node.declare_parameter<bool>("use_crv_model", false);
    node.declare_parameter<bool>("use_icra_model", false);
    node.declare_parameter<bool>("after_deskewing", true);
    node.declare_parameter<bool>("soft_uncertainty_threshold", true);
    node.declare_parameter<bool>("is_mapping", true);
    node.declare_parameter<int>("skew_model", 0);
    node.declare_parameter<float>("corner_point_uncertainty", 0.0);
    node.declare_parameter<float>("uncertainty_threshold", 1000.0);
    node.declare_parameter<float>("uncertainty_quantile", 1.0);
    node.declare_parameter<float>("binary_uncertainty_threshold", 0.03);
    node.declare_parameter<float>("scale_factor", 1);
}

void NodeParameters::retrieveParameters(rclcpp::Node& node)
{
	node.get_parameter("odom_frame", odomFrame);
	node.get_parameter("sensor_frame", sensorFrame);
	node.get_parameter("robot_frame", robotFrame);
	node.get_parameter("initial_map_file_name", initialMapFileName);
	node.get_parameter("initial_map_pose", initialMapPoseString);
	node.get_parameter("final_map_file_name", finalMapFileName);
	node.get_parameter("final_map_pose_file_name", finalMapPoseFileName);
	node.get_parameter("final_trajectory_file_name", finalTrajectoryFileName);
	node.get_parameter("icp_config", icpConfig);
	node.get_parameter("input_filters_config", inputFiltersConfig);
	node.get_parameter("map_post_filters_config", mapPostFiltersConfig);
	node.get_parameter("map_update_condition", mapUpdateCondition);
	node.get_parameter("mean_residual_file_name", meanResidualFileName);
	node.get_parameter("final_transformation_file_name", finalTransformationFileName);
	node.get_parameter("inertia_file_name", inertiaFileName);
    node.get_parameter("scan_directory", scanDirectory);
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
	node.get_parameter("compute_residual", computeResidual);
	node.get_parameter("record_inertia", recordInertia);
	node.get_parameter("perpendicular_residual", perpendicularResidual);
	node.get_parameter("point_to_plane_residual", pointToPlaneResidual);
	node.get_parameter("use_crv_model", useCRVModel);
	node.get_parameter("use_icra_model", useICRAModel);
	node.get_parameter("after_deskewing", afterDeskewing);
	node.get_parameter("soft_uncertainty_threshold", softUncertaintyThreshold);
	node.get_parameter("is_mapping", isMapping);
	node.get_parameter("skew_model", skewModel);
	node.get_parameter("corner_point_uncertainty", cornerPointUncertainty);
	node.get_parameter("uncertainty_threshold", uncertaintyThreshold);
	node.get_parameter("uncertainty_quantile", uncertaintyQuantile);
	node.get_parameter("binary_uncertainty_threshold", binaryUncertaintyThreshold);
	node.get_parameter("scale_factor", scaleFactor);
}

void NodeParameters::validateParameters()
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
	parseInitialMapPose();
}

void NodeParameters::parseInitialMapPose()
{
	int homogeneousDim = is3D ? 4 : 3;
	initialMapPose = PM::TransformationParameters::Identity(homogeneousDim, homogeneousDim);
	
	if(!initialMapFileName.empty())
	{
		initialMapPoseString.erase(std::remove(initialMapPoseString.begin(), initialMapPoseString.end(), '['), initialMapPoseString.end());
		initialMapPoseString.erase(std::remove(initialMapPoseString.begin(), initialMapPoseString.end(), ']'), initialMapPoseString.end());
		std::replace(initialMapPoseString.begin(), initialMapPoseString.end(), ',', ' ');
		std::replace(initialMapPoseString.begin(), initialMapPoseString.end(), ';', ' ');
		
		float poseMatrix[homogeneousDim * homogeneousDim];
		std::stringstream poseStringStream(initialMapPoseString);
		for(int i = 0; i < homogeneousDim * homogeneousDim; i++)
		{
			if(!(poseStringStream >> poseMatrix[i]))
			{
				throw std::runtime_error("An error occurred while trying to parse the initial map pose.");
			}
		}
		
		float extraOutput = 0;
		if((poseStringStream >> extraOutput))
		{
			throw std::runtime_error("Invalid initial map pose dimension.");
		}
		
		for(int i = 0; i < homogeneousDim * homogeneousDim; i++)
		{
			initialMapPose(i / homogeneousDim, i % homogeneousDim) = poseMatrix[i];
		}
	}
}
