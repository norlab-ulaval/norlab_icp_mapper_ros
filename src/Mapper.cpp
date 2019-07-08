#include "Mapper.h"
#include <nabo/nabo.h>
#include <fstream>
#include <chrono>

Mapper::Mapper(std::string icpConfigFilePath, PM::DataPointsFilters inputFilters, PM::DataPointsFilters mapPostFilters, std::string mapUpdateCondition,
			   double mapUpdateOverlap, double mapUpdateDelay, double mapUpdateDistance, double minDistNewPoint, double sensorMaxRange,
			   double priorDynamic, double thresholdDynamic, double beamHalfAngle, double epsilonA, double epsilonD, double alpha, double beta,
			   bool is3D, bool isOnline, bool computeProbDynamic):
		inputFilters(inputFilters),
		mapPostFilters(mapPostFilters),
		transformation(PM::get().TransformationRegistrar.create("RigidTransformation")),
		mapUpdateCondition(mapUpdateCondition),
		mapUpdateOverlap(mapUpdateOverlap),
		mapUpdateDelay(mapUpdateDelay),
		mapUpdateDistance(mapUpdateDistance),
		minDistNewPoint(minDistNewPoint),
		sensorMaxRange(sensorMaxRange),
		priorDynamic(priorDynamic),
		thresholdDynamic(thresholdDynamic),
		beamHalfAngle(beamHalfAngle),
		epsilonA(epsilonA),
		epsilonD(epsilonD),
		alpha(alpha),
		beta(beta),
		is3D(is3D),
		isOnline(isOnline),
		computeProbDynamic(computeProbDynamic),
		newMapAvailable(false)
{
	if(icpConfigFilePath != "")
	{
		std::ifstream ifs(icpConfigFilePath.c_str());
		if(ifs.good())
		{
			icp.loadFromYaml(ifs);
		}
		else
		{
			icp.setDefault();
		}
	}
	else
	{
		icp.setDefault();
	}
	
	PM::Parameters radiusFilterParams;
	radiusFilterParams["dim"] = "-1";
	radiusFilterParams["dist"] = std::to_string(sensorMaxRange);
	radiusFilterParams["removeInside"] = "0";
	radiusFilter = PM::get().DataPointsFilterRegistrar.create("DistanceLimitDataPointsFilter", radiusFilterParams);
	
	int nbRows = is3D ? 4 : 3;
	sensorPose = PM::Matrix::Identity(nbRows, nbRows);
}

void Mapper::processCloud(PM::DataPoints& cloudInSensorFrame, PM::TransformationParameters& estimatedSensorPose, std::time_t timeStamp)
{
	inputFilters.apply(cloudInSensorFrame);
	
	PM::DataPoints cloudInMapFrameBeforeCorrection = transformation->compute(cloudInSensorFrame, estimatedSensorPose);
	
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	if(currentMap.getNbPoints() == 0)
	{
		sensorPose = estimatedSensorPose;
		
		updateMap(cloudInMapFrameBeforeCorrection, timeStamp);
	}
	else
	{
		PM::DataPoints cutMapInSensorFrame = transformation->compute(currentMap, estimatedSensorPose.inverse());
		radiusFilter->inPlaceFilter(cutMapInSensorFrame);                                              // TODO: find efficient way to compute this...
		PM::DataPoints cutMap = transformation->compute(cutMapInSensorFrame, estimatedSensorPose);
		
		PM::TransformationParameters correction = icp(cloudInMapFrameBeforeCorrection, cutMap);
		sensorPose = correction * estimatedSensorPose;
		
		if(shouldUpdateMap(timeStamp, sensorPose, icp.errorMinimizer->getOverlap()))
		{
			updateMap(transformation->compute(cloudInMapFrameBeforeCorrection, correction), timeStamp);
		}
	}
}

bool Mapper::shouldUpdateMap(const std::time_t& currentTime, const PM::TransformationParameters& currentSensorPose, const double& currentOverlap)
{
	if(isOnline)
	{
		// if previous map is not done building
		if(mapBuilderFuture.valid() && mapBuilderFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
		{
			return false;
		}
	}
	
	if(mapUpdateCondition == "overlap")
	{
		return currentOverlap < mapUpdateOverlap;
	}
	else if(mapUpdateCondition == "delay")
	{
		return std::difftime(currentTime, lastTimeMapWasUpdated) > mapUpdateDelay;
	}
	else if(mapUpdateCondition == "distance")
	{
		int nbRows = is3D ? 3 : 2;
		PM::Vector lastSensorLocation = lastSensorPoseWhereMapWasUpdated.topRightCorner(nbRows, 1);
		PM::Vector currentSensorLocation = currentSensorPose.topRightCorner(nbRows, 1);
		return std::abs((currentSensorLocation - lastSensorLocation).norm()) > mapUpdateDistance;
	}
	else
	{
		throw std::runtime_error("Invalid map update condition: " + mapUpdateCondition);
	}
}

void Mapper::updateMap(const PM::DataPoints& currentCloud, const std::time_t& timeStamp)
{
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	lastTimeMapWasUpdated = timeStamp;
	lastSensorPoseWhereMapWasUpdated = sensorPose;
	
	if(isOnline && currentMap.getNbPoints() != 0)
	{
		mapBuilderFuture = std::async(&Mapper::buildMap, this, currentCloud, currentMap, sensorPose);
	}
	else
	{
		buildMap(currentCloud, currentMap, sensorPose);
	}
}

void Mapper::buildMap(PM::DataPoints currentCloud, PM::DataPoints currentMap, PM::TransformationParameters currentSensorPose)
{
	if(computeProbDynamic)
	{
		currentCloud.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, currentCloud.features.cols(), priorDynamic));
	}
	
	if(currentMap.getNbPoints() == 0)
	{
		currentMap = currentCloud;
	}
	else
	{
		PM::DataPoints cloudPointsToKeep = retrievePointsFurtherThanMinDistNewPoint(currentCloud, currentMap);
		
		if(computeProbDynamic)
		{
			computeProbabilityOfPointsBeingDynamic(currentCloud, currentMap, currentSensorPose);
		}
		
		currentMap.concatenate(cloudPointsToKeep);
	}
	
	PM::DataPoints mapInSensorFrame = transformation->compute(currentMap, currentSensorPose.inverse());
	mapPostFilters.apply(mapInSensorFrame);                                              // TODO: find efficient way to compute this...
	currentMap = transformation->compute(mapInSensorFrame, currentSensorPose);
	
	if(computeProbDynamic && !currentMap.descriptorExists("normals"))
	{
		throw std::runtime_error("compute_prob_dynamic is set to true, but field normals does not exist for map points.");
	}
	
	mapLock.lock();
	map = currentMap;
	newMapAvailable = true;
	mapLock.unlock();
}

PM::DataPoints Mapper::retrievePointsFurtherThanMinDistNewPoint(const PM::DataPoints& currentCloud, const PM::DataPoints& currentMap)
{
	typedef Nabo::NearestNeighbourSearch<T> NNS;
	
	PM::Matches matches(PM::Matches::Dists(1, currentCloud.getNbPoints()), PM::Matches::Ids(1, currentCloud.getNbPoints()));
	std::shared_ptr<NNS> nns = std::shared_ptr<NNS>(NNS::create(currentMap.features, currentMap.features.rows() - 1,
																NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
	
	nns->knn(currentCloud.features, matches.ids, matches.dists, 1, 0);
	
	int goodPointCount = 0;
	PM::DataPoints goodPoints(currentCloud.createSimilarEmpty());
	for(int i = 0; i < currentCloud.getNbPoints(); ++i)
	{
		if(matches.dists(i) >= std::pow(minDistNewPoint, 2))
		{
			goodPoints.setColFrom(goodPointCount, currentCloud, i);
			goodPointCount++;
		}
	}
	goodPoints.conservativeResize(goodPointCount);
	
	return goodPoints;
}

void Mapper::computeProbabilityOfPointsBeingDynamic(const PM::DataPoints& currentCloud, PM::DataPoints& currentMap,
													const PM::TransformationParameters& currentSensorPose)
{
	typedef Nabo::NearestNeighbourSearch<T> NNS;
	const int nbRows = is3D ? 3 : 2;
	const double eps = 0.0001;
	
	PM::DataPoints currentCloudInSensorFrame = transformation->compute(currentCloud, currentSensorPose.inverse());
	
	PM::Matrix currentCloudInSensorFrameRadii;
	PM::Matrix currentCloudInSensorFrameAngles;
	convertToSphericalCoordinates(currentCloudInSensorFrame, currentCloudInSensorFrameRadii, currentCloudInSensorFrameAngles);
	
	PM::DataPoints cutMapInSensorFrame = transformation->compute(currentMap, currentSensorPose.inverse());
	PM::Matrix globalId(1, currentMap.getNbPoints());
	int nbPointsCutMap = 0;
	for(int i = 0; i < currentMap.getNbPoints(); i++)
	{
		if(cutMapInSensorFrame.features.col(i).head(nbRows).norm() < sensorMaxRange)
		{
			cutMapInSensorFrame.setColFrom(nbPointsCutMap, cutMapInSensorFrame, i);
			globalId(0, nbPointsCutMap) = i;
			nbPointsCutMap++;
		}
	}
	cutMapInSensorFrame.conservativeResize(nbPointsCutMap);
	
	PM::Matrix cutMapInSensorFrameRadii;
	PM::Matrix cutMapInSensorFrameAngles;
	convertToSphericalCoordinates(cutMapInSensorFrame, cutMapInSensorFrameRadii, cutMapInSensorFrameAngles);
	
	std::shared_ptr<NNS> nns = std::shared_ptr<NNS>(NNS::create(currentCloudInSensorFrameAngles));
	PM::Matches::Dists dists(1, cutMapInSensorFrame.getNbPoints());
	PM::Matches::Ids ids(1, cutMapInSensorFrame.getNbPoints());
	nns->knn(cutMapInSensorFrameAngles, ids, dists, 1, 0, NNS::ALLOW_SELF_MATCH, 2 * beamHalfAngle);
	
	PM::DataPoints::View viewOnProbabilityDynamic = currentMap.getDescriptorViewByName("probabilityDynamic");
	PM::DataPoints::View viewOnMapNormals = cutMapInSensorFrame.getDescriptorViewByName("normals");
	for(int i = 0; i < cutMapInSensorFrame.getNbPoints(); i++)
	{
		if(dists(i) != std::numeric_limits<float>::infinity())
		{
			const int readingPointId = ids(0, i);
			const int mapPointId = globalId(0, i);
			
			const Eigen::VectorXf readingPoint = currentCloudInSensorFrame.features.col(readingPointId).head(nbRows);
			const Eigen::VectorXf mapPoint = cutMapInSensorFrame.features.col(i).head(nbRows);
			const float delta = (readingPoint - mapPoint).norm();
			const float d_max = epsilonA * readingPoint.norm();
			
			const Eigen::VectorXf mapPointNormal = viewOnMapNormals.col(i);
			
			const float w_v = eps + (1. - eps) * fabs(mapPointNormal.dot(mapPoint.normalized()));
			const float w_d1 = eps + (1. - eps) * (1. - sqrt(dists(i)) / (2 * beamHalfAngle));
			
			const float offset = delta - epsilonD;
			float w_d2 = 1.;
			if(delta < epsilonD || mapPoint.norm() > readingPoint.norm())
			{
				w_d2 = eps;
			}
			else
			{
				if(offset < d_max)
				{
					w_d2 = eps + (1 - eps) * offset / d_max;
				}
			}
			
			float w_p2 = eps;
			if(delta < epsilonD)
			{
				w_p2 = 1;
			}
			else
			{
				if(offset < d_max)
				{
					w_p2 = eps + (1. - eps) * (1. - offset / d_max);
				}
			}
			
			if((readingPoint.norm() + epsilonD + d_max) >= mapPoint.norm())
			{
				const float lastDyn = viewOnProbabilityDynamic(0, mapPointId);
				
				const float c1 = (1 - (w_v * w_d1));
				const float c2 = w_v * w_d1;
				
				double probDynamic;
				double probStatic;
				if(lastDyn < thresholdDynamic)
				{
					probDynamic = c1 * lastDyn + c2 * w_d2 * ((1 - alpha) * (1 - lastDyn) + beta * lastDyn);
					probStatic = c1 * (1 - lastDyn) + c2 * w_p2 * (alpha * (1 - lastDyn) + (1 - beta) * lastDyn);
				}
				else
				{
					probDynamic = 1 - eps;
					probStatic = eps;
				}
				
				viewOnProbabilityDynamic(0, mapPointId) = probDynamic / (probDynamic + probStatic);
			}
		}
	}
}

void Mapper::convertToSphericalCoordinates(const PM::DataPoints& points, PM::Matrix& radii, PM::Matrix& angles)
{
	const int nbRows = is3D ? 3 : 2;
	radii = points.features.topRows(nbRows).colwise().norm();
	angles = PM::Matrix(2, points.getNbPoints());
	
	for(int i = 0; i < points.getNbPoints(); i++)
	{
		angles(0, i) = 0;
		if(is3D)
		{
			const float ratio = points.features(2, i) / radii(0, i);
			angles(0, i) = asin(ratio);
		}
		angles(1, i) = atan2(points.features(1, i), points.features(0, i));
	}
}

PM::DataPoints Mapper::getMap()
{
	mapLock.lock();
	PM::DataPoints currentMap = map;
	mapLock.unlock();
	
	return currentMap;
}

bool Mapper::getNewMap(PM::DataPoints& mapOut)
{
	bool mapReturned = false;
	
	mapLock.lock();
	if(newMapAvailable)
	{
		mapOut = map;
		newMapAvailable = false;
		mapReturned = true;
	}
	mapLock.unlock();
	
	return mapReturned;
}

const PM::TransformationParameters& Mapper::getSensorPose()
{
	return sensorPose;
}
