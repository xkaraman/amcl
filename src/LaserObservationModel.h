/*
 * LaserObservationModel.h
 *
 *  Created on: Nov 5, 2018
 *      Author: xenakis
 */

#ifndef SRC_LASEROBSERVATIONMODEL_H_
#define SRC_LASEROBSERVATIONMODEL_H_

#include <libPF/ObservationModel.h>
#include <amcl_depth_types.h>

#include <MapModel.h>
#include <RobotState.h>
#include <memory>
#include <vector>
#include <tf2/LinearMath/Transform.h>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointXYZRGB PointRGBT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointCloud<PointRGBT> PointCloudRGB;

class LaserObservationModel: public libPF::ObservationModel<RobotState> {
public:

	LaserObservationModel(ros::NodeHandle *nh,std::shared_ptr<MapModel> mapModel);

	virtual ~LaserObservationModel();

    /**
     * Calculate the weight for the current state
     * @param state Reference to the state that has to be weightened.
     * @return weight for the given state.
     */
	double measure(const RobotState &state) const;

    /**
     * Set the map in ColorOcTree Octomap Structure
     * @param map Map to be saved
     */
	void setMap(const std::shared_ptr<octomap::ColorOcTree> &map);

	/**
	 * Set the transform between "base"->"target_sensor" TF frame
	 * @param baseToSensor Transform between base and sensor frame;
	 *
	 */
	void setBaseToSensorTransform(const tf2::Transform &baseToSensor);

	/**
	 * Set the measurements received from sensors along with their respective range from
	 * point of origin
	 * @param observed The measurements received from sensors in PoinctCloud (PCL Library)
	 * @param ranges The ranges of each Point in PointCloud from the source of origin
	 */
	void setObservedMeasurements(const PointCloud &observed,
			const std::vector<float> &ranges);

private:
	tf2::Transform m_BaseToSensorTransform;

	std::shared_ptr<octomap::ColorOcTree> m_Map;

	PointCloud m_observedMeasurement;
	std::vector<float> m_observedRanges;

	bool m_RGB;
	double m_ZHit;
	double m_ZShort;
	double m_ZRand;
	double m_ZMax;
	double m_SigmaHit;
	double m_LambdaShort;
};

#endif /* SRC_LASEROBSERVATIONMODEL_H_ */
