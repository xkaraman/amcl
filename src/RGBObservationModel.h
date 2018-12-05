/*
 * RGBObservationModel.h
 *
 *  Created on: Nov 5, 2018
 *      Author: xenakis
 */

#ifndef SRC_RGBOBSERVATIONMODEL_H_
#define SRC_RGBOBSERVATIONMODEL_H_

////#include <FeatureMatching.h>

#include <amcl_depth_types.h>
#include <libPF/ObservationModel.h>
#include <RobotState.h>
#include <tf2/LinearMath/Transform.h>
#include <memory>
#include <ros/ros.h>

class MapModel;
namespace octomap {
class ColorOcTree;
} /* namespace octomap */


class RGBObservationModel: public libPF::ObservationModel<RobotState> {
public:

	RGBObservationModel(ros::NodeHandle *nh,
			std::shared_ptr<MapModel> mapModel);

	virtual ~RGBObservationModel();

	/**
	 * Calculate the weight for the current state
	 * @param state Reference to the state that has to be weightened.
	 * @return weight for the given state.
	 */
	double measure(const RobotState& state) const;

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
	void setObservedMeasurements(const PointCloudRGB &observed);

	void setRGB(const bool rgb);

	void setMinMax3D(PointRGBT min,PointRGBT max);

private:

	tf2::Transform m_BaseToSensorTransform;
	PointCloudRGB m_ObservedMeasurementRGB;
	PointRGBT m_min3D;
	PointRGBT m_max3D;

	bool m_RGB;
	std::shared_ptr<octomap::ColorOcTree> m_Map;

	// User Defined Parameters for Observation Model
	double m_DownsampleVoxelSize;
	int m_MaximumIterations;
	double m_TransformationEpsilon;
	int m_KeypointType;
	int m_DescriptorType;
};

#endif /* SRC_RGBOBSERVATIONMODEL_H_ */
