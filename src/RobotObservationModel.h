#ifndef CAROBSERVATIONMODEL_H
#define CAROBSERVATIONMODEL_H

#include <libPF/ObservationModel.h>
#include <MapModel.h>
#include <RobotState.h>
#include <tf2/LinearMath/Transform.h>

#ifndef SQRT_2_PI
#define SQRT_2_PI 2.506628274
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointRGBT;
typedef pcl::PointCloud< PointT > PointCloud;
typedef pcl::PointCloud< PointRGBT > PointCloudRGB;

/** 
 * @class RobotObservationModel
 *
 * @brief Observation model that measures the position of a robot.
 *
 */
class RobotObservationModel : public libPF::ObservationModel<RobotState> {
    
  public:

    /** 
     * Constructor
     */
    RobotObservationModel(ros::NodeHandle *nh,std::shared_ptr<MapModel> mapModel);

    /**
     * Deconstructor
     */
    ~RobotObservationModel();

    /**
     *
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
     * Set the transfrom between "base"->"target_sensor" TF frame
     * @param baseToSensor Transform between base and sensor frame;
     *
     */
    void setBaseToSensorTransform(const tf2::Transform &baseToSensor);

    /**
     * Set the measuremenets received from sensors along with their respective range from
     * point of origin
     * @param observed The measurements received from sensors in PoinctCloud (PCL Library)
     * @param ranges The ranges of each Point in PointCloud from the source of origin
     */
    void setObservedMeasurements(const PointCloud &observed,const std::vector<float> &ranges);

    /**
     * Set the measuremenets received from sensors along with their respective range from
     * point of origin
     * @param observed The measurements received from sensors in PoinctCloud (PCL Library)
     * @param ranges The ranges of each Point in PointCloud from the source of origin
     */
    void setObservedMeasurements(const PointCloudRGB &observed,const std::vector<float> &ranges);

    /**
     * Set if the measurements involve RGB points
     */
    void setRGB(const bool &rgb);

    void setTrueCarState(const RobotState& state);

  protected:

  private:

    RobotState m_TrueCarState;

    tf2::Transform m_BaseToSensorTransform;

    std::shared_ptr<octomap::ColorOcTree> m_Map;

    PointCloud m_observedMeasurement;
	PointCloudRGB m_observedMeasurementRGB;
    std::vector<float> m_observedRanges;

    bool m_RGB;
    double m_ZHit;
    double m_ZShort;
    double m_ZRand;
    double m_ZMax;
    double m_SigmaHit;
    double m_LambdaShort;
};

#endif
