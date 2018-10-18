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
typedef pcl::PointCloud< pcl::PointXYZ > PointCloud;

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
    void setMap(std::shared_ptr<octomap::ColorOcTree> map);

    /**
     * Set the transfrom between "base"->"target_sensor" TF frame
     * @param baseToSensor Transform between base and sensor frame;
     *
     */
    void setBaseToSensorTransform(tf2::Transform baseToSensor);

    /**
     * Set the measuremenets received from sensors along with their respective range from
     * point of origin
     * @param observed The measurements received from sensors in PoinctCloud (PCL Library)
     * @param ranges The ranges of each Point in PointCloud from the source of origin
     */
    void setObservedMeasurements(PointCloud observed,std::vector<float> ranges);

    void setTrueCarState(const RobotState& state);

  protected:

  private:

    RobotState m_TrueCarState;

    tf2::Transform m_BaseToSensorTransform;

    std::shared_ptr<octomap::ColorOcTree> m_Map;

    PointCloud m_observedMeasurement;
    std::vector<float> m_observedRanges;

    double m_ZHit;
    double m_ZShort;
    double m_ZRand;
    double m_ZMax;
    double m_SigmaHit;
    double m_LambdaShort;

};

#endif
