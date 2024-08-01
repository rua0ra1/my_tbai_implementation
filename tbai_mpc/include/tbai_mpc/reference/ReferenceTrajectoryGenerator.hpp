#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <ocs2_anymal_commands/ReferenceExtrapolation.h>
#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ros/ros.h>
#include <tbai_reference/ReferenceVelocityGenerator.hpp>

namespace tbai {
namespace mpc {
namespace reference {

using namespace switched_model;  // NOLINT

using ocs2::scalar_t;
using ocs2::SystemObservation;

class LocalTerrainEstimator {
   public:
    LocalTerrainEstimator();
    void updateFootholds(const ocs2::SystemObservation &observation);
    inline const TerrainPlane &getPlane() const { return terrainPlane_; }

   private:
    void updateLocalTerrainEstimate(const std::vector<vector3_t> &footholds);

    // Local terrain estimate
    TerrainPlane terrainPlane_;

    // last foorholds
    std::vector<vector3_t> lastFootholds_;

    // Kinematics model
    std::unique_ptr<KinematicsModelBase<scalar_t>> kinematicsModel_;
};

class ReferenceTrajectoryGenerator {
   public:
    ReferenceTrajectoryGenerator(const std::string &targetCommandFile, ros::NodeHandle &nh);
    void publishReferenceTrajectory();
    void reset() { firstObservationReceived_ = false; }

    bool isInitialized() const { return firstObservationReceived_; }

   private:
    BaseReferenceHorizon getBaseReferenceHorizon();
    BaseReferenceCommand getBaseReferenceCommand(scalar_t time);
    BaseReferenceState getBaseReferenceState();
    const TerrainPlane &getTerrainPlane() const;
    ocs2::TargetTrajectories generateReferenceTrajectory(scalar_t time, scalar_t dt);

    void loadSettings(const std::string &targetCommandFile);

    std::unique_ptr<tbai::reference::ReferenceVelocityGenerator> velocityGeneratorPtr_;

    LocalTerrainEstimator localTerrainEstimator_;
    ros::Publisher terrainPublisher_;

    // ROS callbacks
    ros::Subscriber observationSubscriber_;
    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg);

    ros::Subscriber terrainSubscriber_;
    void terrainCallback(const grid_map_msgs::GridMap &msg);

    ros::Publisher referencePublisher_;

    ocs2::vector_t defaultJointState_;
    TerrainPlane localTerrain_;
    bool firstObservationReceived_;
    scalar_t comHeight_;
    scalar_t trajdt_;   // timestep
    size_t trajKnots_;  // number of timesteps in reference horizon
    std::mutex observationMutex_;
    std::mutex terrainMutex_;
    std::unique_ptr<grid_map::GridMap> terrainMapPtr_;
    SystemObservation latestObservation_;

    bool blind_;
};

std::unique_ptr<ReferenceTrajectoryGenerator> getReferenceTrajectoryGeneratorUnique(ros::NodeHandle &nh);

std::shared_ptr<ReferenceTrajectoryGenerator> getReferenceTrajectoryGeneratorShared(ros::NodeHandle &nh);

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
