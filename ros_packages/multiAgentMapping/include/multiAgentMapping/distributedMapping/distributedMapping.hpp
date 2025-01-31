#ifndef _DISTRIBUTED_MAPPPING_H_
#define _DISTRIBUTED_MAPPING_H_

#include <rclcpp/rclcpp.hpp>

#include <multiAgentMapping/distributedMapping/subGraphParams.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

using namespace gtsam;
using namespace std;

class distributedMapping : public subGraphParams {
    public:
        distributedMapping();

        bool saveFrame(const Pose3& psoe_to);
    
    private:
        pcl::PointCloud<PointPose3D>::Ptr keyposes_cloud_3d; // 3-dof keyposes in local frame
		pcl::PointCloud<PointPose6D>::Ptr keyposes_cloud_6d; // 6-dof keyposes in local frame
};

#endif //_DISTRIBUTED_MAPPING_H_