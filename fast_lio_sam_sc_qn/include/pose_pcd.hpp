#ifndef FAST_LIO_SAM_SC_QN_POSE_PCD_HPP
#define FAST_LIO_SAM_SC_QN_POSE_PCD_HPP

///// coded headers
#include "utilities.hpp"

struct PosePcd
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    double timestamp_;
    int idx_;
    bool processed_ = false;
    bool is_degenerate_ = false;    // set true when LIO covariance exceeds threshold
    bool cloud_sparsified_ = false; // set true once the cloud has been background-downsampled
    PosePcd() {}
    PosePcd(const nav_msgs::Odometry &odom_in,
            const sensor_msgs::PointCloud2 &pcd_in,
            const int &idx_in,
            bool input_in_lidar_frame = false);
};

inline PosePcd::PosePcd(const nav_msgs::Odometry &odom_in,
                        const sensor_msgs::PointCloud2 &pcd_in,
                        const int &idx_in,
                        bool input_in_lidar_frame)
{
    tf::Quaternion q(odom_in.pose.pose.orientation.x,
                     odom_in.pose.pose.orientation.y,
                     odom_in.pose.pose.orientation.z,
                     odom_in.pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat_tf(q);
    Eigen::Matrix3d rot_mat_eig;
    tf::matrixTFToEigen(rot_mat_tf, rot_mat_eig);
    pose_eig_.block<3, 3>(0, 0) = rot_mat_eig;
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;
    pcl::PointCloud<PointType> tmp_pcd;
    pcl::fromROSMsg(pcd_in, tmp_pcd);
    if (input_in_lidar_frame)
    {
        // Cloud is already in LiDAR frame — store directly.
        pcd_ = std::move(tmp_pcd);
    }
    else
    {
        // Cloud is in world (odom) frame — invert the odometry transform to get LiDAR frame.
        pcd_ = transformPcd(tmp_pcd, pose_eig_.inverse());
    }
    timestamp_ = odom_in.header.stamp.toSec();
    idx_ = idx_in;
}

#endif
