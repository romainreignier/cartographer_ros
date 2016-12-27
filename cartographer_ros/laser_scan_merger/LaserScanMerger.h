#pragma once

#include <laser_geometry/laser_geometry.h>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LaserScanMerger
{
    typedef message_filters::Subscriber<sensor_msgs::LaserScan> tLaserSub;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::LaserScan, sensor_msgs::LaserScan>
        tApproxTimeLaser;
    typedef message_filters::Synchronizer<tApproxTimeLaser> tSynchronizer;
    typedef std::unique_ptr<tSynchronizer> tSynchronizerPtr;

public:
    LaserScanMerger();
    void scanCallback(const sensor_msgs::LaserScanConstPtr& _scan1,
                      const sensor_msgs::LaserScanConstPtr& _scan2);
    void laserScanToPointCloud(const sensor_msgs::LaserScanConstPtr& _scan,
                               sensor_msgs::PointCloud2& _cloud);

protected:
    ros::NodeHandle m_nh;
    laser_geometry::LaserProjection m_projector;
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;
    ros::Publisher m_pointCloudPublisher;
    tSynchronizerPtr m_synchronizer;
    tLaserSub m_scan1Subscriber;
    tLaserSub m_scan2Subscriber;
    std::string m_outputFrame;
};
