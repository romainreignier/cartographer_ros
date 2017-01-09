#include "LaserScanMerger.h"

#include "cartographer/common/make_unique.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

using cartographer::common::make_unique;

LaserScanMerger::LaserScanMerger()
    : m_nh("~"), m_tfListener(m_tfBuffer), m_outputFrame("base_link")
{
    if(!m_nh.getParam("output_frame", m_outputFrame))
    {
        ROS_WARN_STREAM(
            "Rosparam 'output_frame' not found. Default: " << m_outputFrame);
    }

    m_scan1Subscriber.subscribe(m_nh, "scan1", 1);
    m_scan2Subscriber.subscribe(m_nh, "scan2", 1);

    m_synchronizer = make_unique<tSynchronizer>(
        tApproxTimeLaser(2), m_scan1Subscriber, m_scan2Subscriber);
    m_synchronizer->registerCallback(
        boost::bind(&LaserScanMerger::scanCallback, this, _1, _2));

    m_pointCloudPublisher =
        m_nh.advertise<sensor_msgs::PointCloud2>("merged_cloud", 1);
}

void LaserScanMerger::scanCallback(const sensor_msgs::LaserScanConstPtr& _scan1,
                                   const sensor_msgs::LaserScanConstPtr& _scan2)
{
    sensor_msgs::PointCloud2 cloudScan1;
    laserScanToPointCloud(_scan1, cloudScan1);
    sensor_msgs::PointCloud2 cloudScan2;
    laserScanToPointCloud(_scan2, cloudScan2);
    sensor_msgs::PointCloud2 mergedCloud;
    pcl::concatenatePointCloud(cloudScan1, cloudScan2, mergedCloud);
    mergedCloud.header.stamp = (_scan1->header.stamp > _scan2->header.stamp) ? _scan1->header.stamp : _scan2->header.stamp;
    m_pointCloudPublisher.publish(mergedCloud);
}

void LaserScanMerger::laserScanToPointCloud(
    const sensor_msgs::LaserScanConstPtr& _scan,
    sensor_msgs::PointCloud2& _cloud)
{
    std::string errorMsg;
    if(m_tfBuffer.canTransform(m_outputFrame,
                               _scan->header.frame_id,
                               _scan->header.stamp +
                                   ros::Duration((double)_scan->scan_time),
                               ros::Duration(0.05),
                               &errorMsg))
    {
        try
        {
            m_projector.transformLaserScanToPointCloud(
                m_outputFrame,
                *_scan,
                _cloud,
                m_tfBuffer,
                laser_geometry::channel_option::Distance);
        }
        catch(const tf2::TransformException& e)
        {
            ROS_ERROR_STREAM("tf: " << e.what());
        }
    }
    else
    {
        ROS_ERROR_STREAM("tf: " << errorMsg);
    }
}

