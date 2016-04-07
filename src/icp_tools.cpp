#include "icp_test/icp_tools.h"

void transferPclPointCloudXYZToXYPointsMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pc,  CSimplePointsMap*  point_map)
{
    std::vector<float> xs, ys;

    for (size_t next = 0; next < input_pc->points.size(); ++next)
    {
      pcl::PointXYZ _point = input_pc->points.at(next);
      xs.push_back(_point.x);
      ys.push_back(_point.y);
    }
    point_map->setAllPoints(xs, ys);
}

void transferPclPointCloudToXYPointsMap(VPointCloud::Ptr &input_pc,  CSimplePointsMap*  point_map)
{
    std::vector<float> xs, ys;

    for (size_t next = 0; next < input_pc->points.size(); ++next)
    {
      velodyne_pointcloud::PointXYZIR _point = input_pc->points.at(next);
      xs.push_back(_point.x);
      ys.push_back(_point.y);
    }
    point_map->setAllPoints(xs, ys);

}

