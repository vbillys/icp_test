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


void transferPointsMapToPointCloud(CSimplePointsMap*  point_map, VPointCloud::Ptr output_pc )
{
  std::vector<float> xs, ys;
  point_map->getAllPoints(xs, ys);
  int idx = 0;
  for (std::vector<float>::iterator it = xs.begin(); it< xs.end(); it++, idx++)
  {
    VPoint _point;
    _point.x  = *it; _point.y = ys[idx]; _point.z = 0;
    output_pc->push_back(_point);

  }


}

namespace ICPTools
{

  Point2D transformPoint2D(Point2D point, Pose2D pose)
  {
    Point2D result;
    result.x = point.x * cos(pose.yaw) - point.y * sin(pose.yaw);
    result.y = point.x * sin(pose.yaw) + point.y * cos(pose.yaw);
    result.x += pose.x; result.y += pose.y;
    return result;
  }
  tf::Transform calcTransformFromPose2D(Pose2D pose)
  {
    tf::Transform transform_var;
    transform_var.setOrigin(tf::Vector3(pose.x, pose.y, 0) );
    tf::Quaternion quat;
    quat.setRPY(0 ,0 ,pose.yaw);
    transform_var.setRotation(quat);
    return transform_var;
  }
  void zeroPose2D(Pose2D & pose)
  {
    pose.x = 0; pose.y = 0; pose.yaw = 0;
  }
  float calcVectorLength(Point2D point)
  {
    return sqrt(point.x*point.x + point.y*point.y);
  }
}
