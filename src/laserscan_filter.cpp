#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>



// Convert from map index to world coords
#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)

// Convert from world coords to map coords
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)


typedef struct
{
  int occ_state;
  double occ_dist;
} MapCell;

typedef struct
{
  double origin_x, origin_y;
  double scale;
  int size_x, size_y;
  MapCell *cells;
  double max_occ_dist;
} Map;

void mapFree(Map *map)
{
  free(map->cells);
  free(map);
  return;
}

Map *mapAlloc(void)
{
  Map *map;

  map = (Map*) malloc(sizeof(Map));

  map->origin_x = 0;
  map->origin_y = 0;

  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;

  map->cells = (MapCell*) NULL;

  return map;
}

Map *map;

ros::Subscriber map_sub;
nav_msgs::OccupancyGrid g_map_rosmsg;
bool g_map_received = false;
ros::Publisher g_pub_laserscan;

void mapCallback(const nav_msgs::OccupancyGridPtr & map_msg)
{
  g_map_rosmsg = *map_msg;

  map = mapAlloc();
  ROS_ASSERT(map);
  map->size_x = map_msg->info.width;
  map->size_y = map_msg->info.height;
  map->scale = map_msg->info.resolution;
  map->origin_x = map_msg->info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg->info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (MapCell*)malloc(sizeof(MapCell)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    map->cells[i].occ_dist = map_msg->data[i];
    //if(map_msg->data[i] == 0)
      //map->cells[i].occ_state = -1;
    //else if(map_msg->data[i] == 100)
      //map->cells[i].occ_state = +1;
    //else
      //map->cells[i].occ_state = 0;
  }

  map_sub.shutdown();
  g_map_received = true;
  ROS_WARN("got map debug, shutting down subscriber");
}

void scanCallback(const sensor_msgs::LaserScanPtr & laser_msg)
{
  static tf::TransformListener tf_listener;
  tf::StampedTransform transform, laser_transform;
  try
  {
    tf_listener.lookupTransform("map","laser_qmcl",ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }

  if (!g_map_received) 
  {
    ROS_WARN("laser scan but no map!");
    return;
  }

  sensor_msgs::LaserScan filtered_scan_msg(*laser_msg);
  //filtered_scan_msg.header = laser_msg->header;
  //filtered_scan_msg.header.stamp = ros::Time::now();
  //filtered_scan_msg.angle_min = laser_msg->angle_min;
  //filtered_scan_msg.angle_max = laser_msg->angle_max;
  //filtered_scan_msg.angle_increment = laser_msg->angle_increment;
  //filtered_scan_msg.time_increment = laser_msg->time_increment;
  //filtered_scan_msg.scan_time = laser_msg->scan_time;
  //filtered_scan_msg.range_min = laser_msg->range_min;
  //filtered_scan_msg.range_max= laser_msg->range_max;
  //filtered_scan_msg.set_ranges_size(laser_msg->ranges.size());
  //filtered_scan_msg.set_intensities_size(laser_msg->ranges.size());
  //filtered_scan_msg.ranges.resize(laser_msg->ranges.size());
  //filtered_scan_msg.intensities.resize(laser_msg->ranges.size());
  //for (unsigned int j=0; j<laser_msg->ranges.size() ; j ++)
  //{
    //filtered_scan_msg.ranges[j]= laser_msg->ranges[j];
    ////filtered_scan_msg.ranges.push_back( laser_msg->ranges[j]);
  //}

  double lx = transform.getOrigin().x();
  double ly = transform.getOrigin().y();
  double lyaw = tf::getYaw(transform.getRotation());
  for (unsigned int j=0; j<laser_msg->ranges.size() ; j ++)
  {				
    double ox = lx+laser_msg->ranges[j]*cos(lyaw + laser_msg->angle_min + j*laser_msg->angle_increment);
    double oy = ly+laser_msg->ranges[j]*sin(lyaw + laser_msg->angle_min + j*laser_msg->angle_increment);
    int mi = MAP_GXWX(map, ox);
    int mj = MAP_GYWY(map, oy);
    if (!MAP_VALID(map,mi,mj))
    {
      filtered_scan_msg.intensities[j] = 0;
      //filtered_scan_msg.intensities.push_back( 0);
    }
    else
    {
      filtered_scan_msg.intensities[j] = double(map->cells[MAP_INDEX(map,mi,mj)].occ_dist);
      //filtered_scan_msg.intensities.push_back( map->cells[MAP_INDEX(map,mi,mj)].occ_dist);
    }
  }
  g_pub_laserscan.publish(filtered_scan_msg);

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "laserscan_filter");
  ros::NodeHandle nh;//("~");
  map_sub = nh.subscribe("/map_debug", 1, mapCallback);
  ros::Subscriber laser_sub = nh.subscribe("/scan_qmcl", 2, scanCallback);
  g_pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/scan_filtered", 3);
  ros::spin();

  if (NULL != map)
    mapFree(map);
}
