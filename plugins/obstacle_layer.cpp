/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/obstacle_layer.h>

#include <costmap_2d/costmap_math.h>

#include <iostream>

#include <fstream>

#include <ctime>
//#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

//double RobotPose_x_;
//double RobotPose_y_;
//double RobotPose_theta_;
//using costmap_2d::ObservationBuffer;
//using costmap_2d::Observation;

namespace costmap_2d
{

void ObstacleLayer::onInitialize()
{
 //std::cout << "ObstacleLayer::onInitialize() " << std::endl;
  //确定rolling_window_、
  //track_unknown_space的值，并调用matchSize函数，根据主地图的参数来设置障碍层地图。
  //ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space =  layered_costmap_->isTrackingUnknown();
  //std::cout << "track_unknown_space =" << std::endl;
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  ObstacleLayer::matchSize();
  current_ = true;

  //global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance  = 0.2;

  max_obstacle_height_ = 1.8;  //障碍物最大高度


  sq_obstacle_range_ = 5.0;//10.0;//1.5;// 3.0;//5.0;//5.0;//10.0;//1.5;////1.5;//1.2 2.5;  //传感器探测最大距离

  raytrace_range_ = 5.0;//10.0;// 3.0;     //清理前方距离
  
  footprint_clearing_enabled_ = false;//true

  combination_method_ =  1;
  //std::cout << "sensorINtil" << std::endl;
  //std::cout << "sensro:end"<< std::endl;

  //roslaser_sub_obj_ = new ROSLaserSubscriber();
  //boost::shared_ptr<ROSLaserSubscriber> roslaser_sub_obj_(new ROSLaserSubscriber());
  //otherlaser_sub_obj_ = new OtherLaserSubscriber();
 // boost::shared_ptr<OtherLaserSubscriber> otherlaser_sub_obj_(new OtherLaserSubscriber());
  //camera_sub_obj_ = new CameraSubscriber();
 // boost::shared_ptr<CameraSubscriber> camera_sub_obj_(new CameraSubscriber());

  //RCToPP_sub_obj_ = new RCToPPSubscriber_();
 // boost::shared_ptr<RCToPPSubscriber_> RCToPP_sub_obj_(new RCToPPSubscriber_());
  //boost::thread *obs_thread = new boost::thread(&navigation::subscrber,this);
  //todo:待输入点云信息
 // point_xy a = {0.05,0.05,0};//world cooridates!
 // pointCloud_.push_back(a);

}



ObstacleLayer::~ObstacleLayer()
{
  /*
    if (dsrv_)
        delete dsrv_;*/
    //if(roslaser_sub_obj_)
   // delete sensor_sub_obj_;
  
}

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{

  
  
  point_xy sensor_coords; 
  //sensor_sub_obj_->getNavPose(sensor_coords);
  originx_Sensor_.x = 0;
  originx_Sensor_.y = 0;
  originx_Sensor_.z = 0;
  originx_Sensor_.theta = 0;;

  //lock.unlock();
  
  if (rolling_window_)  //rolling_window_ = 1
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
 // if (!enabled_)
 //  return;
  useExtraBounds(min_x, min_y, max_x, max_y);
  //在接收到传感器数据后，buffer将被更新，同样，marking_buffers_和
  //clearing_buffers_也更新（即buffer中的内容），将二者分别提取到observations和clearing_observations中存储。
  //bool current = true;
  //double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;
  //std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  //current = current && getMarkingObservations(observations);

  // get the clearing observations
  //current = current && getClearingObservations(clearing_observations);

  // update the global current status
  //current_ = current;
  pointCloud_.clear();

  
  std::vector<point_xy> m_rosLaser_temp;
  std::vector<point_xy> m_otherLaser_temp;
  std::vector<point_xy> m_cameraPoint1_temp;
  std::vector<point_xy> m_cameraPoint2_temp;
  std::vector<point_xy> m_cameraPoint3_temp;
  
  raytraceFreespace(pointCloud_, min_x, min_y, max_x, max_y);
  // raytrace freespace
  ///std::cout << "raytrace" << std::endl;
  /*
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }*/
/***第二步是marking操作，即将点云中的点标记为障碍物。

在标记时通过二重循环，外层迭代各观测轮次，内层迭代一次观测得到的点云点，
剔除本身太高（z坐标过大）、与传感器距离太远的点，将符合要求的障碍点坐标从global系转换到map系，并在本层地图上标记致命障碍。

并调用touch函数，确保标记的障碍点包含在 bound内。

作者：wanghuohuo0716
链接：https://www.jianshu.com/p/2984cdc99236
来源：简书
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。*/
  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  /*
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)
      {
        ROS_DEBUG("The point is too high");
        continue;
      }
       //计算点与点之间的距离平方
      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it  
      if (sq_dist >= sq_obstacle_range)
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }
      //设置它为致命
      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  */
  
  
  //file_obs.close();
  for(int i = 0;i < pointCloud_.size(); i++){

      double px = pointCloud_[i].x, py = pointCloud_[i].y, pz = pointCloud_[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)
      {
        std::cout<<"The point is too high"<<std::endl;
          // 获取当前系统时间的秒数
        std::time_t now = std::time(0);
        // 将秒数转换成 std::tm 结构体类型的时间
        std::tm *localTime = std::localtime(&now);
        // 格式化时间输出
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localTime);    
        std::ofstream file;
        file.open("/userdata/CarryBoy/log_obstacle_updateBounds.txt",std::ios::app);
        if(file.is_open()){
              file <<buffer<<" " <<"The point is too high"<<std::endl;
        }
        file.close();
        continue;
      }
       //计算点与点之间的距离平方
      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - originx_Sensor_.x) * (px -  originx_Sensor_.x) + (py -  originx_Sensor_.y) * (py -  originx_Sensor_.y)
          + (pz -  originx_Sensor_.z) * (pz -  originx_Sensor_.z);
      sq_dist = sqrt(sq_dist);
      // if the point is far enough away... we won't consider it  
      if ((sq_dist >= sq_obstacle_range_) || sq_dist < 0.4)
      {
        continue;
      }
    
      {// 获取当前系统时间的秒数
        std::time_t now = std::time(0);
        // 将秒数转换成 std::tm 结构体类型的时间
        std::tm *localTime = std::localtime(&now);
        // 格式化时间输出
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localTime);   
        //file_obs << px <<" " << py << std::endl; 
      }
    
      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      
      if (!worldToMap(px, py, mx, my))
      {
        std::cout<<"Computing map coords failed"<<std::endl;
        continue;
      }
      //设置它为致命
      unsigned int index = getIndex(mx, my);
    //  std::cout << "getindx" << index << std::endl;
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
   }
  //在以上两步完成后，调用updateFootprint函数，
  //它的作用是基于机器人当前位置确定该位置下的足迹，并在内部调用touch函数保证足迹包含在bound范围内。
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  //pointCloud_.clear();
}

void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  //这个函数就是将机器人足迹范围内设置为FREE_SPACE，并且在bound范围内将本层障碍地图的内容合并到主地图上。
 /* if (!enabled_)
    return;
*/
  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

/*
void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}
*/
/*
void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}
*/
/*
bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}
*/
/*
bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}
*/
void ObstacleLayer::raytraceFreespace(const std::vector<point_xy>& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{
  ////clearing_observation的origin_是传感器坐标，传入
  double ox = originx_Sensor_.x;
  double oy = originx_Sensor_.y;
  //pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);
  //得到传感器原点在地图上的坐标
  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0))
  {
        std::cout<<"The origin for the sensor at ("<<ox<<","<<oy<<") is out of map bounds. So, the costmap cannot raytrace for it."
        << std::endl;
        return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  //我们可以在内环之外预先计算地图的点。。。我们稍后需要这些 
  // Remain Confirm 
  double origin_x  = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  touch(ox, oy, min_x, min_y, max_x, max_y);
  

  //接下来的几个判断结构实际上做的主要工作就是不断迭代传感器原点和点云中的点的连线，
  //并对其调用raytraceLine函数，将连线上的点在本层地图上全部标记为FREE_SPACE。
  //这里获取连线时，要注意该点是否超出地图范围，如果超出，则通过相似三角形去除连线在地图外的部分。


  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  //对于点云中的每个点，我们要追踪原点和clear obstacles之间的一条线
  for (unsigned int i = 0; i < clearing_observation.size(); ++i)
  {
    //wx wy是当前点云中的点的坐标
    double wx = clearing_observation[i].x;// cloud.points[i].x;
    double wy = clearing_observation[i].y;//   cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    //a、b是该点跟传感器原点的距离
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    //如果当前点x方向比地图原点还小
    if (wx < origin_x)
    {
      //t（比例）=（地图原点-传感器原点）/（点云中的该点-传感器原点）
      double t = (origin_x - ox) / a;
      wx = origin_x;
      //当前点y = 
      //实际上还是把点云点和传感器连线之间清空，只是通过相似三角形丢弃了超出地图原点范围外的部分，下面三个判断结构同理
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x)
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = cellDistance(raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    updateRaytraceBounds(ox, oy, wx, wy, raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::activate()
{ /*
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
  */
}
void ObstacleLayer::deactivate()
{
  /*
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
  */
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset()
{
  /*
    deactivate();
    resetMaps();
    current_ = true;
    activate();
    */
}



}  // namespace costmap_2d
