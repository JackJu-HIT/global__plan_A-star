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
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <boost/chrono.hpp>
#include <chrono>
#include <thread>
//#include  <boost/config.hpp>
//#include  <boost/shared_ptr.hpp>
//#include  <boost/shared_ptr.hpp>
//using  boost::make_shared;
#include <fstream>

#include <ctime>

using namespace std;
double RobotPose_x;
double RobotPose_y;
double RobotPose_theta;
namespace costmap_2d
{


Costmap2DROS::Costmap2DROS(std::string name) :
    layered_costmap_(NULL), name_(name),stop_updates_(false), initialized_(true), stopped_(false),
    robot_stopped_(false), map_update_thread_(NULL)// last_publish_(0)
{

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
 
  track_unknown_space = false;  //参数描述：用于设置是否将未知空间视为空闲空间。如果为false，则会将未知空间视为空闲空间；否则视为未知空间。
  always_send_full_costmap = false;
  std::cout << "costmap2DROS Init..."<<std::endl;
  double incrible_radius_;
  double inflation_radius_;
  if(name == "global_costmap") 
  {
      rolling_window = false;
      incrible_radius_  = 0.5;
      inflation_radius_ = 0.8;
      
      layered_costmap_ = new LayeredCostmap(rolling_window, track_unknown_space);
      
      //初始化静态地图层           
      //costmap_2d::StaticLayer  boost::make_shared<costmap_2d::StaticLayer>();
      //boost::shared_ptr<Layer> pluginStatic = boost::make_shared<costmap_2d::StaticLayer>();//     new costmap_2d::StaticLayer();
      std::cout << "costmap2DROS Init2..."<<std::endl;
     
      boost::shared_ptr<Layer> pluginStatic(new costmap_2d::StaticLayer());//boost::make_shared<costmap_2d::StaticLayer>();
      //boost::shared_ptr<costmap_2d::StaticLayer> pluginStatic = boost::make_shared<costmap_2d::StaticLayer>();
      std::cout << "costmap2DROS Init2.5.."<<std::endl;
      layered_costmap_->addPlugin(pluginStatic);
      //std::cout <<  "costmap2DROS Init2.6.." << std::endl;
      
      // oninitialize方法（Layer类中的虚函数，被定义在各层地图中）
      pluginStatic->initialize(layered_costmap_, name + "/" + "static_layer");
      std::cout << "costmap2DROS Init3.."<<std::endl;
     
      //初始化膨胀层
     boost::shared_ptr<Layer> pluginInflation(new costmap_2d::InflationLayer(inflation_radius_));//=  boost::make_shared<costmap_2d::InflationLayer>();// new InflationLayer();
     layered_costmap_->addPlugin(pluginInflation);
     pluginInflation->initialize(layered_costmap_, name + "/" + "inflation_layer");
      std::cout << pluginStatic->getName()<< std::endl;
      //todo:RObot footprint
      geometry_msgs::Polygon footprint;
      
      geometry_msgs::Point32 pointvalue;
      
      pointvalue.x = -incrible_radius_;
      pointvalue.y = incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);
      
      pointvalue.x = incrible_radius_;
      pointvalue.y = incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);

      pointvalue.x = incrible_radius_;
      pointvalue.y =-incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);

      pointvalue.x = -incrible_radius_;
      pointvalue.y = -incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);
      Costmap2DROS::setUnpaddedRobotFootprintPolygon(footprint);
  } 

  if(name == "local_costmap"){
      rolling_window = true;
      incrible_radius_ = 0.65;
      inflation_radius_ = 0.8;
      layered_costmap_ = new LayeredCostmap(rolling_window, track_unknown_space);
      

      //初始化障碍物层
      boost::shared_ptr<Layer> pluginObstacle(new costmap_2d::ObstacleLayer()); //boost::make_shared<costmap_2d::ObstacleLayer>(); //new ObstacleLayer();
      layered_costmap_->addPlugin(pluginObstacle);
      pluginObstacle->initialize(layered_costmap_, name + "/" + "obstacle_layer");
      std::cout << pluginObstacle->getName()<< std::endl;
      //初始化膨胀层
     
      boost::shared_ptr<Layer> pluginInflation(new costmap_2d::InflationLayer(inflation_radius_));//=  boost::make_shared<costmap_2d::InflationLayer>();// new InflationLayer();
      layered_costmap_->addPlugin(pluginInflation);
      pluginInflation->initialize(layered_costmap_, name + "/" + "inflation_layer");

      
      //double 
       //todo:RObot footprint
      geometry_msgs::Polygon footprint;
      
      geometry_msgs::Point32 pointvalue;
      
      pointvalue.x = -incrible_radius_;
      pointvalue.y = incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);
      
      pointvalue.x =incrible_radius_;
      pointvalue.y =incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);

      pointvalue.x = incrible_radius_;
      pointvalue.y = -incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);

      pointvalue.x = -incrible_radius_;
      pointvalue.y = -incrible_radius_;
      pointvalue.z = 0;
      footprint.points.push_back(pointvalue);
      Costmap2DROS::setUnpaddedRobotFootprintPolygon(footprint);
  }

  

  

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  robot_stopped_ = false;

  //debug 
  Costmap2DROS::reconfigureCB();
 

}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }

  delete layered_costmap_;
}

void Costmap2DROS::reconfigureCB()
{

  if (map_update_thread_ != NULL)
  { 
    std::cout<<"map_update_thread_ not null"<<std::endl;
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = 5;//config.update_frequency;

  //toDo:
  //find size parameters  
  double map_width_meters = 10, map_height_meters = 10, resolution = 0.05, origin_x =
            0,
         origin_y = 0;
  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }
  
  //std::cout<<"test"<<std::endl;
  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
 
  //std::cout<<"test3"<<std::endl;

}


void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  //mapUpdateLoop这个成员函数是用于不停刷新代价地图的，它在每个循环通过updateMap函数实现
  //，而Costmap2DROS::updateMap函数是利用LayeredCostmap的updateMap函数进行更新，在前面已经分析过。
 // if (frequency == 0.0)
 //   return;

  //int i = 0;
  //std::cout<<"map_update_thread_shutdown_"<<map_update_thread_shutdown_<<std::endl;
  //if(name_ != "global_costmap")
    
   
    std::ofstream file;
    file.open("/userdata/CarryBoy/log_costmap_time.txt",std::ios::app);
     
  while (!map_update_thread_shutdown_)
  { 
            
    struct timeval start, end;
    double start_t, end_t, t_diff;
    //gettimeofday(&start, NULL);
    //std::cout<<i<<std::endl;

    updateMap();


    gettimeofday(&end, NULL);
   

    if (layered_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
    }
  
    auto costmape = layered_costmap_->getCostmap();
    if(costmape->getCharMap() != nullptr  && (name_ == "global_costmap")){
         map_update_thread_shutdown_ = true;
    }
    
   //std::this_thread::sleep_for(std::chrono::microseconds(100)); //   seconds(1));//delay 1 s
    boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
   
  }
  file.close();
}

void Costmap2DROS::updateMap()
{
  //std::cout<<"stop_updates_:"<<stop_updates_<<std::endl;
  if (!stop_updates_)
  {
     //todo:
     
     double robot_x = RobotPose_x;
     double robot_y = RobotPose_y;
     double robot_yaw = RobotPose_theta;
     double x = robot_x, y = robot_y, yaw = robot_yaw;
    // std::cout << "Please input robot pose:robot_x,robot_y" << std::endl;
    // std::cin >> robot_x;
    // std::cin >> robot_y;
     //std::cout << "Costmap2DROS::updateMap()"<<std::endl;
      //调用layered_costmap_的updateMap函数，参数是机器人位姿
     layered_costmap_->updateMap(x, y, yaw);
    // std::cout << "stopju" << std::endl;
     geometry_msgs::PolygonStamped footprint;
      //footprint.header.frame_id = global_frame_;
      //footprint.header.stamp = ros::Time::now();
      //transformFootprint(x, y, yaw, padded_footprint_, footprint);//make coordiates to global coordiates
      //footprint_pub_.publish(footprint);
      transformFootprint(x, y, yaw, padded_footprint_, footprint);//make coordiates to global coordiates
      initialized_ = true;
      
   
  }
}

void Costmap2DROS::start()
{
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  /*
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
    */
}


void Costmap2DROS::resetLayers()
{
  Costmap2D* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

}  // namespace costmap_2d
