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
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(bool rolling_window, bool track_unknown) :
    costmap_(),rolling_window_(rolling_window), initialized_(false), size_locked_(false)
{

  if (track_unknown)
    costmap_.setDefaultValue(255);
  else
    costmap_.setDefaultValue(0);
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  size_locked_ = size_locked;
  //std::cout << "size_locked_ "<< std::endl;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();
  }
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  //这个函数在Costmap2DROS的地图更新线程中被循环调用。
  //它分为两步：第一步：更新bound，即确定地图更新的范围；第二步：更新cost，更新每层地图cell对应的cost值后整合到主地图上。
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));//lock

  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  //rolling_window_默认为false，如果开启的话，地图是时刻跟随机器人中心移动的，
  //这里需要根据机器人当前位置和地图大小计算出地图的新原点，设置给主地图。
  //std::cout <<"updateMap" <<std::endl;
  //std::cout << "rolling_window_"<<rolling_window_<<std::endl;
  //std::cout << "robot_x="<<robot_x << std::endl;
  //std::cout << "robot_y =" << robot_y << std::endl;
  //std::cout << "costmap_.getSizeInMetersX() / 2" << costmap_.getSizeInMetersX() / 2 <<std::endl;
  if (rolling_window_)
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
    //std::cout << "new_origin_x = " << new_origin_x << std::endl;
    //std::cout << "new_origin_y = " << new_origin_y << std::endl;
  }
 
  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;
  

/*接下来进行地图更新的第一步：更新bound
设置好minx_、miny_、maxx_、maxy_的初始值，然后对每一层的子地图调用其updateBounds函数，
传入minx_、miny_、maxx_、maxy_，函数将新的bound填充进去。
updateBounds函数在Layer类中声明，在各层地图中被重载，第二步使用到的updateCosts函数也是如此。
这两个函数的具体内容在各层地图部分详述。
————————————————
版权声明：本文为CSDN博主「BRAND-NEO」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/Neo11111/article/details/104844646*/
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    //plugin->
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      /*
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());*/

      std::cout<<"Illegal bounds change, was [tl:("<<prev_minx<<","<<prev_miny<<"),"<<"br:("<<
      prev_maxx<<","<<prev_maxy<<")"<<"but,is now [tl:("<<minx_<<","<<miny_<<")"<<"br: ("<<maxx_
      << ","<<maxy_<<")." << "The offending layer is "<<  (*plugin)->getName().c_str() << std::endl;
    }
  }
  //接下来调用Costmap2D类的worldToMapEnforceBounds函数，
  //将得到的bound转换到地图坐标系。这个函数可以防止转换后的坐标超出地图范围。
  int x0, xn, y0, yn;
  costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  
  costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);
 // std::cout << "maxx_ = " << maxx_ << std::endl;
 // std::cout << "getResolution = " << costmap_.getResolution() << std::endl;
 // std::cout << "x0 = " << x0 << std::endl;
 // std::cout << "y0 = " << y0 << std::endl;
//std::cout << "xn = " << xn << std::endl;
 // std::cout << "yn = " << yn << std::endl;
 // std::cout << "getSizeInCellsX = " << costmap_.getSizeInCellsX() << std::endl;
 // std::cout << "getOriginX = " << costmap_.getOriginX() << std::endl;
  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);


  //范围更新
  //ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);
  //std::cout << "Updating area x:[" << x0 <<","<< xn <<"]" << "y:["<< y0 
  //<< ","<<yn<<"]"<<std::endl;

  if (xn < x0 || yn < y0)
    return;
   
 // std::cout << "juchunyu2" << std::endl;
  //接下来，调用resetMap，将主地图上bound范围内的cell的cost恢复为默认值（track_unknown：255 / 否则：0），
  //再对每层子地图调用updateCosts函数。
  costmap_.resetMap(x0, y0, xn, yn);
 // std::cout << "juchunyu2.6" << std::endl;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
   // std::cout << "juchunyu2.5" << std::endl;
    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
   // std::cout << "juchunyu3" << std::endl;
  }
//std::cout << "juchunyu1" << std::endl;
  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
  
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);
  std::cout << "setFootprint:inscribed_radius_ = " << inscribed_radius_ << std::endl;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
