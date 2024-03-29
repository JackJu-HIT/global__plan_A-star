/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <../include/yaml-cpp/yaml.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

//#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{


StaticLayer::StaticLayer(){}

StaticLayer::~StaticLayer()
{
}

void StaticLayer::onInitialize()
{
  std::cout << "StaticLayer::onInitialize() :" << std::endl;
  first_map_only_ = false;
  track_unknown_space_ = true;
  use_maximum_  = true;
  subscribe_to_updates_ = true;

  int temp_lethal_threshold = 100, temp_unknown_cost_value = -1;
  trinary_costmap_ = false;
  

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //TOdo:地图需要处理
      //Read pgm
 // cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/AStar-ROS/map/map.pgm",cv::IMREAD_GRAYSCALE);
 cv::Mat m4 = cv::imread("/home/juchunyu/20231013/240308/costmap_2d_A_star/map/PM.pgm",cv::IMREAD_GRAYSCALE);///home/juchunyu/20231013/globalPlanner
  //cv::Mat m4 = cv::imread("/userdata/CarryBoy/ShareFiles/PM.pgm",cv::IMREAD_GRAYSCALE);
  //cv::imshow("map_info_Rotate11",m4);
         //boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        //	cv::imshow("map_info", map_info.t());
  //cv::waitKey(0);
  std::cout << "图像宽为：" << m4.cols << "\t高度为：" << m4.rows << "\t通道数为：" << m4.channels() << std::endl;
  
  StaticLayer::incomingMap(m4);  
}


void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  std::cout << "isrooling"<<!layered_costmap_->isRolling() << std::endl;
  if (!layered_costmap_->isRolling())
  { 
   // std::cout << "tezt"<<std::endl;
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

unsigned char StaticLayer::interpretValue(unsigned char value)
{
  //接收到的地图上：unknown_cost_value（默认为-1）为未知区域，lethal_cost_threshold（默认100）
  //以上为致命障碍物,当接收到的地图上为-1时，若追踪未知区域，则本层地图上赋值NO_INFORMATION（255）；
  //否则，在本层地图上赋值FREE_SPACE（0）；当接收到的地图上>=100时，在本层地图上赋值LETHAL_OBSTACLE（254）；
  //若以上都不是，则按比例返回代价值。
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_){
   
    return NO_INFORMATION;
  }
  else if (!track_unknown_space_ && value == unknown_cost_value_){
    return FREE_SPACE;}  //FREE_SPACE = 0;
  else if (value >= lethal_threshold_){
   return LETHAL_OBSTACLE;} //LETHAL_OBSTACLE = 254;
  else if (trinary_costmap_){
      return FREE_SPACE;
  }
  
  double scale = (double) value / lethal_threshold_;
  
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::incomingMap(cv::Mat map_info)
{
  //cv::imshow("map_info", map_info);
  // 读取map.yaml文件
 // YAML::Node node = YAML::LoadFile("/home/juchunyu/20231013/Navigation_system/map/map/PM.yaml");

    std::string filename = "/home/juchunyu/20231013/240308/costmap_2d_A_star/map/PM.yaml";
    PMHeader header = readPMHeader(filename);
    std::cout << "yamp-test" << std::endl;
    std::cout << "image: " << header.image << std::endl;
    std::cout << "resolution: " << header.resolution << std::endl;
    std::cout << "origin: [";

    for (int i = 0; i < header.origin.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << header.origin[i];
    }

    std::cout << "]" << std::endl;
    std::cout << "occupied_thresh: " << header.occupied_thresh << std::endl;
    std::cout << "free_thresh: " << header.free_thresh << std::endl;
    std::cout << "negate: " << header.negate << std::endl;
  // 获取resolution字段的值
  //double resolution_ = node["resolution"].as<double>();

  // 获取origin字段的值
 // std::vector<double> origin = node["origin"].as<std::vector<double>>();

  // 获取negate字段的值
  //int negate = node["negate"].as<int>();

  // 获取occupied_thresh字段的值
  //double occupied_thresh = node["occupied_thresh"].as<double>();

  // 获取free_thresh字段的值
  //double free_thresh = node["free_thresh"].as<double>();

  // 打印读取的字段值
 // std::cout << "resolution: " << resolution_ << std::endl;
  //std::cout << "origin: [" << origin[0] << ", " << origin[1] << ", " << origin[2] << "]" << std::endl;
  //std::cout << "negate: " << negate << std::endl;
  //std::cout << "occupied_thresh: " << occupied_thresh << std::endl;
  //std::cout << "free_thresh: " << free_thresh << std::endl;
 // double resolution_ = 0.05;

  //获取接收到的静态地图的尺寸，当地图不随机器人移动时，
  //若接收到的静态地图和主地图的尺寸/分辨率/起点不同，以接收到的地图为准，调整主地图的参数。
  unsigned int size_x = map_info.cols;// = new_map.size();//todonew_map->info.width, 
  unsigned int size_y = map_info.rows;//= new_map.size();//todo:

  double origin_x = header.origin[0];//origin[0];//  交叉编译临时写死
  double origin_y = header.origin[1];//origin[1];
  double resolution_ = header.resolution;
  
  //ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  //如果master map的尺寸、分辨率或原点与获取到的地图不匹配，重新设置master map
  Costmap2D* master = layered_costmap_->getCostmap();
  //ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
  layered_costmap_->resizeMap(size_x, size_y, resolution_,origin_x,
                             origin_y, true);
  //std::cout << "resizemap"<<std::endl; 

  if(costmap_ == nullptr)
  {
    std::cout << "static::costmap_ is null!"<<std::endl;
    return;
  }
                         
  /*
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    //ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  { //若本层的数据和接收到的静态地图的参数不同时，继续以接收到的地图为准，调整本层参数。
    // only update the size of the costmap stored locally in this layer
    //ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }
  */
  unsigned int index = 0;
  //将接收到的静态地图数据复制到本层地图，复制过程中调用interpretValue函数，进行“翻译”，内容后述。
  //并设置好地图坐标系中的原点x_、y_，以及地图尺寸和标志位。
  //若first_map_only_开启，则在第一次接收到地图后，话题将不再接收消息。

  // initialize the costmap with static data
  //unsigned char value[9] = {0,0,0,34,2,5,0,0,0};
  # if 0
  int k = 0;
  for (unsigned int i = size_y - 1; i > 0; i--)
  {
    for (unsigned int j = 0; j < size_x; j++)
    {
      //todo:
      //unsigned char value = 255;
      ///unsigned char value = 50;  //待处理new_map->data[index]
      //std::cout << "Please input number:" << std::endl;
      //std::cin >> value;
      //std::cout << "StaticLayer::incomingMap24...index = "<< index<<std::endl;
      //std::cout << "1"<<std::endl
      // std::cout << "map_info[" << index << "] = " << static_cast<int>(map_info.at<unsigned char>(i, j)) << std::endl;
      costmap_[index] = interpretValue(( 254 - map_info.at<unsigned char>(i, j)));
      // std::cout << "255 - map_info[" << index << "] = " << static_cast<int>(254 - map_info.at<unsigned char>(i, j)) << std::endl;
      ++index;
    }
  }
  #endif
  int k = 0;
  for (unsigned int j = size_y - 1; j > 0; j--)
  {
    for (unsigned int i = 0; i < size_x; i++)
    {
      //todo:
      //unsigned char value = 255;
      ///unsigned char value = 50;  //待处理new_map->data[index]
      //std::cout << "Please input number:" << std::endl;
      //std::cin >> value;
      //std::cout << "StaticLayer::incomingMap24...index = "<< index<<std::endl;
      //std::cout << "1"<<std::endl
      // std::cout << "map_info[" << index << "] = " << static_cast<int>(map_info.at<unsigned char>(i, j)) << std::endl;
      //index = MAP_IDX(size_x,i,size_y - j - 1);
       costmap_[index] = interpretValue((255 - map_info.at<unsigned char>(j, i)));
       costmap_[index] = interpretValue((255 - map_info.at<unsigned char>(j, i)));
      // std::cout << "255 - map_info[" << index << "] = " << static_cast<int>(254 - map_info.at<unsigned char>(i, j)) << std::endl;
      ++index;
    }
  }
  //resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = +100;
 
  #if 0
  /*
  cv::Mat map_info__(size_x, size_y, CV_8UC1);//initlize
  cv::Mat map_info_Rotate__(size_x, size_y, CV_8UC1);//initlize

        for(int i = (size_x)*(size_y) - 1; i > 0; i--)
        {
            int x = i%size_x;  //还原为像素坐标
            int y = i/size_x;  //还原为像素坐标
            //unsigned char value = 255;
            map_info__.at<unsigned char>(x, y) =  costmap_[i];
            //cout<<endl;
        }   

         //int l = 0;int k = size_y;
         for(int j = 0,k = size_y - 1; j < size_x; j++,k--){
                  for(int i = 0,l = 0; i < size_x;i++,l++){
                  map_info_Rotate__.at<unsigned char>(l,k) = map_info__.at<unsigned char>(i, j);
            }
         }
         cv::imshow("map_info__", map_info__);
         cv::rotate(map_info__, map_info_Rotate__,cv::ROTATE_90_COUNTERCLOCKWISE);// cv::ROTATE_90_CLOCKWISE);
         cv::imshow("map_info_Rotate", map_info_Rotate__);
         cv::waitKey(0);
         */
  #endif
 // std::cout << "static_layer_map:" << std::endl;
  //for(int i = 0;i < size_x * size_y;i++){
  //  std::cout << static_cast<double>(costmap_[i]) << " ";
  //}
  //std::cout<<std::endl;
  //map_frame_ = new_map->header.frame_id;

  //we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    //ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    //map_sub_.shutdown();
  }

  
}

/*void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}
*/
void StaticLayer::activate()
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  /*
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
    */
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  //若非rolling地图，在有地图数据更新时更新边界，否则，根据静态层更新的区域的边界更新传入的边界。
  
  if( !layered_costmap_->isRolling() ){
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }

  //return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  
  double wx, wy;
  //将map系中的起点（x_， y_）与终点（x_ + width_,， y_ + height_）转换到世界系，
  //确保传入的bound能包含整个map在世界系中的范围。
  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  /*
  std::cout << "max_wxv = " << wx << " " << "max_wy = " << wy << std::endl;
  std::cout << "reslution = " << resolution_ << std::endl;
  std::cout << "origin_x_ = " << origin_x_ << std::endl;
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);
  std::cout << "x_ = " << x_ << std::endl;
  std::cout << "width_" <<width_<< std::endl;
  std::cout << "minx = " << *min_x<<std::endl;
  std::cout << "min_y = " << *min_y<<std::endl;
  std::cout << "max_x = " << *max_x<<std::endl;
  std::cout << "max_y = " << *max_y << std::endl;
  std::cout<<"Enter static_layer update bounds..."<<std::endl;
  */

  //std::cout << "width_" <<width_<< std::endl;
  //std::cout << "minx = " << *min_x<<std::endl;
  //std::cout << "min_y = " << *min_y<<std::endl;
  //std::cout << "max_x = " << *max_x<<std::endl;
  //std::cout << "max_y = " << *max_y << std::endl;

  //has_updated_data_ = false;
}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::cout << "Static_updateosts:" << std::endl;
  /*
  if (!map_received_)
    return;
  */

  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    //std::cout << "!use_maximum_ = " << !use_maximum_ << std::endl;
   // if (!use_maximum_)
  updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);

  //std::cout << "end" << std::endl;
   // else
    //updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    /*
    // Might even be in a different frame
    tf::StampedTransform transform;
    try
    {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    */
     //TODO:Might even be in a different frame
    // Copy map data given proper transformations
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        //tf::Point p(wx, wy, 0);
        //p = transform(p);
        // Set master_grid with cell from map
        if (worldToMap(wx, wy, mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }

  #if 0 
  /*
  unsigned char* master_array = master_grid.getCharMap();
  int size_x = width_;
  int  size_y = height_;
  cv::Mat map_info__(size_x, size_y, CV_8UC1);//initlize
  cv::Mat map_info_Rotate__(size_x, size_y, CV_8UC1);//initlize

        for(int i = (size_x)*(size_y) - 1; i > 0; i--)
        {
            int x = i%size_x;  //还原为像素坐标
            int y = i/size_x;  //还原为像素坐标
            //unsigned char value = 255;
            map_info__.at<unsigned char>(x, y) =  master_array[i];
            //cout<<endl;
        }   

         //int l = 0;int k = size_y;
         for(int j = 0,k = size_y - 1; j < size_x; j++,k--){
                  for(int i = 0,l = 0; i < size_x;i++,l++){
                  map_info_Rotate__.at<unsigned char>(l,k) = map_info__.at<unsigned char>(i, j);
            }
         }
         cv::imshow("map_info__StaticLayer::updateCosts", map_info__);
         cv::rotate(map_info__, map_info_Rotate__,cv::ROTATE_90_COUNTERCLOCKWISE);// cv::ROTATE_90_CLOCKWISE);
         cv::imshow("map_info_Rotate_StaticLayer::updateCosts", map_info_Rotate__);
         cv::waitKey(0);
         */
  #endif



}

}  // namespace costmap_2d
