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
#include <algorithm>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
//#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{
//InflationLayer(0,)
InflationLayer::InflationLayer(double inflation_radius)
  : inflation_radius_(inflation_radius)
  , weight_(0)
  , inflate_unknown_(false)
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , seen_(NULL)
  , cached_costs_(NULL)
  , cached_distances_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  inflation_access_ = new boost::recursive_mutex();
}

void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    current_ = true;
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    need_reinflation_ = true;//https://www.cswamp.com/post/114   need_reinflation_指示下次updateBounds时，是否需要更新整张代价地图。初始值true，经过一次updateBounds后改为false。
    double inflation_radius = inflation_radius_;//0.1;//0.6;// 0.55;// //0.55;
    //inflation_radius  = 
    ///std::cout << "==============inflation_radius =" << inflation_radius << 
    
    double cost_scaling_factor = 5.5;//5.5;// 10;
    //std::cout << "InflationLayer::onInitialize() " << std::endl;
    //std::cout << "setInflationParameters" << std::endl;
    //std::cout << "updatr costsstaet" << std::endl;
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    resolution_ = costmap->getResolution();
    //resolution_ = costmap->getResolution();
    setInflationParameters(inflation_radius, cost_scaling_factor);

    //std::cout << "updatr costsend11" << std::endl;
   // dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb = boost::bind(
      //  &InflationLayer::reconfigureCB, this, _1, _2);
   /*
    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
    */
  }
 
  //std::cout << "test" << std::endl;

  matchSize();
  //std::cout << "testend" << std::endl;
  //std::cout << "inflation match size" << std::endl;
  //由于InflationLayer没有继承Costmap2D，所以它和静态地图与障碍地图这两层不同，它没有属于自己的栅格地图要维护，
  //所以matchSize函数自然不需要根据主地图的参数来调节本层地图。这个函数先获取主地图的分辨率，接着调用cellDistance函数，
  //这个函数可以把global系以米为单位的长度转换成以cell为单位的距离，故获得了地图上的膨胀参数cell_inflation_radius_。
}
/*
void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

  if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown) {
    enabled_ = config.enabled;
    inflate_unknown_ = config.inflate_unknown;
    need_reinflation_ = true;
  }
}
*/
void InflationLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
}

void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{

  ///std::cout << "enter inflation layer update bounds..." << std::endl;
  //if(costmap_ == nullptr){std::cout << "staticwuyouixao"<<std::endl;}
  if (need_reinflation_)
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void InflationLayer::onFootprintChanged()
{
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  //std::cout << ":onFootprintChanged()cell_inflation_radius_" << cell_inflation_radius_<< std::endl;
  //std::cout << "resolutin" << resolution_ << std::endl;
  //std::cout << "test" << std::endl;
  computeCaches();  //maybe problem?
  need_reinflation_ = true;

 // ROS_DEBUG("InflationLayer::onFootprintChanged(): num footprint points: %lu,"
 //           " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
 //           layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
  std::cout<<"InflationLayer::onFootprintChanged(): num footprint points:" << 
  layered_costmap_->getFootprint().size() << "inscribed_radius_ = "<< inscribed_radius_ << 
  "inflation_radius_ =" << inflation_radius_ << std::endl;
}

void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  //std::cout << "cached_costs_ = " << static_cast<int> (cached_costs_[2][2])<< std::endl;
  //std::cout << "enter inflation layer InflationLayer  updateCosts..." << std::endl;
  //std::cout << "cell_inflation_radius_ = " << cell_inflation_radius_ << std::endl;
  //std::cout << "!enabled = "  << !enabled_ << std::endl;
  //用指针master_array指向主地图，并获取主地图的尺寸，确认seen_数组被正确设置。
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  //if (!enabled_ || (cell_inflation_radius_ == 0))
  //  return;
  
  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  //ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
  //std::cout << "size_x = " << size_x << std::endl;
  //std::cout << "size_y = " << size_y <<  std::endl;

  if (seen_ == NULL) {
    //ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
    std::cout << "InflationLayer::updateCosts(): seen_ array is NULL" << std::endl;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  else if (seen_size_ != size_x * size_y)
  {
    //ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
    std::cout << "InflationLayer::updateCosts(): seen_ array size is wrong" << std::endl;
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  //边际膨胀
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);
 

  //std::cout << "边际膨胀 : " <<"mini = " << min_i << " " << "min_j =" << min_j << " " << "max_i = " << max_i << " " << "max_j = " << max_j << std::endl;

  // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

  //接下来遍历bound中的cell，找到cost为LETHAL_OBSTACLE，即障碍物cell，将其以CellData形式放进inflation_cells_[0.0]中，inflation_cells_的定义如下：

  //std::map<double, std::vector > inflation_cells_;
  //它是一个映射，由浮点数→CellData数组，CellData这个类定义在inflation_layer.h中，
  //是专门用来记录当前cell的索引和与它最近的障碍物的索引的。这个映射以距离障碍物的距离为标准给bound内的cell归类，为膨胀做准备；
  //自然距离为0对应的cell即障碍物cell本身，
  //目前得到的inflation_cells_只包括障碍物本身。

  // Start with lethal obstacles: by definition distance is 0.0
  std::vector<CellData>& obs_bin = inflation_cells_[0.0];
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost >= LETHAL_OBSTACLE)
      {
        obs_bin.push_back(CellData(index, i, j, i, j));
      }
    }
  }
  //std::cout << "obs_bin:" << obs_bin.size() << std::endl;

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  //通过增加距离处理细胞；新的单元格被附加到相应的距离仓中，因此它们
  //可以超越先前插入但距离更远的单元格
  std::map<double, std::vector<CellData> >::iterator bin;
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i)
    {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

     // std::cout << "mx = " << mx  << " " << "my = " << my <<  std::endl;
    //  std::cout << "sx = " << sx  << " " << "sy = " << sy <<  std::endl;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = costLookup(mx, my, sx, sy);
     // std::cout << "costs:=" << static_cast<int>(cost) << std::endl;
     // std::cout << "cell_inflation_radius_ =" << cell_inflation_radius_ << std::endl;
      unsigned char old_cost = master_array[index];
      //std::cout << "old_cost:" << static_cast<double>(old_cost)<< std::endl;
    // if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
      //master_array[index] = cost;//linshi
       // std::cout << "master_array[" << index << "] = " << static_cast<int>(cost) << std::endl;
     // else
     //   master_array[index] = std::max(old_cost, cost);
       //std::cout << "new_cost:" << static_cast<double>(master_array[index])<< std::endl;
      // attempt to put the neighbors of the current cell onto the inflation list

      if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
       master_array[index] = cost;
      else
      master_array[index] = std::max(old_cost, cost);
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);
    }
  }
  #if 0
  /*
  cv::Mat map_info__(size_x, size_y, CV_8UC1);//initlize
  cv::Mat map_info_Rotate__(size_x, size_y, CV_8UC1);//initlize

    for(int i = (size_x)*(size_y) - 1; i > 0; i--)
        {
            int x = i%size_x;  //还原为像素坐标
            int y = i/size_x;  //还原为像素坐标
            //unsigned char value = 255;
            map_info__.at<unsigned char>(x, y) =254 - master_array[i];
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
         cv::imshow("map_info_Rotate_inflat", map_info_Rotate__);
         cv::waitKey(0);
  */
  #endif
  inflation_cells_.clear();
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */

/**
*@brief给定成本图中某个单元格的索引，将其放入等待障碍物膨胀的列表中
*@param grid成本图
*@param index单元格的索引
*@param mx单元格的x坐标（可以根据索引计算，但可以节省存储时间）
*@param my单元格的y坐标（可以根据索引计算，但可以节省存储时间）
*@param src_x障碍点通货膨胀的x指数开始于
*@param src_y障碍点通货膨胀的y指数始于
*/
inline void InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index])
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_)
      return;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}

void InflationLayer::computeCaches()
{
  //这里称cached_distances_和cached_costs_为参考矩阵，因为它们是后续膨胀计算的参照物。
  //由于是基于膨胀参数来设置参考矩阵的，所以当cell_inflation_radius_==0，直接返回。
  //第二个if结构是用来初始化这两个参考矩阵的，只在第一次进入时执行，它们的
  //大小都是(cell_inflation_radius_ + 2)x(cell_inflation_radius_ + 2)，
  //设置cached_distances_矩阵的元素值为每个元素到(0,0)点的三角距离。
  //最后调用computeCosts函数将cached_distances_矩阵“翻译”到cached_costs_矩阵。

  if (cell_inflation_radius_ == 0)
    return;
  //std::cout << "bases" << std::endl;
  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    //std::cout << "isudatr" << std::endl;
    deleteKernels();

    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        cached_distances_[i][j] = hypot(i, j);
        std::cout <<  hypot(i, j) << " ";
      }
      std::cout << std::endl;
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }
  std::cout << "dayin:" << std::endl;
  //设置cached_cell_inflation_radius_，所以第二次再进入的时候就不再执行这个if结构

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
      std::cout <<  static_cast<double> (cached_costs_[i][j]) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "end" << std::endl;
}

void InflationLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

void InflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
  std::cout << "set" << std::endl;
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
  {
    std::cout << "is or out" << std::endl;
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
   // std::cout << "cell_inflation_radius_ = " << cell_inflation_radius_ << std::endl;
   // std::cout << "inflation_radius_ = " << inflation_radius_ << std::endl;
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }
}

}  // namespace costmap_2d
