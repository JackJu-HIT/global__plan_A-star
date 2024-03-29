/*
* Function: A Global Planner Based on A-star and Cost Map!
* Create by:juchunyu
* Date:2024-03-29 13:59:00
* Last modified:juchunyu
*/

#include<iostream>
#include<iostream>
#include <chrono>
#include <thread>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "../include/global_planner/astar.h"
#include "../include/global_planner/expander.h"
#include "../include/global_planner/traceback.h"
#include "../include/global_planner/grid_path.h"
#include "../include/global_planner/gradient_path.h"
#include "../include/global_planner/quadratic_calculator.h"

struct point_cell
{
      int x;
      int y;
};

costmap_2d::Costmap2DROS*   planner_costmap_ros_;
costmap_2d::Costmap2D* global_cost_map_;
       
//global_planner::PotentialCalculator* p_calc_;
global_planner::QuadraticCalculator* p_calc_;
global_planner::Expander* planner_;
global_planner::Traceback* path_maker_;
float* potential_array_;
float convert_offset_;
unsigned char* costs_; 
std::vector<point_cell> plan_result_cell_;


bool getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                     const point_cell goal,std::vector<point_cell> &plan)
{
    //clear the plan, just in case
    plan.clear();
    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        std::cout << "NO PATH!" << std::endl;
        return false;
    }

    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
       // mapToWorld(point.first, point.second, world_x, world_y);

        point_cell point_cell;
        point_cell.x = point.first;
        point_cell.y = point.second;

        plan.push_back(point_cell); //image cordites //todo:
        
    }
   //if(old_navfn_behavior_){
   plan.push_back(goal);
   
    return !plan.empty();
}


bool makePlanner(point_cell start,point_cell goal,std::vector<point_cell> &plan)
{
    plan.clear();
  
    int nx = global_cost_map_->getSizeInCellsX(), ny = global_cost_map_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];
   //将costmap的所有点都标为有障碍物

     bool found_legal = planner_->calculatePotentials(global_cost_map_->getCharMap(),global_cost_map_,global_cost_map_,start.x, start.y, goal.x, goal.y,
                                                    nx * ny * 2, potential_array_);
    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start.x,start.y, goal.x, goal.y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            point_cell goal_copy = goal;
            //goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            std::cout << "Failed to get a plan from potential when a legal potential was found. This shouldn't happen." << std::endl;
        }
    }else{
        std::cout << "Failed to get a plan." << std::endl;
   }
   delete potential_array_;
   return !plan.empty();
}


void show()
{

  // while(1)
  {
        //cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/AStar-ROS/map/map.pgm",cv::IMREAD_GRAYSCALE);
        //cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/PM.pgm",cv::IMREAD_GRAYSCALE);
        //cv::Mat m4 = cv::imread("/home/juchunyu/20231013/NewCostmap/costmap_2d/map/PM.pgm",cv::IMREAD_GRAYSCALE); 
        cv::Mat m4 = cv::imread("/home/juchunyu/20231013/240308/costmap_2d_A_star/map/PM.pgm",cv::IMREAD_GRAYSCALE);
        
        cv::imshow("originalcharmap",m4);
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_costmap(*(planner_costmap_ros_->getCostmap()->getMutex()));
        cv::Mat map_info(global_cost_map_->getSizeInCellsY(), global_cost_map_->getSizeInCellsX(), CV_8UC1);//initlize
        cv::Mat map_info_temp(global_cost_map_->getSizeInCellsY(), global_cost_map_->getSizeInCellsX(), CV_8UC1);//initlize
        cv::Mat map_info_Rotate(global_cost_map_->getSizeInCellsY(), global_cost_map_->getSizeInCellsX(), CV_8UC1);//initlize
       
        //for(int i = (global_cost_map_->getSizeInCellsX())*(global_cost_map_->getSizeInCellsY()) - 1; i > 0; i--)
        for(int i = 0;i < (global_cost_map_->getSizeInCellsX())*(global_cost_map_->getSizeInCellsY());i++)
        {
            int x = i % global_cost_map_->getSizeInCellsX();  //还原为像素坐标
            int y = i / global_cost_map_->getSizeInCellsX();  //还原为像素坐标
            //unsigned char value = 255;
             map_info_temp.at<unsigned char>(y, x) = 254 - costs_[i];
            //cout<<endl;
        }
        std::cout << "plan_result_cell_.size = " << plan_result_cell_.size() << std::endl;
        for(int i = 0;i < plan_result_cell_.size();i++){
           map_info_temp.at<unsigned char>(plan_result_cell_[i -1].y,plan_result_cell_[i-1].x) = 0;//plan traj plot
        }
     
         //int l = 0;int k = global_cost_map_->getSizeInCellsY();
         for(int j = 0,k =  global_cost_map_->getSizeInCellsY();k > 0 && j < global_cost_map_->getSizeInCellsX(); j++,k--){
               for(int i = 0,l = 0;l < global_cost_map_->getSizeInCellsX() &&  i < global_cost_map_->getSizeInCellsX();i++,l++){
                  map_info.at<unsigned char>(j,i) = map_info_temp.at<unsigned char>(k, l);
            }
         }

         
       cv::imshow("map_info_char_map", map_info);
       cv::waitKey(0.5);
      if (cv::waitKey(1) == 'q') {
          // break;
      }
   }
        
}



int main(){

    // 下面通过模块costmap_2d构建了全局代价地图对象，并通过其成员函数pause()暂停运行，将在必要的功能模块初始化结束之后通过成员接口start()开启。
     planner_costmap_ros_    = new costmap_2d::Costmap2DROS("global_costmap");
      // costmap_2d::Costmap2DROS* planner_costmap_ros_;



      boost::this_thread::sleep_for(boost::chrono::seconds(5));//wait for map intilize ok
    
      global_cost_map_ = planner_costmap_ros_->getCostmap();
      costs_           = global_cost_map_->getCharMap(); 
      unsigned int cx  = global_cost_map_->getSizeInCellsX(), cy = global_cost_map_->getSizeInCellsY();
     
       
      convert_offset_ = 0.5;
            
      p_calc_ = new  global_planner::QuadraticCalculator(cx, cy);
      //p_calc_ = new global_planner::PotentialCalculator(cx, cy);

      planner_ = new global_planner::AStarExpansion(p_calc_, cx, cy);

      bool use_grid_path;
            
      path_maker_ = new global_planner::GridPath(p_calc_);
            
      path_maker_->setLethalCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);//set A-star obstacle cost

      point_cell start;
      point_cell goal;

      double start_x =683;//261;// width
      double start_y =cy - 933;//cy - 484;// 99;height
     
      double goal_x = 1347;//1324;//round(goal_msg->pose.position.x*10)/10;
      double goal_y = cy - 294;//1521- 343;//
      start.x = start_x;
      start.y = start_y;
      goal.x  = goal_x;
      goal.y  = goal_y;

      std::cout << "start:" <<  start.x << " " <<  start.y << std::endl;
      std::cout << "goal:" <<  goal.x << " " <<   goal.y   <<  std::endl;
      
      bool gotPlan = makePlanner(start,goal,plan_result_cell_);

      for(int i = 0; i < plan_result_cell_.size();i++){
         std::cout << plan_result_cell_[i].x << " " << plan_result_cell_[i].y << std::endl;
      }

      show();

}

