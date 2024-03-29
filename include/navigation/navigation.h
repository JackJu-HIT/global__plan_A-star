
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include<iostream>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "../include/path_generator/path_generator.h"
#include "../include/global_planner/astar.h"
#include "../include/global_planner/expander.h"
#include "../include/global_planner/traceback.h"
#include "../include/global_planner/grid_path.h"
#include "../include/global_planner/gradient_path.h"
#include "../inc/teb_config.h"
#include "../inc/pose_se2.h"
#include "../inc/robot_footprint_model.h"
#include "../inc/obstacles.h"
#include "../inc/optimal_planner.h"
#include <boost/smart_ptr.hpp>
//#include "../matplotlib-cpp/matplotlibcpp.h"
#include <vector>
#include <Eigen/Dense>

#include <dds/DDS_Data.h>
#include <dds/DDS_DataPubSubTypes.h>
#include <dds/PP_Data.h>
#include <dds/PP_DataPubSubTypes.h>
#include <dds/DDS_DataSubscriber.h>
#include <dds/subscription_callback_helper.h>
#include <dds/DDS_DataPublisher.h>
#include <ctime>
//#define PLANNING 0
//#define CONTROLLING 1
//#define CLEARING 2

#define MAX_VALUE 10000000000

using namespace std;
using namespace Eigen;
using namespace teb_local_planner;
//namespace plt = matplotlibcpp;//可视化

namespace navigation_system
{
   enum NavigationState{
      PLANNING,
      CONTROLLING,
      CLEARING
  };
  struct pose
  {
    double x;
    double y;
    double z;
    double theta;
  };
  
   struct point_meters
   {
      double x;
      double y;
   };
   struct point_cell
   {
      int x;
      int y;
   };
   struct velocity
   {
      double vel_x;
      double vel_y;
      double vel_z;
      double w;
   };
   struct Robot {
      point_meters center;
      double radius;
      double theta;
   };

class RCToPPSubscriber
{
     public:
      RCToPPSubscriber();

      void ddsRCToPPhandler(const DDSRcToPP* const ddsRCToPPMsg);
      void getTargetPath(std::vector<pose>& path);
      void getCurrentPose(pose& pose);
      public:
        DDS_DataSubscriber <DDSRcToPPPubSubType, DDSRcToPP>  ddsSubRCToPP;
        boost::mutex roboMutex_;
        std::vector<pose> work_Path_;
        boost::recursive_mutex get_work_path_;
        pose m_curPose;


};


class ROSLaserSubscriber
{

     public:
      ROSLaserSubscriber();

      void ddsROSLaserhandler(const NAVPointCloudDDS* const ddsROSLaserMsg);
      public:
        DDS_DataSubscriber <NAVPointCloudDDSPubSubType, NAVPointCloudDDS>  ddsSubRosLaser;
        //getCurrentPose
};


   class navigation
   {
    public:
        navigation();

        ~navigation();

        bool makePlan();

        void start();

        bool globalPlan(double start_x,double start_y,double end_x,double end_y);

        void sleep(int time);

        bool makePlanner(point_cell start,point_cell goal,std::vector<point_cell> &plan);

        void mapToWorld(double mx, double my, double& wx, double& wy);

        bool worldToMap(double wx, double wy, double& mx, double& my);

        bool getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                     const point_cell goal,std::vector<point_cell> &plan);
        int matrixToArray(int x,int y);

        bool calculateControlCommend();
        
        void executeCb();

        bool executeCycle();

        void publishZeroVelocity();

        void resetState();

        bool teb_controller();

        bool getIntersectionPointObstacleAvoidance(const point_cell& RobotPose,int& localStartIndx,int& localEndIndx,std::vector<int>viaPointsX,std::vector<int> viaPointsY);

        bool rayCircleIntersection(point_meters ray_start, point_meters ray_dir, point_meters circle_center, double circle_radius);
        
        bool combinePathAndPrediction(vector<double>& via_points_x,vector<double>& via_points_y,  std::vector<navigation_system::pose> work_Path, double pre_dist);
        
        std::vector<point_meters> smoothCurve(std::vector<point_meters>& points);
    public:
       costmap_2d::Costmap2DROS* planner_costmap_ros_;
       costmap_2d::Costmap2DROS* controller_costmap_ros_;
       costmap_2d::Costmap2D* global_cost_map_;
       costmap_2d::Costmap2DROS* local_costmap_ros_;
       costmap_2d::Costmap2D* local_cost_map_;
       boost::recursive_mutex planner_mutex_;
       std::vector<point_meters> plan_result_meters_;
       std::vector<point_cell> controller_plan_;
       std::vector<point_cell> latest_plan_;
       std::vector<point_cell> plan_result_cell_;
       boost::thread *navigation_thread_;
       boost::thread *navigation_thread_teb_controller_;
       boost::thread *stateMachine_thread_;
       
       global_planner::PotentialCalculator* p_calc_;
       global_planner::Expander* planner_;
       global_planner::Traceback* path_maker_;

       float* potential_array_;
       float convert_offset_;
       unsigned char* costs_; 
       unsigned char* costs_local_; 

       bool  motion_Plan_Request;

       boost::mutex mutex_;
      // boost::condition_variable planner_cond_;
       boost::condition_variable_any planner_cond_;
       bool plan_status_;
       bool runner_;
      
       NavigationState state_;
       bool new_global_plan_;
       bool control_finish_status_;
       bool teb_find_result_;
       velocity control_information_;
       int32_t max_planning_retries_;
       uint32_t planning_retries_;
       std::vector<Eigen::Vector3f> path_;
       teb_local_planner::ObstContainer obstacles_;
       ViaPointContainer via_points_PP_;
       double mindx_previous_;
       double goal_x_temp;
       double goal_y_temp;
       double global_theta_temp;
       vector<double> via_points_x;
       vector<double> via_points_y;
       // 定义发布器
       DDS_DataPublisher< DDSPPToRcPubSubType > ddsPubPPToRC_;
       RCToPPSubscriber* rctoppsub_; //sub pose

       
   };

/*
class Pose3DSubscriber
{

     public:
      Pose3DSubscriber();

      void dds3DPoseHandler(const DDS3DPose* const dds3DPoseMsg);
      public:
        DDS_DataSubscriber <DDS3DPosePubSubType, DDS3DPose>  ddsSub3DPose;
};
*/


}
#endif  // NAVIGATION_H

