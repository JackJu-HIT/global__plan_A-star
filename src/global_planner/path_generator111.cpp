#include "../include/path_generator/path_generator.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <vector>
#include <Eigen/Dense>

#include <iostream>

using namespace std;


PathGenerator::PathGenerator()
{
    //subscribeAndPublish();
}

PathGenerator::~PathGenerator()
{

}
/*
void PathGenerator::subscribeAndPublish()
{
}

void PathGenerator::gridMapHandler()
{   
}

void PathGenerator::navGoalHandler()
{
}
*/
int main(int argc, char **argv)
{
   PathGenerator PG;
    //Read pgm
   cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/AStar-ROS/map/map.pgm",cv::IMREAD_GRAYSCALE);
  
   cout << "图像宽为：" << m4.cols << "\t高度为：" << m4.rows << "\t通道数为：" << m4.channels() << endl;
  /*
   for (int r = 0; r < m4.rows; ++r) {
        for (int c = 0; c < m4.cols; ++c) {
            int data = m4.at<unsigned char>(r, c);
        }
    }
    cout<<"0"<<endl;
    */
     // Round goal coordinate
    float goal_x            = 10;//round(goal_msg->pose.position.x*10)/10;
    float goal_y            = 10;//round(goal_msg->pose.position.y*10)/10;
    double origin[3]        = {-9.500000, -10.000000, 0.0};
    double  occupied_thresh =  0.65;
    double  free_thresh     =  0.196;
    int Occupy              = 1;
    int NoOccupy            = 0;
    double map_resolution   = 0.05;
    


   /*
    vector<vector<int>> maze = {
		{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
		{ 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1 },
		{ 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1 },
		{ 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1 },
		{ 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
		{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
	};
    */
   vector<vector<int>> maze = {
		{ 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 1, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0 }
	};


    /*
0 0
1 1
2 1
3 2
4 3
5 2
6 3
    */

    vector<int> data;
    for(int i = m4.rows-1;i >= 0;i--){
		for(int j = 0;j < m4.cols;j++){
            if(m4.at<unsigned char>(i,j)/255 > free_thresh){
             data.push_back(Occupy);
		     } else {
              data.push_back(NoOccupy);
		   }
		}
	}
    /*
    int num = 0;
    for(int i =maze.size()-1;i >= 0;i--){
		for(int j = 0;j < maze[0].size();j++){
            num++;
            if(maze[i][j] > free_thresh){
             data.push_back(Occupy);
		     } else {
             data.push_back(NoOccupy);
		   }
		}
	}
    cout<<"cishu"<<num<<endl;
    */

    for(int i = 0;i<data.size();i++){
        cout<<data[i]<<" ";
    }
    cout<<endl;
    
    //cout<<"maze.size()="<<maze.size()<<endl;
    //cout<<"maze[0].size()"<<maze[0].size()<<endl;
	//cv::imshow("res_mat", m4);
	//cv::waitKey(0);


   // map_exsit_ = false;

    //map_info_ = map_msg->info;
    //int  map_info_width  = maze[0].size();//m4.cols;
    //int  map_info_height = maze.size();//m4.rows;
    int  map_info_width  = m4.cols;
    int  map_info_height = m4.rows;
    // Generate Map, Options
    PG.map_generator_.setWorldSize({map_info_width, map_info_height}); //{x, y}
    PG.map_generator_.setHeuristic(AStar::Heuristic::manhattan);
    PG.map_generator_.setDiagonalMovement(true);
    cout<<"-3"<<endl;
    // Add Wall
    int x, y;
    for(int i = 0; i<map_info_width*map_info_height; i++)
    {
        x = i%map_info_width;
        y = i/map_info_width;
        cout<<"i"<<i<<endl;
        cout<<"sum:"<<map_info_width*map_info_height<<endl;
        double v = double(i/(map_info_width*map_info_height));
        cout<<v<<"%"<<endl;

        if(data[i] != 0)
        {   cout<<"obstacle:"<<endl;
             //PG.map_generator_.addCollision({x, y}, 3);
             PG.map_generator_.addCollision({x, y}, 3);
             cout<<"("<<x<<","<<y<<")"<<" ";
        }
        cout<<endl;
    }   
    cout<<"-2"<<endl; 
    // Remmaping coordinate
    AStar::Vec2i target;
    target.x = 162;//6;//2;//161;//(goal_x - origin[0]) / map_resolution;
    target.y = 105;//3;//9;//112; //(goal_y - origin[1]) / map_resolution;

    AStar::Vec2i source;
    source.x = 94;//0;//94;//(0 - origin[0]) /  map_resolution;
    source.y = 99;//99;//(0 - origin[1]) / map_resolution;
    
    cout<<"1"<<endl;
    // Find Path
    auto path =  PG.map_generator_.findPath(source, target);
    cout<<"2"<<endl;
    //cout<<path->x<<' '<<path->y<<endl;
     
    //nav_msgs::Path path_msg;
    if(path.empty())
    {
        cout<<"\033[1;31mFail generate path!\033[0m"<<endl;
        //ROS_INFO("\033[1;31mFail generate path!\033[0m");
    }


    for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
    {
       // geometry_msgs::PoseStamped point_pose;

        // Remmaping coordinate
        //point_pose.pose.position.x = (coordinate->x + map_info_.origin.position.x / map_info_.resolution) * map_info_.resolution;
        //point_pose.pose.position.y = (coordinate->y + map_info_.origin.position.y / map_info_.resolution) * map_info_.resolution;
        //path_msg.poses.push_back(point_pose);
        cout<<coordinate->x<<" "<<coordinate->y<<endl;
    }

    //path_msg.header.frame_id = "map";
   // pub_robot_path_.publish(path_msg);

    //ROS_INFO("\033[1;36mSuccess generate path!\033[0m");


   // ros::spin();
    return 0;
}
