#include <navigation/navigation.h>
namespace navigation_system
{

   navigation::navigation(costmap_2d::Costmap2DROS* planner_costmap_ros):planner_costmap_ros_(planner_costmap_ros){}
   
   void navigation::start()
   {
            {
               boost::thread *navigation_thread_ = new boost::thread(&navigation::makePlan,this);
               //lock.unlock();
               navigation_thread_->join();
            }
            //boost::thread navigation_thread(&navigation::makePlan,this);
   }
   navigation::~navigation()
   {
      delete navigation_thread_;
   }
   bool navigation::makePlan()
   {
      cv::namedWindow("Dynamic Image", cv::WINDOW_AUTOSIZE); // 创建一个窗口
     // while(1)
      {
         //boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
         global_cost_map_ = planner_costmap_ros_->getCostmap();
         costs_           = global_cost_map_->getCharMap(); 
         //boost::unique_lock<Costmap2D::mutex_t> lock(*(global_cost_map_->getMutex()));
         //std::cout << "global_cost_map_->getSizeInCellsX() =" << global_cost_map_->getSizeInCellsX() << std::endl;
         //std::cout << "global_cost_map_->getSizeInCellsY() =" << global_cost_map_->getSizeInCellsY() << std::endl;
         int leng = 0;

         //global_make plan
         double start_x =728;//728;//891;//94;//583; //94;//600;//94; width
         double start_y =1521 - 937;//1521 - 937;//// 99;//1657-883;//99;//1521-930;// 99;height
         double goal_x  =748;//1354;//32;//1238;//158;//11;// 16;//150;//round(goal_msg->pose.position.x*10)/10;
         double goal_y  =1521- 343;//1521 - 317;//1657 - 312;//16;//118;//23;// 31;//round(goal_msg->pose.position.y*10)/10;
         //navigation::globalPlan(double start_x,double start_y,double goal_x,double goal_y);
         //cv::Mat m4 = cv::imread("/home/juchunyu/20231013/NewCostmap/costmap_2d/map/PM.pgm",cv::IMREAD_GRAYSCALE); 
       // while(1)
       {
         navigation::sleep(600000);
         std::cout << "sleep...end..." << std::endl;
         navigation::globalPlan(start_x,start_y,goal_x,goal_y);
         std::cout << "planning...end..." << std::endl;
         //std::cout << "charmap:"<<std::endl;
        // std::cout << " (global_cost_map_->getSizeInCellsX()) = " <<  (global_cost_map_->getSizeInCellsX()) << std::endl;
       //  std::cout << "global_cost_map_->getSizeInCellsY() = " << global_cost_map_->getSizeInCellsY() << std::endl;
        // std::cout << "global_cost_map_ ->getOriginX() = "  << global_cost_map_ ->getOriginX() << std::endl;
        // std::cout << "global_cost_map_ ->getOriginY() = "  << global_cost_map_ ->getOriginY() << std::endl;
         //for(int i = 0; i < (global_cost_map_->getSizeInCellsX()) * (global_cost_map_->getSizeInCellsY());i++)
         {    //std::cout<<"printf enter....";
            //if(static_cast<int>(costs_[i]) != 254)
            //std::cout<<static_cast<int>(costs_[i]) << " " << std::endl;
            // leng++;
         }

       // #if 1
       
        //cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/AStar-ROS/map/map.pgm",cv::IMREAD_GRAYSCALE);
        cv::Mat m4 = cv::imread("/home/juchunyu/20231013/globalPlanner/PM.pgm",cv::IMREAD_GRAYSCALE);
        //cv::Mat m4 = cv::imread("/home/juchunyu/20231013/NewCostmap/costmap_2d/map/PM.pgm",cv::IMREAD_GRAYSCALE);
        //cv::imshow("originalcharmap",m4);
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
       
        for(int i = 0;i < plan_result_cell_.size();i++){
           map_info_temp.at<unsigned char>(plan_result_cell_[i-1].y,plan_result_cell_[i-1].x) = 0;//plan traj plot
        }
          #if 1
     
         //int l = 0;int k = global_cost_map_->getSizeInCellsY();
         for(int j = 0,k =  global_cost_map_->getSizeInCellsY();k > 0 && j < global_cost_map_->getSizeInCellsX(); j++,k--){
               for(int i = 0,l = 0;l < global_cost_map_->getSizeInCellsX() &&  i < global_cost_map_->getSizeInCellsX();i++,l++){
                  map_info.at<unsigned char>(j,i) = map_info_temp.at<unsigned char>(k, l);
            }
         }
   
         
          /*  
        for(int i = 0;i < plan_result_cell_.size();i++){
            if(i == 0) 
                 continue;
            cv::Point p1 = {plan_result_cell_[i-1].y,plan_result_cell_[i-1].x};
            cv::Point p2 = {plan_result_cell_[i].y,plan_result_cell_[i].x};
            cv::line(map_info, p1, p2, cv::Scalar(0, 0, 255), 2);
        }
*/

    /*
         int l = 0;int k = global_cost_map_->getSizeInCellsY();
         for(int j = 0,k =  global_cost_map_->getSizeInCellsY(); j < global_cost_map_->getSizeInCellsX(); j++,k--){
                  for(int i = 0,l = 0; i < global_cost_map_->getSizeInCellsX();i++,l++){
                  map_info_Rotate.at<unsigned char>(l,k) = map_info.at<unsigned char>(i, j);
            }
         }
         */
         
         cv::imshow("map_info_char_map", map_info);
         //cv::rotate(map_info, map_info_Rotate,cv::ROTATE_90_COUNTERCLOCKWISE);// cv::ROTATE_90_CLOCKWISE);
         //cv::imshow("map_info_Rotate_char_map", map_info_Rotate);
         //boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        //	cv::imshow("map_info", map_info.t());
         cv::waitKey(0.5);
      if (cv::waitKey(1) == 'q') {
          // break;
      }
         #endif
        // std::cout << "length" << leng << std::endl; 
         
         //std::cout << "map length = " << leng << std::endl;
         //std::cout<<std::endl;
      }
       // 销毁所有打开的窗口
      cv::destroyAllWindows();
      }

   }

   void navigation::sleep(int time){
       clock_t head = clock();
        while (clock() - head <= time) {}
   }



  bool navigation::globalPlan(double start_x,double start_y,double goal_x,double goal_y){

         PathGenerator PG;

         int  map_info_width  = global_cost_map_->getSizeInCellsX();//m4.cols;
         int  map_info_height = global_cost_map_->getSizeInCellsY(); //m4.rows;
         
         // Generate Map, Options
         PG.map_generator_.setWorldSize({map_info_width, map_info_height},costs_,map_info_width,map_info_height); //{x, y}
     
         //PG.map_generator_.setWorldSize({map_info_height, map_info_width}); //{x, y}
         PG.map_generator_.setHeuristic(AStar::Heuristic::manhattan);
         //PG.map_generator_.setHeuristic(AStar::Heuristic::euclidean);
        // euclidean
         PG.map_generator_.setDiagonalMovement(true);


         // add obstacle to map
         /*
         for(int i = 0; i< map_info_width * map_info_height; i++)
         {
            int x = i%map_info_width;
            int y = i/map_info_width;
            
            //cout<<"sum:"<<map_info_width*map_info_height<<endl;
           // double v = double(i/(map_info_width*map_info_height));
            //std::cout << i << std::endl;
            //cout<<v<<"%"<<endl;
            //std::cout << "costsss = " << static_cast<int>(costs_[i]) << std::endl;
            unsigned char value = 0;
            if((costs_[i]) > value)
            {  
               std::cout  << "obstacle = " << "[" << x << "," << y << "]" << std::endl;
               PG.map_generator_.addCollision({x, y}, 3);
            }
         } 
          */

         
         AStar::Vec2i source;
         source.x = start_x;
         source.y = start_y;
         //source.x = 94;//0;//94;//(0 - origin[0]) /  map_resolution;
         //source.y = 99;//99;//(0 - origin[1]) / map_resolution;

         AStar::Vec2i target;
         target.x = goal_x;
         target.y = goal_y;
         //target.x = 162;//6;//2;//161;//(goal_x - origin[0]) / map_resolution;
         //target.y = 105;//3;//9;//112; //(goal_y - origin[1]) / map_resolution;

         // Find Path
         auto path =  PG.map_generator_.findPath(source, target);
         
         if(path.empty())
         {
            cout<<"\033[1;31mFail generate path!\033[0m"<<endl;
         }


         for(auto coordinate=path.end()-1; coordinate>=path.begin(); --coordinate)
         {
            //geometry_msg:: point_pose;
            point_cell points;
            points.x = coordinate->x;
            points.y = coordinate->y;

            plan_result_cell_.push_back(points);

            std::cout << "x = " << coordinate->x << "  " << " y = " << coordinate->y << std::endl;

             // Remmaping coordinate
            //point_pose.pose.position.x = (coordinate->x + map_info_.origin.position.x / map_info_.resolution) * map_info_.resolution;
            //point_pose.pose.position.y = (coordinate->y + map_info_.origin.position.y / map_info_.resolution) * map_info_.resolution;
            //path_msg.poses.push_back(point_pose);
            //cout<<coordinate->x<<" "<<coordinate->y<<endl;
         }

  }
   

}