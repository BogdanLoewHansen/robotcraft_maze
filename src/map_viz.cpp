#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na


#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"


//ros::Publisher map_pub;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    ROS_INFO("Got map %d %d", info.width, info.height);

    /*

    int rows = info.width, cols = info.height;
    int** M = new int*[rows];
    for (int i = 0; i < rows; ++i){
        M[i] = new int[cols];
    }
    
    for (int i = 0; i < rows; ++i) {   // for each row
        for (int j = 0; j < cols; ++j) { // for each column
            if(msg->data[i*cols+ j] > 0){
                M[i][j] = 1;
            } else{
                M[i][j] = 0;
            }
            //std::cout << M[i][j] << " ";
        }
        //std::cout << "\n";
    }

    /*

    nav_msgs::OccupancyGrid* newGrid = map.Grid();
    newGrid->header = header;
    newGrid->info = info;
    map_pub.publish(*newGrid);
    */
    /*
    for (int i = 0; i < rows; ++i){
        delete [] M[i];
    }
    delete [] M;
    */
}

int main(int argc, char **argv){
  ros::init(argc, argv, "grid");
  ros::NodeHandle n;

  //map_pub = n.advertise<nav_msgs::OccupancyGrid>("map_out",10);
  ros::Subscriber map_sub = n.subscribe("map",10,mapCallback);
  
  ros::spin();
  return 0;
}
