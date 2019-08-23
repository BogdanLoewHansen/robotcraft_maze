#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_arduino/Test.h>
#include <string>
#include <list>
#include <tuple>

#include <vector>
#include <algorithm>
#include <utility>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"


#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na


class RobotDriver
{
private:
    //Initialising node NodeHandle
    ros::NodeHandle n;

    //Keep track of current ctime
    ros::Time current_time;
    ros::Time last_time = ros::Time::now();


    //Declare transform broadcaster to send messages via tf and ROS
    tf::TransformBroadcaster odom_broadcaster;

    double v;
    double w;

    int endX, endY;
    int startX, startY;


    //Publisher Topics
    ros::Publisher cmd_vel_pub;
    ros::Publisher odom_pub;
    ros::Publisher set_pose_pub;
    ros::Publisher rgb_leds_pub;
    ros::Publisher ir_front_sensor;
    ros::Publisher ir_left_sensor;
    ros::Publisher ir_right_sensor;

    //Subscriber Topics
    ros::Subscriber left_dist_sub;
    ros::Subscriber right_dist_sub;
    ros::Subscriber front_dist_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber reactive_vel_sub;
    ros::Subscriber map_sub;

    //Service client
    ros::ServiceClient buzzer_client;


    double front_obstacle_distance;
    double right_obstacle_distance;
    double left_obstacle_distance;


    geometry_msgs::Twist calculateCommand(){
        auto msg = geometry_msgs::Twist();
    }

    //Callback functions for subscribers
    void poseCallback(const geometry_msgs::Pose2D& pose_msg){
      //  relay <pose_sub> [odom_pub];

        current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_msg.theta);

        //first, we'll publish the transform over tf - broadcasts to rviz
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pose_msg.x;
        odom_trans.transform.translation.y = pose_msg.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = pose_msg.x;
        odom.pose.pose.position.y = pose_msg.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = w;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;

    }

    void leftCallback( const sensor_msgs::LaserScan::ConstPtr& msg){
        sensor_msgs::Range ir_left;
        //relay <left_dist_sub> [ir_left_sensor]

        //.data is for acces to all the data from the msg
        float sensor_val_left = msg->ranges[0];
        if(sensor_val_left < 0.15){
            ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the left side", sensor_val_left);
        }

        ir_left.header.frame_id = "base_link";
        ir_left.radiation_type = 1;
        ir_left.field_of_view = 0.034906585;
        ir_left.min_range = 0.1;
        ir_left.max_range = 0.8;
        ir_left.range = sensor_val_left;

        //Publish the left sensor value to ir_left_sensor
        ir_left_sensor.publish(ir_left);



    }

    void rightCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        sensor_msgs::Range ir_right;
        //relay <right_dist_sub> [ir_right_sensor]

        float sensor_val_right = msg->ranges[0];
        if(sensor_val_right < 0.15){
            ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the right side", sensor_val_right);
        }

        ir_right.header.frame_id = "base_link";
        ir_right.radiation_type = 1;
        ir_right.field_of_view = 0.034906585;
        ir_right.min_range = 0.1;
        ir_right.max_range = 0.8;
        ir_right.range = sensor_val_right;

        //Publish the right sensor value to ir_right_sensor
        ir_right_sensor.publish(ir_right);
    }

    void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        sensor_msgs::Range ir_front;
        //relay <front_dist_sub> [ir_front_sensor]

        float sensor_val_front = msg->ranges[0];
        if(sensor_val_front < 0.15){
            ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the front side", sensor_val_front);
        }

        ir_front.header.frame_id = "base_link";
        ir_front.radiation_type = 1;
        ir_front.field_of_view = 0.034906585;
        ir_front.min_range = 0.1;
        ir_front.max_range = 0.8;
        ir_front.range = sensor_val_front;

        //Publish the front sensor value to ir_front_sensor
        ir_front_sensor.publish(ir_front);
    }



    void reactiveVelCallback(const geometry_msgs::Twist& vel_msg){
        // Update globally stored velocities for further use/publishing
        v=vel_msg.linear.x;
        w=vel_msg.angular.z;
    }

    geometry_msgs::Twist cmdVelUpdate(){
        auto vel_MSG = geometry_msgs::Twist();
        // Update velocities to be published
        vel_MSG.linear.x = v;
        vel_MSG.angular.z = w;
        return vel_MSG;
    }

    geometry_msgs::Pose2D setPose(){
        auto pose_MSG = geometry_msgs::Pose2D();
        // Set position to zero
        pose_MSG.x = 4.0;
        pose_MSG.y = 4.0;
        pose_MSG.theta = 0.0;
        return pose_MSG;
    }

    std_msgs::UInt8MultiArray setLEDs(){
        auto rgb_MSG = std_msgs::UInt8MultiArray();
        // Set color of led lights, the first 3 enteries are for LED_1 [255,0,0] and the last 3 for LED_2 [0,255,0]
        // THIS IS NOT SET UP CORRECTLY, FOLLOW THIS EXAMPKE: http://alexsleat.co.uk/2011/07/02/ros-publishing-and-subscribing-to-arrays/
        //rgb_MSG.data = [255,0,0,0,255,0];
        return rgb_MSG;
    }

    // NEED HELP WITH STRING IN C++

    void switchBuzzerState(std::string c){
        // Set led to "0" or "1" by char c
        rosserial_arduino::Test Buzzer_ctr;
        Buzzer_ctr.request.input = c;

        //NOT SURE IF THIS IS THE CORRECT WAY TO ACCESS CLIENT
        if(this->buzzer_client.call(Buzzer_ctr)){
            ROS_INFO_STREAM(Buzzer_ctr.response.output);
        }else{
            ROS_ERROR("Failed to call service ");
        }
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        std_msgs::Header header = msg->header;
        nav_msgs::MapMetaData info = msg->info;
        ROS_INFO("Got map %d %d", info.width, info.height);

        /*
        for(int i = 0; i < 100+(info.width*info.height); ++i){
            std::cout << static_cast<int>(msg->data[i]) << " ";
        }

        */
        int cols = info.width, rows = info.height;
        auto p = [&](int x, int y) { return x * cols + y;  };

        //Binary map
        int** M = new int*[rows];
        for (int i = 0; i < rows; ++i){
            M[i] = new int[cols];
        }

        //Potential filed
        int** C = new int*[rows];
        for (int i = 0; i < rows; ++i){
            C[i] = new int[cols];
        }

        //Shortest path
        int** P = new int*[rows];
        for (int i = 0; i < rows; ++i){
            P[i] = new int[cols];
        }
        
        std::cout << "\n\n BINARY MAP \n\n";
        for (int i = (rows-1); i >= 0; --i) {   // for each row
            for (int j = 0; j < cols; ++j) { // for each column
                if(msg->data[i*cols + j] > 0){
                    M[i][j] = 1;
                } else{
                    M[i][j] = 0;
                }
                std::cout << M[i][j] << " ";
            }
            std::cout << "\n";
        }

        for(int x = (rows-1); x >= 0; --x){
            for(int y = 0; y < cols; y++){
                if(x == 0 || y == 0 || x == (cols -1) || y == (rows -1) || M[x][y] == 1){
                    C[rows-1-x][y] = -1;
                }else{
                    C[rows-1-x][y] = 0;
                }
            }
        }



        
        std::list<std::tuple<int,int,int>> nodes;

        nodes.push_back({endX,endY,1});

        while(!nodes.empty()){
            std::list<std::tuple<int,int,int>> new_nodes;

            for(auto &n : nodes){
                int x = std::get<0>(n);
                int y = std::get<1>(n);
                int d = std::get<2>(n);

                C[x][y] = d;

                // Check south
                if((x+1) < rows && C[x+1][y] == 0){
                    new_nodes.push_back({x+1,y,d+1});
                }
                // Check north
                if((x-1) >= 0 && C[x-1][y] == 0){
                    new_nodes.push_back({x-1,y,d+1});
                }
                // Check east
                if((y+1) < cols && C[x][y+1] == 0){
                    new_nodes.push_back({x,y+1,d+1});
                }
                // Check west
                if((y-1) >= 0 && C[x][y-1] == 0){
                    new_nodes.push_back({x,y-1,d+1});
                }

                // Check south east
                if((x+1) < rows && (y+1) < cols && C[x+1][y+1] == 0){
                    new_nodes.push_back({x+1,y+1,d+1});
                }
                // Check north east
                if((x-1) >= 0 && (y+1) < cols && C[x-1][y+1] == 0){
                    new_nodes.push_back({x-1,y+1,d+1});
                }
                // Check south west
                if((x+1) < rows && (y-1) >= 0 && C[x+1][y-1] == 0){
                    new_nodes.push_back({x+1,y-1,d+1});
                }
                // Check north west
                if((x-1) >= 0 && (y-1) >= 0  && C[x-1][y-1] == 0){
                    new_nodes.push_back({x-1,y-1,d+1});
                }


            }
            // Sort the nodes - This will stack up nodes that are similar: A, B, B, B, B, C, D, D, E, F, F
            new_nodes.sort([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                // In this instance I dont care how the values are sorted, so long as nodes that
                // represent the same location are adjacent in the list. I can use the p() lambda
                // to generate a unique 1D value for a 2D coordinate, so I'll sort by that.
                return p(std::get<0>(n1), std::get<1>(n1)) < p(std::get<0>(n2), std::get<1>(n2));
            });

            // Use "unique" function to remove adjacent duplicates       : A, B, -, -, -, C, D, -, E, F -
            // and also erase them                                       : A, B, C, D, E, F
            new_nodes.unique([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                return  p(std::get<0>(n1), std::get<1>(n1)) == p(std::get<0>(n2), std::get<1>(n2));
            });

            // We've now processed all the discoverd nodes, so clear the list, and add the newly
            // discovered nodes for processing on the next iteration
            nodes.clear();
            nodes.insert(nodes.begin(), new_nodes.begin(), new_nodes.end());
        }


        std::cout << "\n\n VISUALIZABLE BINARY \n\n";
        

        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                //std::cout << C[x][y] << " ";
                if(C[x][y] == -1){
                    std::cout << "X ";
                }
                if(C[x][y] >= 0){
                    std::cout << "- ";
                }
            }
            std::cout << "\n";
        }


        std::list<std::pair<int,int>> path;
        path.push_back({startX,startY});
        int locX = startX;
        int locY = startY;
        bool no_path = false;

        while(!(locX == endX && locY == endY) && !no_path){
            std::list<std::tuple<int,int,int>> listNeighbours;

            // Check south
            if((locX+1) < rows && C[locX+1][locY] > 0){
                listNeighbours.push_back({locX+1,locY,C[locX+1][locY]});
            }
            // Check north
            if((locX-1) >= 0 && C[locX-1][locY] > 0){
                listNeighbours.push_back({locX-1,locY,C[locX-1][locY]});
            }
            // Check east
            if((locY+1) < cols && C[locX][locY+1] > 0){
                listNeighbours.push_back({locX,locY+1,C[locX][locY+1]});
            }
            // Check west
            if((locY-1) >= 0 && C[locX][locY-1] > 0){
                listNeighbours.push_back({locX,locY-1,C[locX][locY-1]});
            }

            // Check south east
            if((locX+1) < rows && (locY+1) < cols && C[locX+1][locY+1] > 0){
                listNeighbours.push_back({locX+1,locY+1,C[locX+1][locY+1]});
            }
            // Check north east
            if((locX-1) >= 0 && (locY+1) < cols && C[locX-1][locY+1] > 0){
                listNeighbours.push_back({locX-1,locY+1,C[locX-1][locY+1]});
            }
            // Check south west
            if((locX+1) < rows && (locY-1) >= 0 && C[locX+1][locY-1] > 0){
                listNeighbours.push_back({locX+1,locY-1,C[locX+1][locY-1]});
            }
            // Check north west
            if((locX-1) >= 0 && (locY-1) >= 0  && C[locX-1][locY-1] > 0){
                listNeighbours.push_back({locX-1,locY-1,C[locX-1][locY-1]});
            }


            listNeighbours.sort([&](const std::tuple<int, int, int> &n1, const std::tuple<int, int, int> &n2)
            {
                return std::get<2>(n1) < std::get<2>(n2); // Compare distances
            });

            if (listNeighbours.empty()) // Neighbour is invalid or no possible path
                no_path = true;
            else
            {
                locX = std::get<0>(listNeighbours.front());
                locY = std::get<1>(listNeighbours.front());
                path.push_back({ locX, locY });
            }

        }

        P = C;
        int p_x, p_y;
        for (auto &a : path){
            p_x = a.first;
            p_y = a.second;
            P[p_x][p_y] = 0;
        }

        std::cout << "\n\n SHORTEST PATH \n\n";

        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                if(P[x][y] == -1){
                    std::cout << "X ";
                }
                if(P[x][y] == 0){
                    std::cout << "O ";
                }
                if(P[x][y] > 0){
                    std::cout << "- ";
                }
            }
            std::cout << "\n";
        }



        /*

        nav_msgs::OccupancyGrid* newGrid = map.Grid();
        newGrid->header = header;
        newGrid->info = info;
        map_pub.publish(*newGrid);
        */
        
        for (int i = 0; i < rows; ++i){
            delete [] M[i];
        }
        delete [] M;

        for (int i = 0; i < rows; ++i){
            delete [] C[i];
        }
        delete [] C;

        
    }


public:
  //constructor
    RobotDriver(){
        // Initialize ROS
        this->n = ros::NodeHandle();
        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        //this->odom_pub = this->n.advertise<nav_msgs::Odometry>("odom", 10);
        //this->set_pose_pub = this->n.advertise<geometry_msgs::Pose2D>("set_pose", 10);
        this->ir_front_sensor = this->n.advertise<sensor_msgs::Range>("ir_front_sensor", 10);
        this->ir_left_sensor = this->n.advertise<sensor_msgs::Range>("ir_left_sensor", 10);
        this->ir_right_sensor = this->n.advertise<sensor_msgs::Range>("ir_right_sensor", 10);

        // Create a subscriber for laser scans
        //this->pose_sub = n.subscribe("pose", 10, &RobotDriver::poseCallback, this);
        this->left_dist_sub = n.subscribe("base_scan_2", 10, &RobotDriver::leftCallback, this);
        this->right_dist_sub = n.subscribe("base_scan_3", 10, &RobotDriver::rightCallback, this);
        this->front_dist_sub = n.subscribe("base_scan_1", 10, &RobotDriver::frontCallback, this);
        this->reactive_vel_sub = n.subscribe("reactive_vel", 10, &RobotDriver::reactiveVelCallback, this);
        this->map_sub = n.subscribe("map",10,&RobotDriver::mapCallback,this);


        this->n.getParam("/startX", startX);
        this->n.getParam("/startY", startY);
        this->n.getParam("/endX", endX);
        this->n.getParam("/endY", endY);

    }



    void run(){
        int count = 0;

        // Send messages in a loop
        ros::Rate loop_rate(10);

        while (ros::ok())
        {

            // Calculate the command to apply
            auto msg = calculateCommand();
            auto vel_MSG = cmdVelUpdate();
            auto rgb_MSG = setLEDs();
            auto pose_MSG = setPose();

            //HAVENT CREATED THESE VARIABLES YET
            // Publish the new command
            this->cmd_vel_pub.publish(vel_MSG);
            //this->rgb_leds_pub.publish(rgb_MSG);
            //this->set_pose_pub.publish(pose_MSG);


            std::string on = "1";
            std::string off = "0";

            // Buzz from count 100 to 500
            //switchBuzzerState(on);


            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){

    // Initialize ROS
    ros::init(argc, argv, "message_service");

    // Create our controller object and run it
    auto controller = RobotDriver();
    controller.run();
}
