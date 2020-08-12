#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 1;
    pose.pose.position.z = 2;
    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 10;
    pose1.pose.position.y = 1;
    pose1.pose.position.z = 2;
    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 20;
    pose2.pose.position.y = 1;
    pose2.pose.position.z = 2;
    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = 15;
    pose3.pose.position.y = 10;
    pose3.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ros::Time time_start = ros::Time::now();
    ros::Time time_start1 = ros::Time::now();
    bool isArnmed = false;
    int  countPosition = 0;
    while(ros::ok()){

        ros::Time loop_time_start = ros::Time::now();
        if( current_state.mode != "OFFBOARD" &&
          	(ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                  	  offb_set_mode.response.mode_sent){
                     	ROS_INFO("Offboard enabled");

                }
                last_request = ros::Time::now();
        } else {
                if( !current_state.armed &&
          	  (ros::Time::now() - last_request > ros::Duration(5.0))){
                  	if( arming_client.call(arm_cmd) &&
                  	    arm_cmd.response.success){
                  	  ROS_INFO("Vehicle armed");
                      time_start1 = ros::Time::now();
                  	}
            	       last_request = ros::Time::now();
                }
        }

        // Update the desired pose:
        /*pose.pose.position.x = 9*sin(2.0*M_PI*0.3*(ros::Time::now()-time_start).toSec());
        pose.pose.position.y = 9*cos(2.0*M_PI*0.3*(ros::Time::now()-time_start).toSec());
        //Update the desired velocity:
        vel.linear.x = 4.0*M_PI*0.1*cos(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
        vel.linear.y = -4.0*M_PI*0.1*sin(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());*/
        if(countPosition == 0){

          for(int i = 150; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
          }

        }
        else if(countPosition == 1){


          std::cout << "enter into into region of sensor" + std::to_string(countPosition) << '\n';
          //std::cout << "enter into if counter 1" + std::to_string(current_pos.pose.pose.position.y) << '\n';

          for(int i = 150; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose1);
            ros::spinOnce();
            rate.sleep();
          }

          std::cout << "exit from region of sensor" + std::to_string(countPosition) << '\n';
          //std::cout << "enter into if counter 1" + std::to_string(current_pos.pose.pose.position.y) << '\n';
        }

        else if(countPosition == 2){


          std::cout << "enter into into region of sensor" + std::to_string(countPosition) << '\n';
          for(int i = 150; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose2);
            ros::spinOnce();
            rate.sleep();
          }

          std::cout << "exit from region of sensor" + std::to_string(countPosition) << '\n';
        } else if(countPosition == 3){


          std::cout << "enter into into region of sensor" + std::to_string(countPosition) << '\n';
          for(int i = 150; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose3);
            ros::spinOnce();
            rate.sleep();
          }

          std::cout << "exit from region of sensor" + std::to_string(countPosition) << '\n';
        }







        if(current_state.armed && ros::Time::now() - time_start1 > ros::Duration(8.0)){
          //std::cout << "position is" + std::to_string(countPosition) << '\n';


          countPosition += 1  ;

          if(countPosition > 3){
            countPosition = 0;
            ROS_INFO("uav0/countPosition 0"  );
            time_start1 = ros::Time::now();
          }
        }

        //local_pos_pub.publish(pose);

        //local_vel_pub.publish(vel);



        ros::spinOnce();
        rate.sleep();
      }





    return 0;
}
