#include <iostream>
#include "math.h"
#include "sstream"

#include "ros/ros.h"
//#include "franka/model.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include<franka_move/IMU_sEMG.h>
#include "geometry_msgs/Pose.h"
#include "examples_common.h"


static franka::Robot robot("192.168.1.2");

class franka_emika
{
public:
  //   franka_emika();
    franka_emika();
     ~franka_emika();
     void start();
private:
     void subCallback(const franka_move::IMU_sEMG::ConstPtr& msg);
     bool emg_data_flag;
     geometry_msgs::Pose pose_now,pose_target_true,pose_target,pose_reference;

     ros::NodeHandle nh;
     ros::Subscriber IMU_sub;
    // franka::Robot robot;
     franka_move::IMU_sEMG global_msg;
};


franka_emika::franka_emika()
{
  //  IMU_sub = nh.subscribe<franka_move::IMU_sEMG>("emg_data", 1000, &franka_emika::subCallback,this);
    emg_data_flag=true;
   // robot=franka::Robot("192.168.1.5");
}

//franka_emika::franka_emika()
//{}




franka_emika::~franka_emika()
{

}

void franka_emika::subCallback(const franka_move::IMU_sEMG::ConstPtr &msg)
{
  ROS_INFO("You are in IMU callback!");
  double beishu = 3;
  if(emg_data_flag)
  {
    //相当于这个数据是一个总的参考点
    global_msg.IMU_datax = msg->IMU_datax;
    global_msg.IMU_datay = msg->IMU_datay;
    global_msg.IMU_dataz = msg->IMU_dataz;
    emg_data_flag=false;
  }
  double delta_x = (msg->IMU_datax - global_msg.IMU_datax)/beishu;
  double delta_y = (msg->IMU_datay - global_msg.IMU_datay)/beishu;
  double delta_z = (msg->IMU_dataz - global_msg.IMU_dataz)/beishu;

  pose_target_true.position.x = pose_reference.position.x - delta_y;
  pose_target_true.position.y = pose_reference.position.y + delta_x;
  pose_target_true.position.z = pose_reference.position.z + delta_z;
}


void franka_emika::start()
{
  setDefaultBehavior(robot);
robot.automaticErrorRecovery();
  try{
  std::array<double, 7> q_goal = {{0, 0, 0, 0, 0, 0, 0}};
  MotionGenerator motion_generator(0.1, q_goal);
  std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
  std::cin.ignore();
//  robot.control(motion_generator);
  std::cout << "Finished moving to initial joint configuration." << std::endl;

  double kRadius=0.2;
  std::array<double, 16> initial_pose;
  double time = 0.0;
//  std::array<double, 7> lower_torque_thresholds_nominal{
//        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
//    std::array<double, 7> upper_torque_thresholds_nominal{
//        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
//    std::array<double, 7> lower_torque_thresholds_acceleration{
//        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
//    std::array<double, 7> upper_torque_thresholds_acceleration{
//        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
//    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
//    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
//    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
//    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
//    robot.setCollisionBehavior(
//        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
//        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
//        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
//        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

 robot.control([&time, &initial_pose, &kRadius](const franka::RobotState& robot_state,                                        franka::Duration period)-> franka::JointVelocities
{
 
       time+=period.toSec();
robot.automaticErrorRecovery();
          franka::JointVelocities velocities = {{0.1*time, 0.0, 0.0, 0, 0, 0, 0}};
          if (time >= 2) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(velocities);
          }
          return velocities;
   
});
}

//      time += period.toSec();
//      if (time == 0.0) {
//        initial_pose = robot_state.O_T_EE_c;
//      }
//      double angle = M_PI / 4 * time;
//      double delta_x = kRadius * std::sin(angle);
//     // double delta_z = kRadius * (std::cos(angle) - 1);
//      std::array<double, 16> new_pose = initial_pose;
//      new_pose[12] += delta_x;
//   //   new_pose[14] += delta_z;
//      if (time >= 10.0) {
//        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
//        return franka::MotionFinished(new_pose);
//      }
//      return new_pose;
//    });
    
    catch (const franka::Exception& e) {
      std::cout << e.what() << std::endl;
    }
}








int main(int argc,char **argv)
{ 
  ros::init(argc,argv,"test");
  franka_emika franka_robot;
  franka_robot.start();
  return 1;
//  ros::NodeHandle nh;

  // ROS_INFO("it is a test");
  // while(ros::ok()){}
  // return 0;
}
