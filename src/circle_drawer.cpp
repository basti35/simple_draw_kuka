#include <ros/ros.h>
#include <lwr_controllers/PoseRPY.h>
#include <tf/transform_listener.h>

void getCurrentPose(lwr_controllers::PoseRPY& currentPose, ros::NodeHandle& nh);
void orientEndEffector(lwr_controllers::PoseRPY initialPose, lwr_controllers::PoseRPY goalPose, ros::Publisher& pose_pub, ros::NodeHandle& nh);
void transposeEndEffector(lwr_controllers::PoseRPY initialPose, lwr_controllers::PoseRPY goalPose, ros::Publisher& pose_pub, ros::NodeHandle& nh);


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "simple_circle_drawer");
  ros::NodeHandle nh;


  //Create a publisher to command robot positions
  ros::Publisher pose_pub = nh.advertise<lwr_controllers::PoseRPY>("/lwr/OneTaskInverseKinematics/command_configuration", 1);

  ros::Rate rate(10.0);

  lwr_controllers::PoseRPY initialPose,
                           goalPose;

  //Zero position. Robot movements will start from this pose.
  goalPose.position.x = 0;
  goalPose.position.y = -0.5;
  goalPose.position.z = 1.6;

  goalPose.orientation.roll = 0;
  goalPose.orientation.pitch = 0;
  goalPose.orientation.yaw = 0;

  //Obtain the robot end_effector pose
  getCurrentPose(initialPose, nh);
  std::cout << "Current pose is:" << initialPose << std::endl;

  std::cout << "Transposing." << std::endl;
  //Move the end effector
  transposeEndEffector(initialPose, goalPose, pose_pub, nh);
  std::cout << "Transpose done." << std::endl;

  //Obtain the robot end_effector pose
  getCurrentPose(initialPose, nh);
  std::cout << "Current pose is:" << initialPose << std::endl;
  
  std::cout << "Orienting." << std::endl;
  //Orient the end effector
  orientEndEffector(initialPose, goalPose, pose_pub, nh);
  std::cout << "Orientation done." << std::endl;

  //Update the transform
  getCurrentPose(initialPose, nh);
  std::cout << "Current pose is:" << initialPose << std::endl;

  //Initialize theta
  float theta = 0;
  float radius = .1;
  
  lwr_controllers::PoseRPY nextPose;

  std::cout << "Drawing the circle." << initialPose << std::endl;
  //Move the end effector
  while(nh.ok())
  {
    nextPose.position.x = initialPose.position.x + cos(theta) * radius;
    nextPose.position.y = initialPose.position.y;
    nextPose.position.z = initialPose.position.z + sin(theta) * radius;
    
    nextPose.orientation.roll = initialPose.orientation.roll;
    nextPose.orientation.pitch = initialPose.orientation.pitch;
    nextPose.orientation.yaw = initialPose.orientation.yaw;

    std::cout << "next pose:" << nextPose << std::endl;
    pose_pub.publish(nextPose);
    theta += 0.1;
    rate.sleep();
  }
}

void getCurrentPose(lwr_controllers::PoseRPY& currentPose, ros::NodeHandle& nh)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::Rate rate(10.0);

  while(nh.ok())
  {
    try
    {
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/lwr_base_link", "/lwr_7_link",
                              now, ros::Duration(5.0));
      listener.lookupTransform("/lwr_base_link", "/lwr_7_link",
                               ros::Time(0), transform);
       
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      //skip the break if there was an exception
      ROS_INFO("Skip the break if there was an exception");
      continue;
    }
    //transform obtained, do the next step
    ROS_INFO("Transform obtained, do the next step");
    break;
  }

  //Grab the current pose from the transform
  currentPose.position.x = transform.getOrigin().x();
  currentPose.position.y = transform.getOrigin().y();
  currentPose.position.z = transform.getOrigin().z() + 1;

  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

  currentPose.orientation.roll = roll;
  currentPose.orientation.pitch = pitch;
  currentPose.orientation.yaw = yaw;
}

void orientEndEffector(lwr_controllers::PoseRPY initialPose, lwr_controllers::PoseRPY goalPose, ros::Publisher& pose_pub, ros::NodeHandle& nh)
{
  ros::Rate rate(5.0);

  lwr_controllers::PoseRPY nextPose;
  nextPose.position.x = initialPose.position.x;
  nextPose.position.y = initialPose.position.y;
  nextPose.position.z = initialPose.position.z;

  nextPose.orientation.roll = initialPose.orientation.roll;
  nextPose.orientation.pitch = initialPose.orientation.pitch;
  nextPose.orientation.yaw = initialPose.orientation.yaw;

  while(nh.ok())
  {
    if( fabs(nextPose.orientation.roll - goalPose.orientation.roll) < 0.011 &&
        fabs(nextPose.orientation.pitch - goalPose.orientation.pitch) < 0.011 &&
        fabs(nextPose.orientation.yaw - goalPose.orientation.yaw) < 0.011 )
      break;


    if(nextPose.orientation.roll < goalPose.orientation.roll)
      nextPose.orientation.roll +=  0.005;
    else
      nextPose.orientation.roll -=  0.005;

    if(nextPose.orientation.pitch < goalPose.orientation.pitch)
      nextPose.orientation.pitch +=  0.005;
    else
      nextPose.orientation.pitch -=  0.005;

    if(nextPose.orientation.yaw < goalPose.orientation.yaw)
      nextPose.orientation.yaw +=  0.005;
    else
      nextPose.orientation.yaw -=  0.005;


    pose_pub.publish(nextPose);
    
    rate.sleep();
  }
}

void transposeEndEffector(lwr_controllers::PoseRPY initialPose, lwr_controllers::PoseRPY goalPose, ros::Publisher& pose_pub, ros::NodeHandle& nh)
{
  ros::Rate rate(5.0);

  lwr_controllers::PoseRPY nextPose;
  nextPose.position.x = initialPose.position.x;
  nextPose.position.y = initialPose.position.y;
  nextPose.position.z = initialPose.position.z;

  nextPose.orientation.roll = initialPose.orientation.roll;
  nextPose.orientation.pitch = initialPose.orientation.pitch;
  nextPose.orientation.yaw = initialPose.orientation.yaw;
  
  std::cout << "goal_pose: " << goalPose << std::endl;

  while(nh.ok())
  {
    if( fabs(nextPose.position.x - goalPose.position.x) < 0.011 &&
        fabs(nextPose.position.y - goalPose.position.y) < 0.011 &&
        fabs(nextPose.position.z - goalPose.position.z) < 0.011 )
      break;


    if(nextPose.position.x < goalPose.position.x)
      nextPose.position.x +=  0.005;
    else
      nextPose.position.x -=  0.005;

    if(nextPose.position.y < goalPose.position.y)
      nextPose.position.y +=  0.005;
    else
      nextPose.position.y -=  0.005;

    if(nextPose.position.z < goalPose.position.z)
      nextPose.position.z +=  0.005;
    else
      nextPose.position.z -=  0.005;

    std::cout << "goal_pose: " << goalPose << std::endl;
    std::cout << "next_pose: " << nextPose << std::endl;
   
    pose_pub.publish(nextPose);
    
    rate.sleep();
  }
}
