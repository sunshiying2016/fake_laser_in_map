#include <ros/ros.h>
#include <fake_laser_in_map/GetPoseViewArea.h>


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test_service");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<fake_laser_in_map::GetPoseViewArea>("getPoseViewArea");
  fake_laser_in_map::GetPoseViewArea srv;
  srv.request.x = 1.00144;
  srv.request.y = 2.00465;
  srv.request.theta = 1.56944;     // area: 52013.3
  
  if(client.call(srv))
  {
    std::cout<<"Called area: "<<srv.response.area<<std::endl;
  }
  else
  {
    ROS_ERROR("Error when calling!!!!!!!!!!!!!!!");
    return 1;
  }
  
  return 0; 
}