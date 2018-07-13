//rosrun map_server map_server ~/catkin_ws/src/human_aware_navigation_pkgs/human_aware_navigation/human_aware_nav_launch/maps/turtlebot_at_home_map.yaml 
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/GetMap.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <math.h>
// #include <GetPoseViewArea.h>
#include <fake_laser_in_map/GetPoseViewArea.h>

///map

//如果实时性不够，则可以利用离散的状态，提前做成地图，使用使查表即可


class fake_laser
{
private:
  ros::NodeHandle n_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_pose_;
  ros::ServiceServer service_;
  cv::Mat map_image_;
  nav_msgs::MapMetaData map_metadata_;
  double resolution_;
  std::vector<float> origin_;
//   tf::TransformListener* tf_listener_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double range_max_;   //pixels
  double range_max_meter;   //meters
  double range_num_;  //1081:270度   241：60度
  bool img_show;
  
  cv::Mat map_copy;

  
public:
  fake_laser()
  {}
  ~fake_laser()
  {}
  
  void init()
  {
    angle_min_ = -2.35619449615;
    angle_max_ = 2.35619449615;
    angle_increment_ = 0.00436332309619;
    range_max_ = 0;   //pixels
    range_max_meter = 10;   //meters
    range_num_ = 1081;  //1081:270度   241：60度
    img_show = true;
    
    
    sub_map_ = n_.subscribe("/map", 1000, &fake_laser::callback, this);
    sub_pose_ = n_.subscribe("/initialpose", 1000, &fake_laser::posecallback, this);
    service_ = n_.advertiseService("getPoseViewArea", &fake_laser::getPoseViewAreaService, this);

    ros::ServiceClient map_client = n_.serviceClient<nav_msgs::GetMap> ("/static_map");
	while (! ros::service::waitForService("/static_map",1)){
		ROS_INFO("Waiting for map service");
	}
	
    nav_msgs::GetMap srv;
    map_client.call(srv);
    ROS_INFO_STREAM(srv.response.map.info);
    map_image_ = cv::Mat(srv.response.map.info.height, srv.response.map.info.width,CV_8UC1, cv::Scalar(0));
    map_metadata_ =srv.response.map.info;
    resolution_ = (double) map_metadata_.resolution;
    origin_.push_back(map_metadata_.origin.position.x); //m
    origin_.push_back(map_metadata_.origin.position.y); //m
    origin_.push_back(tf::getYaw(map_metadata_.origin.orientation));
    range_max_ = range_max_meter / resolution_;
    uint8_t *myData = map_image_.data;
    for (int i=0;i<srv.response.map.data.size();i++){
	    if (srv.response.map.data.at(i)==100  || srv.response.map.data.at(i)==-1 ){         //100： 障碍物  -1： 未知
	    }
	    else {  //其他： 空白区域
		    map_image_.data[i] = 255;
// 		        std::cout<<"test: "<<int(map_image_.data[i])<<std::endl;
	    }
    }
  }
  
  void callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
//     std::cout<<"called!!!"<<std::endl;
  }
  
  bool getPoseViewAreaService(fake_laser_in_map::GetPoseViewArea::Request &req, fake_laser_in_map::GetPoseViewArea::Response &res)
  {
    if(img_show){
      cv::cvtColor(map_image_, map_copy, CV_GRAY2BGR);
    }
    
    ros::Time t_start = ros::Time::now();
    geometry_msgs::Point32 point;
    point.x = req.x;
    point.y = req.y;
    point.z = 0;
    std::vector<int> pix;
    pix = worldToMap(&point, &map_metadata_);
    double theta = req.theta;
    
    double dis = 10;
    
    int px1 = pix[0] + int(dis*cos(theta));
    int px2 = pix[1] + int(dis*sin(theta));
    
    int laser_num = 0;
    if(img_show)
    {
      cv::circle(map_copy,cv::Point(pix[0],pix[1]),3,cv::Scalar(0,255,0),2);
      cv::line(map_copy,cv::Point(pix[0],pix[1]),cv::Point(px1,px2),cv::Scalar(0,255,0),2);
    }
    
    double view_area = 0.0;
    
    for(int i_angle=((1-range_num_)/2); i_angle<(1+(range_num_-1)/2); i_angle++)     //540 541
    {
      double angle_now = i_angle * angle_increment_ + theta;
      int px1_max = pix[0] + int(range_max_*cos(angle_now));
      int px2_max = pix[1] + int(range_max_*sin(angle_now));
//       cv::circle(map_copy,cv::Point(px1_max,px2_max),2,cv::Scalar(0),2);
      cv::LineIterator lit(map_image_, cv::Point(pix[0],pix[1]), cv::Point(px1_max,px2_max), 8);
      double dis_point = 0.0;

      for(int i_line=0; i_line<lit.count; i_line++, ++lit)
      {
	cv::Point pt(lit.pos());

	uchar *data = map_image_.data + pt.y*map_image_.step + pt.x*map_image_.elemSize();
// 	std::cout<<"value: "<<(int)data[0]<<std::endl;

// 	cv::imshow("img",map_copy);
// 	cv::waitKey(20);
	
	if((int)data[0] == 0)
	{
	  if(img_show)
	    cv::circle(map_copy,pt,2,cv::Scalar(0,0,255),2);
	  dis_point = sqrt((pt.x - pix[0])*(pt.x - pix[0])+(pt.y - pix[1])*(pt.y - pix[1]));
	  laser_num++;
	  break;
	}
	
	if(i_line == (lit.count-1))
	{
	  if(img_show)
	    cv::circle(map_copy,cv::Point(px1_max,px2_max),2,cv::Scalar(0,0,255),2);
	  dis_point = sqrt((px1_max - pix[0])*(px1_max - pix[0])+(px2_max - pix[1])*(px2_max - pix[1]));
	  laser_num++;
	}
      }
      
      //计算面积
      view_area += dis_point;
      
    }
    
    ros::Time t_end = ros::Time::now();
    ros::Duration t = t_end - t_start;
    
    res.area = view_area;
    
    std::cout<<"num: "<<laser_num<<"  !!!!!!!!!!!!!!!!"<<std::endl;
    std::cout<<"area: "<<view_area<<"  !!!!!!!!!!!!!!!!"<<std::endl;
    std::cout<<"duration: "<<t.toSec()<<"  !!!!!!!!!!!!!!!"<<std::endl;
    
//     cv::imshow("img",map_copy);
//     cv::waitKey(20);
    if(img_show)
      cv::imwrite("/home/sun/img.jpg", map_copy);
    
    
    
    return true;
  }
  
  void posecallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
//     std::cout<<"pose called!!!"<<std::endl;
    if(img_show){
      cv::cvtColor(map_image_, map_copy, CV_GRAY2BGR);
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
//     pose.header.frame_id = "odom";
    
    geometry_msgs::PoseStamped sm;
    std::vector<int> pix;
    sm = transformPoseTo(pose, std::string("map"), false);
    
    ros::Time t_start = ros::Time::now();
    
    geometry_msgs::Point32 point;
    point.x = sm.pose.position.x;
    point.y = sm.pose.position.y;
    point.z = 0.0;
    pix = worldToMap(&point, &map_metadata_);
    double theta = tf::getYaw(sm.pose.orientation);
    
    std::cout<<"Requse pose: "<<std::endl;
    std::cout<<"x: "<<point.x<<std::endl;
    std::cout<<"y: "<<point.y<<std::endl;
    std::cout<<"Theta: "<<theta<<std::endl;
    

	
    double dis = 10;
    
    int px1 = pix[0] + int(dis*cos(theta));
    int px2 = pix[1] + int(dis*sin(theta));
    
    int laser_num = 0;
    if(img_show)
    {
      cv::circle(map_copy,cv::Point(pix[0],pix[1]),3,cv::Scalar(0,255,0),2);
      cv::line(map_copy,cv::Point(pix[0],pix[1]),cv::Point(px1,px2),cv::Scalar(0,255,0),2);
    }
    
    double view_area = 0.0;
    
    for(int i_angle=((1-range_num_)/2); i_angle<(1+(range_num_-1)/2); i_angle++)     //540 541
    {
      double angle_now = i_angle * angle_increment_ + theta;
      int px1_max = pix[0] + int(range_max_*cos(angle_now));
      int px2_max = pix[1] + int(range_max_*sin(angle_now));
//       cv::circle(map_copy,cv::Point(px1_max,px2_max),2,cv::Scalar(0),2);
      cv::LineIterator lit(map_image_, cv::Point(pix[0],pix[1]), cv::Point(px1_max,px2_max), 8);
      double dis_point = 0.0;

      for(int i_line=0; i_line<lit.count; i_line++, ++lit)
      {
	cv::Point pt(lit.pos());

	uchar *data = map_image_.data + pt.y*map_image_.step + pt.x*map_image_.elemSize();
// 	std::cout<<"value: "<<(int)data[0]<<std::endl;

// 	cv::imshow("img",map_copy);
// 	cv::waitKey(20);
	
	if((int)data[0] == 0)
	{
	  if(img_show)
	    cv::circle(map_copy,pt,2,cv::Scalar(0,0,255),2);
	  dis_point = sqrt((pt.x - pix[0])*(pt.x - pix[0])+(pt.y - pix[1])*(pt.y - pix[1]));
	  laser_num++;
	  break;
	}
	
	if(i_line == (lit.count-1))
	{
	  if(img_show)
	    cv::circle(map_copy,cv::Point(px1_max,px2_max),2,cv::Scalar(0,0,255),2);
	  dis_point = sqrt((px1_max - pix[0])*(px1_max - pix[0])+(px2_max - pix[1])*(px2_max - pix[1]));
	  laser_num++;
	}
      }
      
      //计算面积
      view_area += dis_point;
      
    }
    
    ros::Time t_end = ros::Time::now();
    ros::Duration t = t_end - t_start;
    
    std::cout<<"num: "<<laser_num<<"  !!!!!!!!!!!!!!!!"<<std::endl;
    std::cout<<"area: "<<view_area<<"  !!!!!!!!!!!!!!!!"<<std::endl;
    std::cout<<"duration: "<<t.toSec()<<"  !!!!!!!!!!!!!!!"<<std::endl;
    
//     cv::imshow("img",map_copy);
//     cv::waitKey(20);
    if(img_show)
      cv::imwrite("/home/sun/img.jpg", map_copy);
  }
  
  std::vector<int> worldToMap(geometry_msgs::Point32* world_point,nav_msgs::MapMetaData* map_metadata){
	std::vector<int> pixels;
	float x_map = world_point->x - map_metadata->origin.position.x;
	float y_map = world_point->y - map_metadata->origin.position.y;
	pixels.push_back((int)floor(x_map/map_metadata->resolution ));
	pixels.push_back((int)floor(y_map/map_metadata->resolution));
	return pixels;

}
  
  geometry_msgs::PoseStamped transformPoseTo(geometry_msgs::PoseStamped pose_in, std::string frame_out, bool usetime)
  {
	  geometry_msgs::PoseStamped in = pose_in;
	  if(!usetime)
		  in.header.stamp = ros::Time();
		  
	  geometry_msgs::PoseStamped pose_out;
	  
	  geometry_msgs::Quaternion q = in.pose.orientation;
	  if(!isQuaternionValid(q))
	  {
		  ROS_WARN("NavFeatures. transformPoseTo. Quaternion no valid. Creating new quaternion with yaw=0.0");
		  in.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	  }

	  tf::TransformListener tfl;
	  try {
	    ros::Duration timeout(0.5);
	    tfl.waitForTransform(frame_out.c_str(), in.header.frame_id, in.header.stamp,
                         timeout);
		  tfl.transformPose(frame_out.c_str(), in, pose_out);
	  }catch (tf::TransformException ex){

		  ROS_ERROR("NavFeatures. TransformException in method transformPoseTo. TargetFrame: %s : %s", frame_out.c_str(), ex.what());
	  }
	  //printf("Tranform pose. frame_in: %s, x:%.2f, y:%.2f, frame_out: %s, x:%.2f, y:%.2f\n", in.header.frame_id.c_str(), in.pose.position.x, in.pose.position.y, frame_out.c_str(), pose_out.pose.position.x, pose_out.pose.position.y);
	  return pose_out;
  }
  
  bool isQuaternionValid(const geometry_msgs::Quaternion q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
		ROS_ERROR("Quaternion has infs!!!!");
		return false;
    }
    if(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
		ROS_ERROR("Quaternion has nans !!!");
		return false;
	}
	
	if(std::fabs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01) {
		ROS_ERROR("Quaternion malformed, magnitude: %.3f should be 1.0", (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w));
		return false;
	}

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding.");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
}
  
  
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fake_laser");
  fake_laser fl;
  fl.init();
  
  ros::spin();
  return 0;
  
}
