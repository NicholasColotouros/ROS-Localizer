#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>

#define METRE_TO_PIXEL_SCALE 50
#define FORWARD_SWIM_SPEED_SCALING 0.2
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0

class GroundTruthPublisher
{
public:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  ros::Subscriber ground_truth_sub;
  geometry_msgs::PoseStamped ground_truth_location;

  cv::Mat map_image;
  cv::Mat drawing_image;

  GroundTruthPublisher( int argc, char** argv )
  {
    image_transport::ImageTransport it(nh);
    pub = it.advertise("/assign2/ground_truth_image", 1);
    map_image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    drawing_image = map_image.clone();

    ground_truth_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &GroundTruthPublisher::groundTruthCallback, this);
   
    ROS_INFO( "Ground truth publisher constructed. Waiting for model state information." );

  }

  void groundTruthCallback( const gazebo_msgs::ModelStates::ConstPtr& ground_truth_state )
  {
    ROS_INFO( "Got ground truth callback." ); 
   
    std::string robot_name = "aqua";

    int index = -1;
    for( unsigned int i=0; i<ground_truth_state->name.size(); i++ )
    {
      if( ground_truth_state->name[i] == robot_name )
      {
        index = i;
        break;
      }
    }

    if( index < 0 )
    {
      ROS_ERROR( "Unable to find aqua model information in gazebo message");
      return;
    }

    ground_truth_location.header.stamp = ros::Time::now();
    ground_truth_location.pose = ground_truth_state->pose[index];

    int ground_truth_robo_image_x = drawing_image.size().width/2 + METRE_TO_PIXEL_SCALE * ground_truth_location.pose.position.x;
    int ground_truth_robo_image_y = drawing_image.size().height/2 - METRE_TO_PIXEL_SCALE * ground_truth_location.pose.position.y;

    double gt_yaw, gt_pitch, gt_roll;
    tf::Quaternion gt_orientation;
    tf::quaternionMsgToTF(ground_truth_location.pose.orientation, gt_orientation);
    tf::Matrix3x3(gt_orientation).getEulerYPR( gt_yaw, gt_pitch, gt_roll );

    int ground_truth_heading_image_x = ground_truth_robo_image_x + HEADING_GRAPHIC_LENGTH * cos(-gt_yaw);
    int ground_truth_heading_image_y = ground_truth_robo_image_y + HEADING_GRAPHIC_LENGTH * sin(-gt_yaw);

    ROS_INFO( "Ground truth image point at %d, %d", ground_truth_robo_image_x, ground_truth_robo_image_y);

    cv::circle( drawing_image, cv::Point(ground_truth_robo_image_x, ground_truth_robo_image_y), POSITION_GRAPHIC_RADIUS, CV_RGB(0,0,250), -1);
    cv::line( drawing_image, cv::Point(ground_truth_robo_image_x, ground_truth_robo_image_y), cv::Point(ground_truth_heading_image_x, ground_truth_heading_image_y), CV_RGB(0,0,250), 10);
  }

  void spin()
  {
    ros::Rate loop_rate(5);
    while (nh.ok()) {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing_image).toImageMsg();
      pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_truth_publisher");
  GroundTruthPublisher gtp(argc, argv);
  gtp.spin();
}
