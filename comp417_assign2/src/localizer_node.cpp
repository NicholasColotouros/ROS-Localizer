#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#define METRE_TO_PIXEL_SCALE 50
#define FORWARD_SWIM_SPEED_SCALING 0.1
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0

// Class Localizer is a sample stub that you can build upon for your implementation
// (advised but optional: starting from scratch is also fine)
//
class Localizer
{
public:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::Subscriber gt_img_sub;
  image_transport::Subscriber robot_img_sub;

  ros::Subscriber motion_command_sub;

  geometry_msgs::PoseStamped estimated_location;

  cv::Mat map_image;
  cv::Mat ground_truth_image;
  cv::Mat localization_result_image;

  Localizer( int argc, char** argv )
  {
    image_transport::ImageTransport it(nh);
    pub = it.advertise("/assign2/localization_result_image", 1);
    map_image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    estimated_location.pose.position.x = 0;
    estimated_location.pose.position.y = 0;

    localization_result_image = map_image.clone();
    ground_truth_image = map_image.clone();

    gt_img_sub = it.subscribe("/assign2/ground_truth_image", 1, &Localizer::groundTruthImageCallback, this);
    robot_img_sub = it.subscribe("/aqua/back_down/image_raw", 1, &Localizer::robotImageCallback, this);
    motion_command_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aqua/target_pose", 1, &Localizer::motionCommandCallback, this);

    ROS_INFO( "localizer node constructed and subscribed." );

  }

  void robotImageCallback( const sensor_msgs::ImageConstPtr& robot_img )
  {
    // TODO: You must fill in the code here to implement an observation model for your localizer
  }

  // Function motionCommandCallback is a example of how to work with Aqua's motion commands (your view on the odometry).
  // The initial version simply integrates the commands over time to make a very rough location estimate.
  // TODO: You must improve it to work with the localizer you implement.
  //
  // Note the somewhat unique meaning of fields in motion_command
  //    motion_command
  //      pose
  //        position
  //          x - requested forward swim effort in a unitless number ranging from 0.0 to 1.0. You must translate this into
  //              a velocity estimate in some way. Currently a simple constant is used.
  //          y - requested up/down swim effort. Not used in this assignment
  //          z - unused
  //        orientation - A quaternion that represents the desired body orientation w.r.t. global frame. Note that
  //                      Aqua's controller will not achieve this orientation perfectly, so you must consider that
  //                      this is only a noisy estimate of the robot's orientation (correct for it with your filter!)
  //
  // Note that we use negative angles because the geometry of the map image is formed with its Z axis pointing downwards
  // (the camera is looking at the ocean floor)
  //
  void motionCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& motion_command )
  {

    geometry_msgs::PoseStamped command = *motion_command;
    double target_roll, target_pitch, target_yaw;
    tf::Quaternion target_orientation;
    tf::quaternionMsgToTF(command.pose.orientation, target_orientation);
    tf::Matrix3x3(target_orientation).getEulerYPR( target_yaw, target_pitch, target_roll );

    // The following three lines implement the basic motion model example
    estimated_location.pose.position.x = estimated_location.pose.position.x + FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * cos( -target_yaw );
    estimated_location.pose.position.y = estimated_location.pose.position.y + FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * sin( -target_yaw );
    estimated_location.pose.orientation = command.pose.orientation;

    // The remainder of this function is sample drawing code to plot your answer on the map image.

    // Comment the one following line to plot your whole trajectory without ground truth
    localization_result_image = ground_truth_image.clone();

    int estimated_robo_image_x = localization_result_image.size().width/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.x;
    int estimated_robo_image_y = localization_result_image.size().height/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.y;

    int estimated_heading_image_x = estimated_robo_image_x + HEADING_GRAPHIC_LENGTH * cos(-target_yaw);
    int estimated_heading_image_y = estimated_robo_image_y + HEADING_GRAPHIC_LENGTH * sin(-target_yaw);

    cv::circle( localization_result_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), POSITION_GRAPHIC_RADIUS, CV_RGB(250,0,0), -1);
    cv::line( localization_result_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), cv::Point(estimated_heading_image_x, estimated_heading_image_y), CV_RGB(250,0,0), 10);
  }

  // This is a provided convenience function that allows you to compare your localization result to a ground truth path
  void groundTruthImageCallback( const sensor_msgs::ImageConstPtr& gt_img )
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(gt_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ground_truth_image = cv::Mat(cv_ptr->image);
  }

  // This function publishes your localization result image and spins ROS to execute its callbacks
  void spin()
  {
    ros::Rate loop_rate(30);
    while (nh.ok()) {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", localization_result_image).toImageMsg();
      pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizer");
  Localizer my_loc(argc, argv);
  my_loc.spin();
}
