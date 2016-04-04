#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <stdlib.h>

// Provided constants
#define METRE_TO_PIXEL_SCALE 50
#define FORWARD_SWIM_SPEED_SCALING 0.1
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0

// My constants
#define NUM_PARTICLES 25000
#define RGB_DISTANCE 1

// Class Localizer is a sample stub that you can build upon for your implementation
// (advised but optional: starting from scratch is also fine)
//
class Localizer
{
public:
  struct Particle
  {
    int x; // pixel measurements
    int y;
    double theta;
    double weight;
  };
  std::vector<Particle> particles;

  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::Subscriber gt_img_sub;
  image_transport::Subscriber robot_img_sub;

  ros::Subscriber motion_command_sub;
  geometry_msgs::PoseStamped estimated_location;

  cv::Mat map_image;
  cv::Mat current_camera_image;
  cv::Mat ground_truth_image;
  cv::Mat localization_result_image;
  cv::Mat localization_line_image; // Used so that we can refresh the particles each frame



  Localizer( int argc, char** argv )
  {
    image_transport::ImageTransport it(nh);
    pub = it.advertise("/assign2/localization_result_image", 1);
    map_image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    // Initialize images
    localization_result_image = map_image.clone();
    localization_line_image = map_image.clone();
    ground_truth_image = map_image.clone();

    // position
    estimated_location.pose.position.x = 0;
    estimated_location.pose.position.y = 0;

    for(int i = 0; i < NUM_PARTICLES; i++)
    {
      particles.push_back(Particle());
      particles[i].x = rand()%map_image.cols;
      particles[i].y = rand()%map_image.rows;
    }

    gt_img_sub = it.subscribe("/assign2/ground_truth_image", 1, &Localizer::groundTruthImageCallback, this);
    robot_img_sub = it.subscribe("/aqua/back_down/image_raw", 1, &Localizer::robotImageCallback, this);
    motion_command_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aqua/target_pose", 1, &Localizer::motionCommandCallback, this);

    ROS_INFO("localizer node constructed and subscribed.");
  }

  // Draws a point in green with radius 5
  void draw_point(int x, int y)
  {
    double radius = 5.0;
    cv::circle(localization_result_image, cv::Point(x, y), radius, CV_RGB(0,250,0), -1);
  }

  // duplicates the line image and draws the particles on the published image
  void draw_particles()
  {
    localization_result_image = localization_line_image.clone();
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
      draw_point(particles[i].x, particles[i].y);
    }
  }

  // Returns the x pixel coordinate adjusted for the length of the camera according to the provided yaw
  // x_pixels: the x location of the camera center, in pixels
  // returns: the x location of the center of the robot in pixels
  int adjust_x_meters(double x_pixels, double yaw)
  {
    return x_pixels + std::roundl(METRE_TO_PIXEL_SCALE * cos( yaw ) * -0.32);
  }

  // Returns the y pixel coordinate adjusted for the length of the camera according to the provided yaw
  // y_pixels: the y location of the camera center, in pixels
  // returns: the y location of the center of the robot in pixels
  int adjust_y_meters(double y_pixels, double yaw)
  {
    return y_pixels + std::roundl(METRE_TO_PIXEL_SCALE * sin( -yaw ) * -0.32);
  }

  // Compare two pixels by returning the distance from the rgb
  double comparePixels( const cv::Vec3b A, const cv::Vec3b B )
  {
    double ret = pow(A[0]-B[0], 2) + pow(A[1]-B[1], 2) + pow(A[2]-B[2], 2);
    return ret;
  }

  void robotImageCallback( const sensor_msgs::ImageConstPtr& robot_img )
  {
    // TODO: You must fill in the code here to implement an observation model for your localizer
    //ROS_INFO( "Got image callback." );
    draw_particles();
    // localization_result_image = localization_line_image.clone();
    //
    // current_camera_image = cv_bridge::toCvShare(robot_img, "bgr8")->image;
    // int rows = current_camera_image.rows;
    // int cols = current_camera_image.cols;
    // cv::Vec3b centerPixelRobo = current_camera_image.at<cv::Vec3b>(rows/2,cols/2);
    //
    // // Find all "close enough" points
    //  for(int x = 0; x < map_image.cols; x++)
    //  {
    //    for(int y = 0; y < map_image.rows; y++)
    //    {
    //      cv::Vec3b currentPixelMap = map_image.at<cv::Vec3b>(y,x); // Image matrix -- use y then x
    //      if(comparePixels(currentPixelMap, centerPixelRobo) <= RGB_DISTANCE)
    //      {
    //        draw_point(x,y);
    //      }
    //    }
    //  }
  }

  void motionCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& motion_command )
  {
    //ROS_INFO( "Got motion callback." );
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

    /*********************************************************/
    // DRAW THE GROUND (BLUE) TRUTH AND OUR ESTIMATED (RED)  //
    /*********************************************************/
    localization_line_image = ground_truth_image.clone(); // you either get a red line or a red point + bluie line. no superimposed paths

    int estimated_robo_image_x = localization_line_image.size().width/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.x;
    int estimated_robo_image_y = localization_line_image.size().height/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.y;

    int estimated_heading_image_x = estimated_robo_image_x + HEADING_GRAPHIC_LENGTH * cos(-target_yaw);
    int estimated_heading_image_y = estimated_robo_image_y + HEADING_GRAPHIC_LENGTH * sin(-target_yaw);

    // ROS_INFO( "Ground truth image point at %d, %d", estimated_robo_image_x, estimated_robo_image_y);
    cv::circle( localization_line_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), POSITION_GRAPHIC_RADIUS, CV_RGB(250,0,0), -1);
    cv::line( localization_line_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), cv::Point(estimated_heading_image_x, estimated_heading_image_y), CV_RGB(250,0,0), 10);
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
