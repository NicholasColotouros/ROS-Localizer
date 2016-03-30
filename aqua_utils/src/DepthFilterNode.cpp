#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <aquacore/StateMsg.h>
#include <dynamic_reconfigure/server.h>
#include <aqua_utils/DepthFilterConfig.h>
#include <aquacore/EmptyBool.h>
#include <boost/thread/mutex.hpp>
#include <list>
#include <algorithm>
#include <cmath>

typedef std::pair<ros::Time, float> EntryType;
typedef dynamic_reconfigure::Server<aqua_utils::DepthFilterConfig> ReconfigureServer;


/**
 * A re-implementation of depth_filter.py
 *
 * NOTE: cannot find easy way to implement waitTillConstantDepth in roscpp,
 *       since this service handler must be isolated from others, so that
 *       node does not block. This might be doable with multi-threaded spinners,
 *       or with service server that has a dedicated handler thread, but
 *       good luck figuring out kinks with the former or the syntax of the latter:
 *
 *       http://www.ros.org/wiki/roscpp/Overview/Callbacks%20and%20Spinning
 */
class DepthFilter {
public:
#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
  bool waitTillConstantDepth(std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res) {
    // WARNING: may likely block subsequent ROS callbacks!
    constant_depth_off_mutex.lock();
    constant_depth_off_mutex.unlock();
    return true;
  };
#endif

  bool hasReachedConstantDepth(aquacore::EmptyBool::Request& req,
      aquacore::EmptyBool::Response& res) {
    res.result = has_reached_constant_depth;
    has_reached_constant_depth = false;
    return true;
  };

  void dyncfgCB(aqua_utils::DepthFilterConfig& config, uint32_t level) {
    min_window_entries = config.min_window_entries;
    window_size_sec = ros::Duration(config.window_size_sec);
    cutoff_sigma_multiplier = config.cutoff_sigma_multiplier;
    N_sigma_cutoff_m = config.N_sigma_cutoff_m;
  };

  void stateCB(const aquacore::StateMsg::ConstPtr& data) {
    state_mutex.lock();
    
    EntryType curr_entry = std::make_pair(data->header.stamp, data->Depth);

    // Remove outdated entries from sliding window
    if (window.size() > 0) {
      std::list<EntryType>::iterator outdated_i = window.end();
      std::list<EntryType>::iterator i = window.begin();
      for (; i != window.end(); i++) {
        // Stop loop when found entry within sliding window
        if (curr_entry.first - i->first <= window_size_sec) {
          outdated_i = i;
          break;
        }
      }
      window.erase(window.begin(), outdated_i);
    }

    // Compute mean and standard deviation of depth values
    window.push_back(curr_entry);
    size_t num_entries = window.size();
    if ((int) num_entries > min_window_entries) {
      double sum = 0.0;
      double sum_sqrd = 0.0;
      for (std::list<EntryType>::iterator v = window.begin(); v != window.end(); v++) {
        sum += v->second;
        sum_sqrd += v->second * v->second;
      }
      double mean_depth = sum/num_entries;
      double stdev_depth = sqrt(sum_sqrd/num_entries - mean_depth*mean_depth);
      constant_depth = (stdev_depth * cutoff_sigma_multiplier < N_sigma_cutoff_m);
      //ROS_INFO("n: %d, u: %.4f, s: %.4f, c: %d", (int) num_entries, mean_depth, stdev_depth, constant_depth);

      // Publish mean depth (a.k.a. filtered depth)
      filtered_depth_msg.data = mean_depth;
      filtered_depth_pub.publish(filtered_depth_msg);

      // Publish flag indicating whether depth is constant or not
      constant_depth_msg.data = constant_depth;
      constant_depth_pub.publish(constant_depth_msg);

#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
      // Update constant-depth-OFF mutex
      if (!has_prev_constant_depth) {
        if (!constant_depth) {
          constant_depth_off_mutex.lock();
        }
      } else if (constant_depth && !prev_constant_depth) {
        constant_depth_off_mutex.unlock();
      } else if (!constant_depth && prev_constant_depth) {
        constant_depth_off_mutex.lock();
      }
      prev_constant_depth = constant_depth;
      has_prev_constant_depth = true;
#endif

      if (constant_depth) {
        has_reached_constant_depth = true;
      }
    } // if (num_entries > min_window_entries)
    
    state_mutex.unlock();
  };
  
  bool resetHandler(std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res) {
    reset();
    return true;
  };
  
  void reset() {
    state_mutex.lock();
    
    constant_depth = false;
#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
    prev_constant_depth = false;
    has_prev_constant_depth = false;
#endif
    has_reached_constant_depth = false;
  
    window.clear();
    
    state_mutex.unlock();
  };

  DepthFilter() : nh("~"),
      constant_depth(false),
#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
      prev_constant_depth(false), has_prev_constant_depth(false),
#endif
      has_reached_constant_depth(false) {
    double _window_size_sec;
    nh.param<int>("min_window_entries", min_window_entries, 4);
    nh.param<double>("window_size_sec", _window_size_sec, 2.0);
    nh.param<double>("cutoff_sigma_multiplier", cutoff_sigma_multiplier, 3);
    nh.param<double>("N_sigma_cutoff_m", N_sigma_cutoff_m, 0.15);
    window_size_sec = ros::Duration(_window_size_sec);

    filtered_depth_pub = nh.advertise<std_msgs::Float32>("/aqua/filtered_depth", 100);
    constant_depth_pub = nh.advertise<std_msgs::Bool>("/aqua/constant_depth_flag", 100);
    constant_depth_query_svc = nh.advertiseService(
        "/aqua/has_reached_constant_depth", &DepthFilter::hasReachedConstantDepth, this);
    state_sub = nh.subscribe("/aqua/state", 100, &DepthFilter::stateCB, this);

    reset_svc = nh.advertiseService("reset", &DepthFilter::resetHandler, this);

#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
    constant_depth_blocking_svc = nh.advertiseService(
        "/aqua/wait_till_constant_depth", &DepthFilter::waitTillConstantDepth, this);
#endif

    dyncfg_server = new ReconfigureServer(dyncfg_mutex, nh);
    dyncfg_server->setCallback(bind(&DepthFilter::dyncfgCB, this, _1, _2));
    
    reset();
  };


protected:
  ros::NodeHandle nh;

  bool constant_depth;
#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
  bool prev_constant_depth;
  bool has_prev_constant_depth;
#endif
  bool has_reached_constant_depth;

  int min_window_entries;
  ros::Duration window_size_sec;
  double cutoff_sigma_multiplier;
  double N_sigma_cutoff_m;

  std::list<EntryType> window;
#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
  boost::mutex constant_depth_off_mutex;
#endif

  boost::mutex state_mutex;

  ros::Publisher filtered_depth_pub;
  ros::Publisher constant_depth_pub;
  ros::ServiceServer reset_svc;
#ifdef ENABLE_WAIT_TILL_CONSTANT_DEPTH
  ros::ServiceServer constant_depth_blocking_svc;
#endif
  ros::ServiceServer constant_depth_query_svc;
  ros::Subscriber state_sub;

  std_msgs::Float32 filtered_depth_msg;
  std_msgs::Bool constant_depth_msg;

  ReconfigureServer* dyncfg_server;
  boost::recursive_mutex dyncfg_mutex;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_filter");
  DepthFilter filter;
  ros::spin();
  return 0;
};
