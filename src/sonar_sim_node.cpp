/*
 * Creates a sonar simulator action server
 */

#include <cmath>
#include <limits>

#include <actionlib/server/simple_action_server.h>
#include <angles/angles.h>
#include <ros/ros.h>

#include <au_core/DynamicsState.h>
#include <au_sonar/SonarAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <random>

class Sonar {
 public:
  Sonar()
      : abs_heading_(std::numeric_limits<double>::quiet_NaN()),
        rel_heading_(std::numeric_limits<double>::quiet_NaN()),
        pingerA_enable_(true),
        pingerB_enable_(false) {}

  virtual ~Sonar() = default;

  void init(double pinger_position[2][2], double noise) {
    noise_ = noise;
    // init pinger position
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        pinger_position_[i][j] = pinger_position[i][j];
      }
    }
  }

  void calculateHeading(const geometry_msgs::Pose& pose) {
    if (pingerA_enable_) {
      abs_heading_ = computePingerAngle(pose.position.x, pose.position.y,
                                        pinger_position_[0]);
    } else if (pingerB_enable_) {
      abs_heading_ = computePingerAngle(pose.position.x, pose.position.y,
                                        pinger_position_[1]);
    } else {
      abs_heading_ = std::numeric_limits<double>::quiet_NaN();
      rel_heading_ = std::numeric_limits<double>::quiet_NaN();
      return;
    }

    double robot_yaw = extractYaw(pose.orientation);
    rel_heading_ = -1.f * static_cast<float>(angles::shortest_angular_distance(
                              abs_heading_, robot_yaw));
  }

  std::string enablePinger(int val) {
    pingerA_enable_ = false;
    pingerB_enable_ = false;
    if (val == 1) {  // PingerA active
      pingerA_enable_ = true;
      return std::string("Pinger A active");
    } else if (val == 2) {  // PingerB active
      pingerB_enable_ = true;
      return std::string("Pinger B active");
    } else {
      return std::string("Pingers inactive");
    }
  }

  double get_abs_heading() { return abs_heading_; }

  double get_rel_heading() { return rel_heading_; }

 private:
  double extractYaw(const geometry_msgs::Quaternion& orientation) {
    double siny =
        +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    double cosy = +1.0 - 2.0 * (orientation.y * orientation.y +
                                orientation.z * orientation.z);
    return atan2(siny, cosy);
  }

  double computePingerAngle(double x, double y, const double pinger_pos[2]) {
    double dx = pinger_pos[0] - x;
    double dy = pinger_pos[1] - y;
    return atan2(dy, dx) + getNoise();
  }

  double getNoise() {
    if (noise_ > 0) {
      static std::default_random_engine generator;
      static std::uniform_real_distribution<double> dist(-noise_, noise_);
      return dist(generator);
    }
    return 0.0;
  }

  double abs_heading_;
  double rel_heading_;
  double pinger_position_[2][2];
  bool pingerA_enable_;
  bool pingerB_enable_;
  double noise_;
};

typedef struct {
  std::string pose_topic;
  double pinger_position[2][2];
  double processing_time;
  double noise;
} params_t;

class SonarAction : public Sonar {
 public:
  SonarAction(std::string name)
      : as_(nh_, name, false),
        action_name_(name),
        private_nh_("~"),
        pose_init_(false),
        Sonar() {
    loadParams();

    // initialize sonar
    init(ros_params_.pinger_position, ros_params_.noise);

    // register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&SonarAction::goalCB, this));
    as_.start();

    pose_sub_ =
        nh_.subscribe(ros_params_.pose_topic, 1, &SonarAction::poseCB, this);
    enable_sub_ =
        private_nh_.subscribe("enable", 1, &SonarAction::enableCB, this);

    ROS_INFO(
        "%s: Initialized with noise +/-%.3frad. PingerA: %.2f,%.2f and "
        "PingerB: %.2f,%.2f",
        action_name_.c_str(), ros_params_.noise,
        ros_params_.pinger_position[0][0], ros_params_.pinger_position[0][1],
        ros_params_.pinger_position[1][0], ros_params_.pinger_position[1][1]);
  }

 private:
  void loadParams() {
    private_nh_.getParam("a_x", ros_params_.pinger_position[0][0]);
    private_nh_.getParam("a_y", ros_params_.pinger_position[0][1]);
    private_nh_.getParam("b_x", ros_params_.pinger_position[1][0]);
    private_nh_.getParam("b_y", ros_params_.pinger_position[1][1]);
    private_nh_.getParam("processing_time", ros_params_.processing_time);
    private_nh_.param<std::string>("pose_topic", ros_params_.pose_topic,
                                   "/dynamics/state");
    private_nh_.param<double>("noise", ros_params_.noise, 0.0);
    ros_params_.noise *= M_PI / 180.0;  // convert noise to radians
  }

  void goalCB() {
    as_.acceptNewGoal();
    ROS_INFO("%s: Getting sonar ping ...", action_name_.c_str());

    // helper variables
    ros::Rate r(100);

    if (not pose_init_) {  // no pose messages yet
      ROS_WARN("%s: No pose messages detected. Check if %s is active.",
               action_name_.c_str(), ros_params_.pose_topic.c_str());
      feedback_.status = "No pose messages detected. Failed to compute.";
      as_.publishFeedback(feedback_);
      as_.setAborted();
      return;
    }

    // fake processing wait
    for (int i = 0; i <= ros_params_.processing_time; i += 10) {  // every 10ms
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        return;
      }

      // publish fake feedback
      if (i < ros_params_.processing_time / 2000) {
        feedback_.status = "calibrating";
      } else {
        feedback_.status = "calculating";
      }
      as_.publishFeedback(feedback_);

      r.sleep();
    }

    // processing complete. publish result
    feedback_.status = "finished";
    as_.publishFeedback(feedback_);
    result_.rel_heading = get_rel_heading() * 180.0 / M_PI;
    result_.abs_heading = get_abs_heading() * 180.0 / M_PI;
    ROS_INFO("%s: rel_heading=%.3f\tabs_heading=%.3f", action_name_.c_str(),
             result_.rel_heading, result_.abs_heading);
    as_.setSucceeded(result_);
  }

  void poseCB(const au_core::DynamicsState::ConstPtr& msg) {
    // set to true first time this cb is called
    if (not pose_init_) {
      pose_init_ = true;
    }
    calculateHeading(msg->pose);
    ROS_DEBUG("rel_heading: %.3frad", get_rel_heading());
  }

  void enableCB(const std_msgs::Int32::ConstPtr& msg) {
    std::string info_str = enablePinger(msg->data);
    ROS_INFO_STREAM(action_name_ << ": " << info_str);
  }

  // node handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  // pub/sub
  ros::Subscriber pose_sub_;
  ros::Subscriber enable_sub_;
  // action server stuff
  std::string action_name_;
  actionlib::SimpleActionServer<au_sonar::SonarAction> as_;
  au_sonar::SonarFeedback feedback_;
  au_sonar::SonarResult result_;
  // ros params
  params_t ros_params_;
  // has pose cb been called once?
  bool pose_init_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sonar");
  SonarAction sonar(ros::this_node::getName());
  ros::spin();

  return 0;
}
