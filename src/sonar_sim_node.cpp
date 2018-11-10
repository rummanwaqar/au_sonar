/*
 * Creates a sonar simulator action server
 */

#include <cmath>
#include <limits>

#include <ros/ros.h>
#include <angles/angles.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <au_sonar/SonarAction.h>


class SonarAction {
 public:
  SonarAction(std::string name)
      : as_(nh_, name, false),
        action_name_(name),
        private_nh_("~"),
        pingerA_enable_(true),
        pingerB_enable_(false)
  {
    // load params
    private_nh_.getParam("a_x", pinger_position_[0][0]);
    private_nh_.getParam("a_y", pinger_position_[0][1]);
    private_nh_.getParam("b_x", pinger_position_[1][0]);
    private_nh_.getParam("b_y", pinger_position_[1][1]);
    private_nh_.getParam("processing_time", processing_time_);
    private_nh_.param<std::string>("pose_topic", pose_topic_, "/auri/pose");

    // register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&SonarAction::goalCB, this));
    as_.start();

    pose_sub_ = nh_.subscribe(pose_topic_, 1, &SonarAction::poseCB, this);
    enable_sub_ =
        private_nh_.subscribe("enable", 1, &SonarAction::enableCB, this);

    ROS_INFO("%s: Initialized. PingerA: %.2f,%.2f and PingerB: %.2f,%.2f",
             action_name_.c_str(), pinger_position_[0][0],
             pinger_position_[0][1], pinger_position_[1][0],
             pinger_position_[1][1]);
  }

  ~SonarAction(void) {}

  void goalCB() {
    as_.acceptNewGoal();
    ROS_INFO("%s: Getting sonar ping ...", action_name_.c_str());

    // helper variables
    ros::Rate r(100);
    bool success = true;

    // start executing the action
    for (int i = 0; i <= processing_time_; i += 10)  // every 10 ms
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        success = false;
        break;
      }

      // create fake feedback
      if (i < processing_time_ / 2000) {
        feedback_.status = "calibrating";
      } else {
        feedback_.status = "calculating";
      }

      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 100 Hz for
      // demonstration purposes
      r.sleep();
    }

    if (success) {
      result_.heading = heading_ * 180.0 / M_PI;
      ROS_INFO("%s: heading=%.3f", action_name_.c_str(), result_.heading);
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

  void poseCB(const geometry_msgs::Pose::ConstPtr& msg) {
    // get angle between two points in 2d
    double dx, dy;
    if (pingerA_enable_ && !pingerB_enable_) {
      dx = pinger_position_[0][0] - msg->position.x;
      dy = pinger_position_[0][1] - msg->position.y;
    } else if (pingerB_enable_ && !pingerA_enable_) {
      dx = pinger_position_[1][0] - msg->position.x;
      dy = pinger_position_[1][1] - msg->position.y;
    } else {
      heading_ = std::numeric_limits<double>::quiet_NaN();
      return;
    }

    double pinger_angle = atan2(dy, dx);

    // get yaw of robot (z-axis)
    double siny = +2.0 * (msg->orientation.w * msg->orientation.z +
                          msg->orientation.x * msg->orientation.y);
    double cosy = +1.0 - 2.0 * (msg->orientation.y * msg->orientation.y +
                                msg->orientation.z * msg->orientation.z);
    double yaw = atan2(siny, cosy);

    heading_ = static_cast<float>(
        angles::shortest_angular_distance(pinger_angle, yaw));
    ROS_DEBUG("angle: %.3f", heading_);
  }

  void enableCB(const std_msgs::Int32::ConstPtr& msg) {
    pingerA_enable_ = false;
    pingerB_enable_ = false;
    if (msg->data == 1) {  // PingerA active
      pingerA_enable_ = true;
      ROS_INFO("Sonar: Pinger A active");
    } else if (msg->data == 2) {  // PingerB active
      pingerB_enable_ = true;
      ROS_INFO("Sonar: Pinger B active");
    } else {
      ROS_INFO("Sonar: pingers inactive");
    }
  }

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  actionlib::SimpleActionServer<au_sonar::SonarAction> as_;
  ros::Subscriber pose_sub_;
  ros::Subscriber enable_sub_;
  std::string action_name_;
  au_sonar::SonarFeedback feedback_;
  au_sonar::SonarResult result_;
  std::string pose_topic_;
  float heading_;
  double pinger_position_[2][2];
  double processing_time_;
  bool pingerA_enable_;
  bool pingerB_enable_;
  bool pingerA_not_B_;  // if true pinger A is enabled otherwise pinger B
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sonar");
  SonarAction sonar(ros::this_node::getName());
  ros::spin();

  return 0;
}
