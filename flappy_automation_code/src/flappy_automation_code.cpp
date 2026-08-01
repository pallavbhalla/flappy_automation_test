#include "ros/ros.h"
#include "flappy_automation_code/flappy_automation_code.hpp"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include <algorithm>

// State machine
enum State { CRUISE, ALIGN, TRANSIT };
State state = CRUISE;

// Global vars
float ax_max = {3.0};
float ay_max = {35.0};
float vx = {0.0}, vy = {0.0};
float err_y_filtered = 0.0;
float last_gap_angle = 0.0; // prior: angular offset of last known gap

// Tunable parameters (loaded from ROS params)
float pipe_detect_dist;
float alpha;
float vx_gain;
float vx_transit;  // boosted x speed through gap
float slow_factor;
float Kp_x;
float Kp_y;
float Kd_y;
float Kd_transit;  // aggressive damping in gap
float gap_threshold; // ray range to detect pipe walls beside bird

void initNode()
{
  //Initialization of nodehandle
  nh_ = new ros::NodeHandle();
  ros::NodeHandle nh_private("~");

  // Load tunable parameters from private namespace (matches YAML loaded via rosparam ns)
  nh_private.param<float>("pipe_detect_dist", pipe_detect_dist, 3.0);
  nh_private.param<float>("alpha", alpha, 0.6273);
  nh_private.param<float>("vx_gain", vx_gain, 0.5728);
  nh_private.param<float>("vx_transit", vx_transit, 2.0);
  nh_private.param<float>("slow_factor", slow_factor, 3.6126);
  nh_private.param<float>("Kp_x", Kp_x, 2.5588);
  nh_private.param<float>("Kp_y", Kp_y, 87.0372);
  nh_private.param<float>("Kd_y", Kd_y, 10.4209);
  nh_private.param<float>("Kd_transit", Kd_transit, 30.0);
  nh_private.param<float>("gap_threshold", gap_threshold, 0.8);

  //Init publishers and subscribers
  pub_acc_cmd = nh_->advertise<geometry_msgs::Vector3>("/flappy_acc",1);
  sub_vel = nh_->subscribe<geometry_msgs::Vector3>("/flappy_vel", 1, velCallback);
  sub_laser_scan = nh_->subscribe<sensor_msgs::LaserScan>("/flappy_laser_scan", 1, laserScanCallback);
}

void velCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // msg has the format of geometry_msgs::Vector3
  // Example of publishing acceleration command on velocity velCallback
  /*
  geometry_msgs::Vector3 acc_cmd;

  acc_cmd.x = 0;
  acc_cmd.y = 0;
  pub_acc_cmd.publish(acc_cmd);
  */
  vx = msg->x;
  vy = msg->y;
  
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // --- Sensor processing (shared across all states) ---
  float range_front = std::min({msg->ranges[3], msg->ranges[4], msg->ranges[5]});

  // Gap direction: range²-weighted angular average
  // Long-range rays dominate, naturally pointing toward the gap
  int num_rays = (int)msg->ranges.size();
  float weighted_angle_sum = 0.0f;
  float weight_sum = 0.0f;
  for (int i = 0; i < num_rays; i++)
  {
    float angle = msg->angle_min + i * msg->angle_increment;
    float w = msg->ranges[i] * msg->ranges[i];
    weighted_angle_sum += w * angle;
    weight_sum += w;
  }

  float err_y_raw = 0.0f;
  if (weight_sum > 0.0f)
  {
    err_y_raw = weighted_angle_sum / weight_sum;
  }

  // Blend with prior from last known gap (helps when far from pipes, all rays max range)
  float prior_weight = (range_front > pipe_detect_dist) ? 0.3f : 0.0f;
  err_y_raw = (1.0f - prior_weight) * err_y_raw + prior_weight * last_gap_angle;

  // Apply EMA filter
  err_y_filtered = alpha * err_y_raw + (1.0f - alpha) * err_y_filtered;
  float err_y = err_y_filtered;

  // Update gap prior when we can clearly see the gap (approaching pipe)
  if (range_front < pipe_detect_dist)
  {
    last_gap_angle = err_y_filtered;
  }

  // --- State transitions ---
  bool wall_above = (msg->ranges[7] < gap_threshold || msg->ranges[8] < gap_threshold);
  bool wall_below = (msg->ranges[0] < gap_threshold || msg->ranges[1] < gap_threshold);

  switch (state)
  {
    case CRUISE:
      if (range_front < pipe_detect_dist)
        state = ALIGN;
      break;

    case ALIGN:
      // Enter TRANSIT when: front is clear (through gap), walls on both sides, aligned and settled
      if (wall_above && wall_below && range_front >= pipe_detect_dist
          && std::abs(err_y) < 0.05f && std::abs(vy) < 0.15f)
        state = TRANSIT;
      else if (range_front >= pipe_detect_dist && !wall_above && !wall_below)
        state = CRUISE;
      break;

    case TRANSIT:
      if (!wall_above && !wall_below)
        state = CRUISE;
      break;
  }

  // --- Control outputs per state ---
  float acc_x = 0.0f;
  float acc_y = 0.0f;

  switch (state)
  {
    case CRUISE:
    {
      // Full speed forward, damp vertical only
      float vx_target = vx_gain * range_front;
      acc_x = Kp_x * (vx_target - vx);
      acc_y = -Kd_y * vy;
      break;
    }

    case ALIGN:
    {
      // Slow down (more if gap is off-center), steer toward gap
      float vx_target = vx_gain * range_front * std::max(0.1f, 1.0f - slow_factor * std::abs(err_y));
      acc_x = Kp_x * (vx_target - vx);
      acc_y = Kp_y * err_y - Kd_y * vy;
      break;
    }

    case TRANSIT:
    {
      // Rush through gap: boost x, aggressively kill vy
      acc_x = Kp_x * (vx_transit - vx);
      acc_y = -Kd_transit * vy;
      break;
    }
  }

  // Clamp and publish
  acc_x = std::clamp(acc_x, -ax_max, ax_max);
  acc_y = std::clamp(acc_y, -ay_max, ay_max);

  const char* state_names[] = {"CRUISE", "ALIGN", "TRANSIT"};
  ROS_INFO("[%s] rf=%.2f err_y=%.3f | acc_x=%.2f acc_y=%.2f | vx=%.2f vy=%.2f",
           state_names[state], range_front, err_y, acc_x, acc_y, vx, vy);

  geometry_msgs::Vector3 acc_cmd;
  acc_cmd.x = acc_x;
  acc_cmd.y = acc_y;
  pub_acc_cmd.publish(acc_cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"flappy_automation_code");
  initNode();

  // Ros spin to prevent program from exiting
  ros::spin();
  return 0;
}
