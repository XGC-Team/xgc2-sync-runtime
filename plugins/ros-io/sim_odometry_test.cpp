#include "sim_odometry.hpp"
#include <cassert>
#include <limits>
int main() {
  xgc_pose_v1 pose{}; pose.stamp=1; pose.position[0]=12; pose.q_wxyz[0]=std::sqrt(0.5); pose.q_wxyz[3]=std::sqrt(0.5);
  xgc_twist_v1 twist{}; twist.stamp=1; twist.linear[0]=2; twist.angular[1]=3;
  SimOdometry result;
  assert(measured_sim_odometry(pose,twist,0,&result));
  assert(result.pose.position[0]==12);
  assert(std::abs(result.linear[0])<1e-12 && std::abs(result.linear[1]+2)<1e-12);
  assert(std::abs(result.angular[0]-3)<1e-12 && std::abs(result.angular[1])<1e-12);
  assert(!measured_sim_odometry(pose,twist,1,&result));
  twist.stamp=1.01; assert(!measured_sim_odometry(pose,twist,0,&result));
  twist.stamp=1; twist.linear[0]=std::numeric_limits<double>::quiet_NaN(); assert(!measured_sim_odometry(pose,twist,0,&result));
  twist.linear[0]=2; pose.q_wxyz[0]=0; assert(!measured_sim_odometry(pose,twist,0,&result));
}
