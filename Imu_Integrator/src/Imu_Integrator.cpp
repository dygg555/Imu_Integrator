#include "Imu_Integrator.h"

ImuIntegrator::ImuIntegrator() {
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  velocity = zero;

  firstT = true;

  // Line strip is blue
  // path.color.b = 1.0;
  // path.color.a = 1.0;
  // path.type = visualization_msgs::Marker::LINE_STRIP;
  // path.header.frame_id = "/global";
  // path.ns = "points_and_lines";
  // path.action = visualization_msgs::Marker::ADD;
  // path.pose.orientation.w = 1.0;
  // path.scale.x = 0.2;
  Eigen::Vector3d p;
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  path.push_back(p);
}

void ImuIntegrator::ImuCallback(const ImuMsg &msg) {
  if (firstT) {
    time = msg.time;
    deltaT = 0;
    setGravity(msg.linear_acceleration);
    firstT = false;
  } else {
    // std::chrono::duration duration = (msg.time - time);//s
    deltaT = std::chrono::duration_cast<std::chrono::duration<double>>(msg.time - time).count();
    // spdlog::info("deltaT:{}",deltaT);
    time = msg.time;
    calcOrientation(msg.angular_velocity);
    calcPosition(msg.linear_acceleration);
    updatePath(pose.pos);
    // publishMessage();
  }
  // std::cout << pose.pos << std::endl;
}

void ImuIntegrator::setGravity(const Eigen::Vector3d &msg) {
  gravity[0] = msg[0];
  gravity[1] = msg[1];
  gravity[2] = msg[2];
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  Eigen::Vector3d p;
  p.x() = msg[0];
  p.y() = msg[1];
  p.z() = msg[2];
  path.push_back(p);
}

// void ImuIntegrator::publishMessage() { line_pub.publish(path); }

void ImuIntegrator::calcOrientation(const Eigen::Vector3d &msg) {
  Eigen::Matrix3d B;
  B << 0, -msg.z() * deltaT, msg.y() * deltaT, msg.z() * deltaT, 0, -msg.x() * deltaT,
      -msg.y() * deltaT, msg.x() * deltaT, 0;
  double sigma =
      std::sqrt(std::pow(msg.x(), 2) + std::pow(msg.y(), 2) + std::pow(msg.z(), 2)) *
      deltaT;

  if(std::abs(sigma)< 1e-6)
  {
    pose.orien = pose.orien *
                (Eigen::Matrix3d::Identity() + 0.0 * B +
                  0.0 * B * B);
  }
  else
  {
    pose.orien = pose.orien *
                (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B +
                  ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
  }
  // std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
  // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;

}

void ImuIntegrator::calcPosition(const Eigen::Vector3d &msg) {
  Eigen::Vector3d acc_l(msg.x(), msg.y(), msg.z());
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
  // gravity[2]);
  // spdlog::warn("*********{}--{}--{}",acc_g.x(),acc_g.y(),acc_g.z());
  velocity = velocity + deltaT * (acc_g - gravity);
  // spdlog::warn("+++++++++++++{}--{}--{}",velocity.x(),velocity.y(),velocity.z());
  pose.pos = pose.pos + deltaT * velocity;
}

Eigen::Vector3d ImuIntegrator::getCurrPose()
{
  return pose.pos;
}
std::vector<Eigen::Vector3d> ImuIntegrator::getPath()
{
  return path;
}

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "Imu_Integrator_node");
//   ros::NodeHandle nh;
//   ros::Publisher line =
//       nh.advertise<visualization_msgs::Marker>("Imu_path", 1000);
//   ImuIntegrator *imu_integrator = new ImuIntegrator(line);
//   ros::Subscriber Imu_message = nh.subscribe(
//       "/imu/data", 1000, &ImuIntegrator::ImuCallback, imu_integrator);
//   ros::spin();
// }
