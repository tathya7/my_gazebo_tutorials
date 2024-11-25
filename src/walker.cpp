/**
 * @file walker.cpp
 * @author Tathya Bhatt
 * @version 0.1
 *
 */

/**
 * @copyright Copyright 2024 Tathya Bhatt

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

/**
 * @brief Base class representing the state of Roomba Robot according to the
 * State-Design Pattern. This class defines the interface for all concrete
 * states
 *
 */
class RoombaState {
 public:
  virtual ~RoombaState() = default;

  /**
   * @brief Update function which utilizes the lase scan msg type to transition
   * between clockwise and anti-clockwise direction
   *
   * @param msg Shared pointer to LaserScan message comming from the topic
   */
  virtual void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;

  /**
   * @brief Getter method which returns the linear velocity for current state
   *
   * @return Linear Velocity
   */
  virtual double getLinearVel() const = 0;

  /**
   * @brief Getter method which will be used for angular velocity
   *
   * @return Angular Velocity
   */
  virtual double getAngularVel() const = 0;

  ///< Linear and Angular Velocity declaration
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  /**
   * @brief Function to get the next state after following the current state
   * based on LiDAR Data
   *
   * @param msg The input laser scan data which will determine if the obstacle
   * is near or not.
   */
  virtual std::shared_ptr<RoombaState> getNextState(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;
};

/**
 * @brief Concrete state representing the rotation behaviour of the robot
 *
 * If the distance to object is less then a threshold, the the robot switches to
 * another state
 */
class RotatingState : public RoombaState {
 public:
  /**
   * @brief Constructor for the RotatingState
   *
   * Initializes the rotation direction based on the toggle rotation flag
   *
   */
  RotatingState() {
    if (toggle_rotation_) {
      angular_vel = -0.5;  ///< Clockwise rotation
    } else {
      angular_vel = 0.5;  ///< Counterclockwise rotation
    }
  }

  /**
   * @brief the virtual method being overidden for the rotation behaviour of the
   * robot
   *
   * If distance between the object and robot is less than 0.7m, the the robot
   * switches to rotation state while keeping the linear velocity constant
   *
   * @param msg LaserScan message comming from the lidar
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
    // Conditional statement to change the velocity behavior
    if (msg->ranges[0] < 0.7) {
      linear_vel = 0.0;
    } else {
      linear_vel = 0.2;
      angular_vel = 0.2;
    }
  }

  /**
   * @brief Changes the next to back to forward state after the rotation is
   * completed
   *
   * @param LaserScan message
   * @return A shared pointer to forward state
   */
  std::shared_ptr<RoombaState> getNextState(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) override;

  /**
   * @brief Getting the linear and angulr velocity by overriding the functions
   * and setting the current velocities for the behavior
   */
  double getLinearVel() const override { return linear_vel; }
  double getAngularVel() const override { return angular_vel; }

 private:
  static bool toggle_rotation_;  ///< Static toggle flag for alternative
                                 ///< rotation directions
};

// Initializing the rotation flag
bool RotatingState::toggle_rotation_ = true;

/**
 * @brief Concrete state representing the forward behavior of the robot
 *
 * Robot moves forward until an obstacle is detected
 */
class ForwardState : public RoombaState {
 public:
  /**
   * @brief Update the velocities for the forward state behavior
   *
   *
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/) override {
    linear_vel = 0.2;
    angular_vel = 0.3;
  }

  /**
   * @brief Determining the next state after the forward direction detectts the
   * objects
   *
   * The robot transitions to rotating state if the obstacle is detected
   *
   * @param msg Uses the laser data to detect the object and returns the shared
   * pointer to
   */
  std::shared_ptr<RoombaState> getNextState(
      const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
    if (msg->ranges[0] < 0.6) {
      return std::make_shared<RotatingState>();
    }
    return nullptr;
  }

  double getLinearVel() const override { return linear_vel; }
  double getAngularVel() const override { return angular_vel; }
};

// Implementing the RotatingState::getNextState method after Forward State is
// finished
std::shared_ptr<RoombaState> RotatingState::getNextState(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (msg->ranges[0] >= 0.7) {
    toggle_rotation_ = !toggle_rotation_;
    return std::make_shared<ForwardState>();  ///< Move to forward state after
                                              ///< rotation state
  }
  return nullptr;
}

/**
 * @brief Roomba class manages the transition between current states and next
 * states
 */
class Roomba {
 public:
  /**
   * @brief Constructor for the class which initializes robot to move forward
   * using ForwardState
   */
  Roomba() : current_state_(std::make_shared<ForwardState>()) {}

  /**
   * @brief Update the current state of robot based on sensor data recieved
   */
  void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    current_state_->update(msg);

    auto next_state = current_state_->getNextState(msg);
    if (next_state) {
      current_state_ =
          next_state;  ///< Transition to the next state from current state
    }
  }

  /**
   * @brief Get the linear and angular velocity from that states
   */
  double getLinearVel() const { return current_state_->getLinearVel(); }
  double getAngularVel() const { return current_state_->getAngularVel(); }

 private:
  std::shared_ptr<RoombaState>
      current_state_;  ///< Initiallizing the member variable for current state
};

/**
 * @brief ROS2 Node that manages the velocity publisher and subscribes to the
 * lidar data
 */
class RoombaNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the node
   *
   * Initializes publisher to "cmd_vel" topic and subscribes to laserscan topic
   */
  RoombaNode() : Node("roomba_node") {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&RoombaNode::laser_callback, this, std::placeholders::_1));
    roomba_ = std::make_shared<Roomba>();
  }

 private:
  /**
   * @brief Callback function which processes the lidar data and updates the
   * linear and angular velocity
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    roomba_->update(msg);

    twist_msg_.linear.x = roomba_->getLinearVel();
    twist_msg_.angular.z = roomba_->getAngularVel();
    vel_pub_->publish(twist_msg_);
  }

  std::shared_ptr<Roomba> roomba_;  ///< Roomba class instance
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      vel_pub_;  ///< Velocity Publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_sub_;  ///< Laser Subscriber
  geometry_msgs::msg::Twist
      twist_msg_;  ///< velocity attribute to publish the message
};

/**
 * @brief Main function to initialize ROS Node loop and start the nodes
 * @return None
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoombaNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
