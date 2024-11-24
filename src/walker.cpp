#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

// Defining a state interface for the walker which is common to every state

class RoombaState {
    public:

        virtual ~RoombaState() = default;
        
        // Function to update the angular and linear velocity
        virtual void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;

        // virtual double getLinearVel() = 0;
        // virtual double getAngularVel() = 0;

        virtual double getLinearVel() const = 0;
        virtual double getAngularVel() const = 0;
        
        bool was_obstacle = false;
        double linear_vel = 0.0;
        double angular_vel = 0.0;

         virtual std::shared_ptr<RoombaState> getNextState(const sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;

    // private:
        
        // bool RotatingState::toggle_rotation_ = true;

};

// class RotatingState;
// Defining concrete state for Roomba, which are forward and rotations
class ForwardState;

class RotatingState : public RoombaState {
public:
    RotatingState() {
        angular_vel = toggle_rotation_ ? -0.5 : 0.5;
    }

    void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
        if (msg->ranges[0] < 0.7) {
            linear_vel = 0.0;
        } else {
            linear_vel = 0.2;
            angular_vel = 0.0;
        }
    }

    std::shared_ptr<RoombaState> getNextState(const sensor_msgs::msg::LaserScan::SharedPtr msg) override;

    double getLinearVel() const override { return linear_vel; }
    double getAngularVel() const override { return angular_vel; }

private:
    static bool toggle_rotation_;
};

// Initialize static variable
bool RotatingState::toggle_rotation_ = true;

// ForwardState Class
class ForwardState : public RoombaState {
public:
    void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
        linear_vel = 0.2;
        angular_vel = 0.0;
    }

    std::shared_ptr<RoombaState> getNextState(const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
        if (msg->ranges[0] < 0.6) {
            return std::make_shared<RotatingState>();
        }
        return nullptr;
    }

    double getLinearVel() const override { return linear_vel; }
    double getAngularVel() const override { return angular_vel; }
};

// Implement `RotatingState::getNextState` after `ForwardState` is fully defined
std::shared_ptr<RoombaState> RotatingState::getNextState(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (msg->ranges[0] >= 0.7) {
        toggle_rotation_ = !toggle_rotation_;
        return std::make_shared<ForwardState>();
    }
    return nullptr;
}

// Initialize static toggle variable
// bool RotatingState::toggle_rotation_ = true;
// Context Class - Manages the state transitions
class Roomba {
public:
    Roomba() : current_state_(std::make_shared<ForwardState>()) {}

    void update(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        current_state_->update(msg);

        // Get the next state, if any
        auto next_state = current_state_->getNextState(msg);
        if (next_state) {
            current_state_ = next_state; // Transition to the next state
        }
    }

    double getLinearVel() const { return current_state_->getLinearVel(); }
    double getAngularVel() const { return current_state_->getAngularVel(); }

private:
    std::shared_ptr<RoombaState> current_state_;
};

class RoombaNode : public rclcpp::Node {
    public:
        RoombaNode() : Node("roomba_node"){
            vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&RoombaNode::laser_callback, this, std::placeholders::_1));
            roomba_ = std::make_shared<Roomba>();
        }

    private:
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            roomba_->update(msg);

            twist_msg_.linear.x = roomba_->getLinearVel();
            twist_msg_.angular.z = roomba_->getAngularVel();
            vel_pub_->publish(twist_msg_);

        }

    std::shared_ptr<Roomba> roomba_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    geometry_msgs::msg::Twist twist_msg_;


};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoombaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
