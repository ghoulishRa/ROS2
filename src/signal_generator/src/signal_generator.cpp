#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;
using std_msgs::msg::Float32;


class Minimal_Publisher : public rclcpp:: Node{
  public:
    Minimal_Publisher()
    :Node ("minimal_publisher"), count_(0.0){

      publisherSignal_ = this->create_publisher<Float32>("signal", 10); //creating a publisher
      publisherTime_ = this->create_publisher<Float32>("time", 10); //creating a publisher

      timer_ = this->create_wall_timer(
        100ms, std::bind(&Minimal_Publisher::timer_callback, this)
      );
    }
  private:
    void timer_callback(){
      auto msgSignal = Float32();
      auto msgTime = Float32();

      msgTime.data = count_++;
      msgSignal.data = std::sin(count_);

      //LOGGER//

      RCLCPP_INFO (this->get_logger(), "Publishing: '%.2f'", msgTime.data);

      //publisher//

      publisherTime_->publish(msgTime);
      publisherSignal_->publish(msgSignal);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Float32>::SharedPtr publisherSignal_;
    rclcpp::Publisher<Float32>::SharedPtr publisherTime_;
    size_t count_;
};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Minimal_Publisher>());
  rclcpp::shutdown();
  return 0;
}
