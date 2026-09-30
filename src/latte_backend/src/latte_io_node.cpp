// latte_io_node — 拉花 IO 桥（新建，补齐 aubo_boot 面板契约中引用的
// latte_io_node：DO2=拉花喷嘴/咖啡机阀、DO4=咖啡机）。
// /set_latte_do2 /set_latte_do4（std_srvs/SetBool）→ aubo_msgs/SetIO
// （板载用户 DO pin2/pin4）；/latte_di_status（std_msgs/String，JSON）发布
// 板载 DI 状态供面板指示灯。
#include <aubo_msgs/msg/io_state.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <memory>
#include <sstream>

namespace latte_backend
{

class LatteIoNode : public rclcpp::Node
{
public:
  LatteIoNode()
  : Node("latte_io_node")
  {
    declare_parameter<bool>("io_simulated", false);
    declare_parameter<int>("do2_pin", 2);
    declare_parameter<int>("do4_pin", 4);
    io_simulated_ = get_parameter("io_simulated").as_bool();

    io_client_ = create_client<aubo_msgs::srv::SetIO>("/aubo_io_controller/set_io");
    di_pub_ = create_publisher<std_msgs::msg::String>("/latte_di_status", 10);
    io_state_sub_ = create_subscription<aubo_msgs::msg::IOState>(
      "/aubo_io_controller/io_states", 10,
      [this](aubo_msgs::msg::IOState::SharedPtr msg) {onIoState(msg);});

    const int do2 = static_cast<int>(get_parameter("do2_pin").as_int());
    const int do4 = static_cast<int>(get_parameter("do4_pin").as_int());
    do2_srv_ = create_service<std_srvs::srv::SetBool>(
      "/set_latte_do2",
      [this, do2](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr resp) {
        handleSetDo(do2, "DO2", req, resp);
      }, rclcpp::ServicesQoS());
    do4_srv_ = create_service<std_srvs::srv::SetBool>(
      "/set_latte_do4",
      [this, do4](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr resp) {
        handleSetDo(do4, "DO4", req, resp);
      }, rclcpp::ServicesQoS());

    RCLCPP_INFO(
      get_logger(), "latte_io_node 就绪（DO2=pin%d, DO4=pin%d, io_simulated=%d）",
      do2, do4, io_simulated_ ? 1 : 0);
  }

private:
  void handleSetDo(
    int pin, const char * name,
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr resp)
  {
    if (io_simulated_) {
      resp->success = true;
      resp->message = std::string(name) + " io_simulated 旁路（未下发真实 IO）";
      return;
    }
    if (!io_client_->wait_for_service(std::chrono::seconds(3))) {
      resp->success = false;
      resp->message = "/aubo_io_controller/set_io 不可达（mock 栈无 IO 控制器）";
      return;
    }
    auto io_req = std::make_shared<aubo_msgs::srv::SetIO::Request>();
    io_req->fun = aubo_msgs::srv::SetIO::Request::FUN_SET_ROBOT_BOARD_USER_DO;
    io_req->pin = pin;
    io_req->state = req->data ? 1.0f : 0.0f;
    auto future = io_client_->async_send_request(io_req);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      resp->success = false;
      resp->message = std::string(name) + " SetIO 超时";
      return;
    }
    resp->success = future.get()->success;
    resp->message = std::string(name) + (req->data ? " 已开" : " 已关");
    RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
  }

  void onIoState(const aubo_msgs::msg::IOState::SharedPtr msg)
  {
    std::ostringstream oss;
    oss << "{";
    for (size_t i = 0; i < msg->digital_in_states.size(); ++i) {
      if (i > 0) {oss << ", ";}
      oss << "\"di" << static_cast<int>(msg->digital_in_states[i].pin) << "\": "
          << (msg->digital_in_states[i].state ? "true" : "false");
    }
    oss << "}";
    std_msgs::msg::String out;
    out.data = oss.str();
    di_pub_->publish(out);
  }

  bool io_simulated_;
  rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr io_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr di_pub_;
  rclcpp::Subscription<aubo_msgs::msg::IOState>::SharedPtr io_state_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr do2_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr do4_srv_;
};

}  // namespace latte_backend

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<latte_backend::LatteIoNode>());
  rclcpp::shutdown();
  return 0;
}
