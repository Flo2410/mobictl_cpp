#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "MobiCtl.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto mobi_node = MobiCtl::mobictl();

  rclcpp::on_shutdown([mobi_node]() {
    mobi_node->shutdown();
  });

  mobi_node->setup();

  rclcpp::spin(mobi_node);
  rclcpp::shutdown();
  return 0;
}