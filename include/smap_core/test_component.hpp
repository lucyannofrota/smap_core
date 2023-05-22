#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace test
{
    class test_node : public rclcpp::Node
    {
        public:
        test_node(const rclcpp::NodeOptions options)
         : rclcpp::Node("test_node", options)
         {
            RCLCPP_INFO(this->get_logger(), "test_node");
         }
        
        void test2(void);
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(test::test_node)

