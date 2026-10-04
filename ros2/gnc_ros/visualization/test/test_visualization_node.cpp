#include <gtest/gtest.h>

#include <chrono>
#include <optional>
#include <string>
#include <thread>

#include "visualization/visualization_node.hpp"

namespace
{

class VisualizationNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        rclcpp::NodeOptions observer_options;
        observer_options.use_intra_process_comms(true);
        observer_ = std::make_shared<rclcpp::Node>("visualization_observer", observer_options);
        marker_sub_ = observer_->create_subscription<visualization_msgs::msg::MarkerArray>(
            "visualization/markers", 10,
            [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr markers)
            { latest_ = *markers; });
        target_pub_ = observer_->create_publisher<gnc_interfaces::msg::TargetState>(
            "target/ground_truth", 10);
        interceptor_pub_ = observer_->create_publisher<gnc_interfaces::msg::InterceptorState>(
            "interceptor/state", 10);

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        node_ = std::make_shared<gnc_ros::VisualizationNode>(options);
        executor_.add_node(observer_);
        executor_.add_node(node_);
    }

    template <typename Predicate>
    bool spinUntil(Predicate predicate)
    {
        auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
        while (std::chrono::steady_clock::now() < deadline)
        {
            executor_.spin_some();
            if (predicate()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
        return false;
    }

    static visualization_msgs::msg::Marker const* find(
        visualization_msgs::msg::MarkerArray const& markers, std::string const& ns)
    {
        for (auto const& marker : markers.markers)
        {
            if (marker.ns == ns) return &marker;
        }
        return nullptr;
    }

    std::optional<visualization_msgs::msg::MarkerArray> latest_;
    rclcpp::Node::SharedPtr observer_;
    std::shared_ptr<gnc_ros::VisualizationNode> node_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Publisher<gnc_interfaces::msg::TargetState>::SharedPtr target_pub_;
    rclcpp::Publisher<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_pub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(VisualizationNodeTest, PublishesTargetAndInterceptorMarkersInWorldFrame)
{
    gnc_interfaces::msg::TargetState target;
    target.position_m.x = 10.0;
    target.position_m.y = 20.0;
    target.position_m.z = 30.0;
    target_pub_->publish(target);

    ASSERT_TRUE(spinUntil([&] { return latest_.has_value() && find(*latest_, "target"); }));
    auto const* target_marker = find(*latest_, "target");
    ASSERT_NE(target_marker, nullptr);
    EXPECT_EQ(target_marker->header.frame_id, "world");
    EXPECT_DOUBLE_EQ(target_marker->pose.position.x, 10.0);
    EXPECT_DOUBLE_EQ(target_marker->pose.position.y, 20.0);
    EXPECT_DOUBLE_EQ(target_marker->pose.position.z, 30.0);
}

TEST_F(VisualizationNodeTest, InterceptorUpdateDoesNotDropTargetMarker)
{
    gnc_interfaces::msg::TargetState target;
    target.position_m.x = 10.0;
    target_pub_->publish(target);
    ASSERT_TRUE(spinUntil([&] { return latest_.has_value() && find(*latest_, "target"); }));

    gnc_interfaces::msg::InterceptorState interceptor;
    interceptor.position_m.x = 1.0;
    interceptor.attitude.w = 1.0;
    interceptor_pub_->publish(interceptor);
    ASSERT_TRUE(spinUntil([&]
    {
        auto const* marker = find(*latest_, "interceptor");
        return marker != nullptr && marker->pose.position.x == 1.0;
    }));

    auto const* target_marker = find(*latest_, "target");
    ASSERT_NE(target_marker, nullptr);
    EXPECT_DOUBLE_EQ(target_marker->pose.position.x, 10.0);
}

}  // namespace
