#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
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
        options.parameter_overrides({rclcpp::Parameter("trail.history_points", 3),
                                     rclcpp::Parameter("trail.sample_period_s", 1.0)});
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

    void publishTarget(double time_s, double x, std::string const& frame = "world")
    {
        gnc_interfaces::msg::TargetState target;
        target.header.stamp = rclcpp::Time{static_cast<std::int64_t>(time_s * 1'000'000'000.0)};
        target.header.frame_id = frame;
        target.position_m.x = x;
        latest_.reset();
        target_pub_->publish(target);
        ASSERT_TRUE(spinUntil([&] { return latest_.has_value(); }));
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
    EXPECT_EQ(find(*latest_, "interceptor"), nullptr);
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
    ASSERT_TRUE(spinUntil(
        [&]
        {
            auto const* marker = find(*latest_, "interceptor");
            return marker != nullptr && marker->pose.position.x == 1.0;
        }));

    auto const* target_marker = find(*latest_, "target");
    ASSERT_NE(target_marker, nullptr);
    EXPECT_DOUBLE_EQ(target_marker->pose.position.x, 10.0);
}

TEST_F(VisualizationNodeTest, InterceptorSphereUsesTheReceivedPosition)
{
    gnc_interfaces::msg::InterceptorState interceptor;
    interceptor.header.frame_id = "world";
    interceptor.position_m.x = 12.0;
    interceptor.position_m.z = 34.0;
    interceptor.attitude.z = std::sqrt(0.5);
    interceptor.attitude.w = std::sqrt(0.5);
    interceptor_pub_->publish(interceptor);
    ASSERT_TRUE(spinUntil([&] { return latest_.has_value(); }));
    auto const* marker = find(*latest_, "interceptor");
    ASSERT_NE(marker, nullptr);
    EXPECT_EQ(marker->type, visualization_msgs::msg::Marker::SPHERE);
    EXPECT_EQ(marker->pose.position, interceptor.position_m);
    EXPECT_EQ(marker->pose.orientation, interceptor.attitude);
    EXPECT_DOUBLE_EQ(marker->scale.x, 10.0);
    EXPECT_DOUBLE_EQ(marker->scale.y, 10.0);
    EXPECT_DOUBLE_EQ(marker->scale.z, 10.0);
}

TEST_F(VisualizationNodeTest, TrailSamplesHistoryAndKeepsTheLiveEndpoint)
{
    publishTarget(0.0, 0.0);
    publishTarget(0.25, 1.0);
    publishTarget(0.5, 2.0);
    auto const* trail = find(*latest_, "target_trail");
    ASSERT_NE(trail, nullptr);
    ASSERT_EQ(trail->points.size(), 2U);
    EXPECT_EQ(trail->type, visualization_msgs::msg::Marker::LINE_STRIP);
    EXPECT_EQ(trail->action, visualization_msgs::msg::Marker::ADD);
    EXPECT_DOUBLE_EQ(trail->points.front().x, 0.0);
    EXPECT_DOUBLE_EQ(trail->points.back().x, 2.0);

    publishTarget(1.0, 3.0);
    publishTarget(2.0, 4.0);
    publishTarget(3.0, 5.0);
    trail = find(*latest_, "target_trail");
    ASSERT_NE(trail, nullptr);
    ASSERT_EQ(trail->points.size(), 3U);
    EXPECT_DOUBLE_EQ(trail->points.front().x, 3.0);
    EXPECT_DOUBLE_EQ(trail->points.back().x, 5.0);
    EXPECT_DOUBLE_EQ(trail->pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(trail->pose.orientation.w, 1.0);

    publishTarget(3.5, 6.0);
    trail = find(*latest_, "target_trail");
    ASSERT_NE(trail, nullptr);
    ASSERT_EQ(trail->points.size(), 3U);
    EXPECT_DOUBLE_EQ(trail->points.front().x, 4.0);
    EXPECT_DOUBLE_EQ(trail->points.back().x, 6.0);
}

TEST_F(VisualizationNodeTest, DuplicateTimestampsUpdateTheExistingSample)
{
    publishTarget(0.0, 1.0);
    publishTarget(0.0, 2.0);
    publishTarget(1.0, 3.0);
    auto const* trail = find(*latest_, "target_trail");
    ASSERT_NE(trail, nullptr);
    ASSERT_EQ(trail->points.size(), 2U);
    EXPECT_DOUBLE_EQ(trail->points.front().x, 2.0);
    EXPECT_DOUBLE_EQ(trail->points.back().x, 3.0);
}

TEST_F(VisualizationNodeTest, TimeRewindDeletesTheOldTrail)
{
    publishTarget(10.0, 10.0);
    publishTarget(11.0, 11.0);
    publishTarget(0.0, 100.0);
    auto const* trail = find(*latest_, "target_trail");
    ASSERT_NE(trail, nullptr);
    EXPECT_EQ(trail->action, visualization_msgs::msg::Marker::DELETE);
    ASSERT_EQ(trail->points.size(), 1U);
    EXPECT_DOUBLE_EQ(trail->points.front().x, 100.0);

    publishTarget(1.0, 101.0);
    trail = find(*latest_, "target_trail");
    ASSERT_NE(trail, nullptr);
    EXPECT_EQ(trail->action, visualization_msgs::msg::Marker::ADD);
    ASSERT_EQ(trail->points.size(), 2U);
    EXPECT_DOUBLE_EQ(trail->points.front().x, 100.0);
}

TEST_F(VisualizationNodeTest, FrameChangeClearsHistory)
{
    publishTarget(0.0, 0.0);
    publishTarget(1.0, 1.0);
    publishTarget(2.0, 2.0, "map");
    auto const* trail = find(*latest_, "target_trail");
    ASSERT_NE(trail, nullptr);
    EXPECT_EQ(trail->header.frame_id, "map");
    EXPECT_EQ(trail->action, visualization_msgs::msg::Marker::DELETE);
    ASSERT_EQ(trail->points.size(), 1U);
}

TEST_F(VisualizationNodeTest, HistoriesStayIndependent)
{
    publishTarget(0.0, 10.0);
    publishTarget(1.0, 20.0);
    gnc_interfaces::msg::InterceptorState interceptor;
    interceptor.attitude.w = 1.0;
    interceptor.position_m.x = 100.0;
    interceptor_pub_->publish(interceptor);
    ASSERT_TRUE(spinUntil([&] { return find(*latest_, "interceptor") != nullptr; }));
    interceptor.header.stamp.sec = 1;
    interceptor.position_m.x = 200.0;
    interceptor_pub_->publish(interceptor);
    ASSERT_TRUE(spinUntil([&] { return find(*latest_, "interceptor")->pose.position.x == 200.0; }));
    auto const* target = find(*latest_, "target_trail");
    auto const* ego = find(*latest_, "interceptor_trail");
    ASSERT_NE(target, nullptr);
    ASSERT_NE(ego, nullptr);
    ASSERT_EQ(target->points.size(), 2U);
    ASSERT_EQ(ego->points.size(), 2U);
    EXPECT_DOUBLE_EQ(target->points.front().x, 10.0);
    EXPECT_DOUBLE_EQ(ego->points.front().x, 100.0);
    EXPECT_DOUBLE_EQ(ego->points.back().x, 200.0);
}

}  // namespace
