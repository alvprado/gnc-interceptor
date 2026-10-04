#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

#include "estimation/estimation_node.hpp"

namespace
{

class EstimationNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        rclcpp::NodeOptions observer_options;
        observer_options.use_intra_process_comms(true);
        observer_ = std::make_shared<rclcpp::Node>("estimation_observer", observer_options);
        estimate_sub_ = observer_->create_subscription<gnc_interfaces::msg::TargetEstimate>(
            "target/estimate", 10,
            [this](gnc_interfaces::msg::TargetEstimate::ConstSharedPtr estimate)
            { latest_estimate_ = *estimate; });
        interceptor_pub_ = observer_->create_publisher<gnc_interfaces::msg::InterceptorState>(
            "interceptor/state", 10);
        radar_pub_ = observer_->create_publisher<gnc_interfaces::msg::RadarMeasurement>(
            "radar/measurement", 10);

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        node_ = std::make_shared<gnc_ros::EstimationNode>(options);
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

    static gnc_interfaces::msg::InterceptorState makeInterceptorState()
    {
        gnc_interfaces::msg::InterceptorState state;
        state.attitude.w = 1.0;
        return state;
    }

    static gnc_interfaces::msg::RadarMeasurement makeRadarMeasurement(double range_m)
    {
        gnc_interfaces::msg::RadarMeasurement measurement;
        measurement.range_m = range_m;
        measurement.range_rate_mps = -10.0;
        measurement.azimuth_rad = 0.0;
        measurement.elevation_rad = 0.0;
        return measurement;
    }

    std::optional<gnc_interfaces::msg::TargetEstimate> latest_estimate_;
    rclcpp::Node::SharedPtr observer_;
    std::shared_ptr<gnc_ros::EstimationNode> node_;
    rclcpp::Subscription<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_sub_;
    rclcpp::Publisher<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_pub_;
    rclcpp::Publisher<gnc_interfaces::msg::RadarMeasurement>::SharedPtr radar_pub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(EstimationNodeTest, PublishesNothingBeforeFirstMeasurement)
{
    for (int i = 0; i < 20; ++i)
    {
        interceptor_pub_->publish(makeInterceptorState());
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    EXPECT_FALSE(latest_estimate_.has_value());
}

TEST_F(EstimationNodeTest, PublishesValidEstimateAfterFirstMeasurement)
{
    // The EKF is stateful: wait for discovery first, then publish the one real measurement
    // exactly once. Re-publishing it while waiting would apply repeated dt=0 corrections.
    ASSERT_TRUE(spinUntil([&] {
        return interceptor_pub_->get_subscription_count() > 0 &&
               radar_pub_->get_subscription_count() > 0;
    }));

    interceptor_pub_->publish(makeInterceptorState());
    for (int i = 0; i < 5; ++i)
    {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    radar_pub_->publish(makeRadarMeasurement(500.0));

    ASSERT_TRUE(spinUntil([&] { return latest_estimate_.has_value(); }));

    EXPECT_EQ(latest_estimate_->status, gnc_interfaces::msg::TargetEstimate::VALID);
    EXPECT_NEAR(latest_estimate_->state.position_m.x, 500.0, 1.0e-6);
    EXPECT_TRUE(std::isfinite(latest_estimate_->covariance[0]));
}

}  // namespace
