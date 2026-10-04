#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

#include "guidance/guidance_node.hpp"

namespace
{

class GuidanceNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        rclcpp::NodeOptions observer_options;
        observer_options.use_intra_process_comms(true);
        observer_ = std::make_shared<rclcpp::Node>("guidance_observer", observer_options);
        command_sub_ = observer_->create_subscription<gnc_interfaces::msg::GuidanceCommand>(
            "guidance/command", 10,
            [this](gnc_interfaces::msg::GuidanceCommand::ConstSharedPtr command)
            { latest_command_ = *command; });
        interceptor_pub_ = observer_->create_publisher<gnc_interfaces::msg::InterceptorState>(
            "interceptor/state", 10);
        estimate_pub_ = observer_->create_publisher<gnc_interfaces::msg::TargetEstimate>(
            "target/estimate", 10);

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        node_ = std::make_shared<gnc_ros::GuidanceNode>(options);
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

    static gnc_interfaces::msg::TargetEstimate makeValidEstimate(std::int32_t stamp_sec, double x)
    {
        gnc_interfaces::msg::TargetEstimate estimate;
        estimate.status = gnc_interfaces::msg::TargetEstimate::VALID;
        estimate.state.header.stamp.sec = stamp_sec;
        estimate.state.position_m.x = x;
        estimate.state.velocity_mps.x = -10.0;
        return estimate;
    }

    std::optional<gnc_interfaces::msg::GuidanceCommand> latest_command_;
    rclcpp::Node::SharedPtr observer_;
    std::shared_ptr<gnc_ros::GuidanceNode> node_;
    rclcpp::Subscription<gnc_interfaces::msg::GuidanceCommand>::SharedPtr command_sub_;
    rclcpp::Publisher<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_pub_;
    rclcpp::Publisher<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_pub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(GuidanceNodeTest, PublishesCommandOnceBothInputsAreAvailable)
{
    // No interceptor state cached yet: an estimate alone must not trigger a command, no matter
    // how many times it's (re-)delivered while intra-process discovery settles.
    for (int i = 0; i < 20; ++i)
    {
        estimate_pub_->publish(makeValidEstimate(0, 500.0));
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    EXPECT_FALSE(latest_command_.has_value());

    interceptor_pub_->publish(makeInterceptorState());
    // Re-publish each iteration: a one-shot publish can race intra-process subscription matching.
    ASSERT_TRUE(spinUntil([&]
    {
        estimate_pub_->publish(makeValidEstimate(1, 490.0));
        return latest_command_.has_value();
    }));
    EXPECT_TRUE(std::isfinite(latest_command_->thrust_n));
    EXPECT_TRUE(std::isfinite(latest_command_->load_factor));
    EXPECT_TRUE(std::isfinite(latest_command_->bank_angle_rad));
}

TEST_F(GuidanceNodeTest, IgnoresNonValidEstimates)
{
    for (int i = 0; i < 20; ++i)
    {
        interceptor_pub_->publish(makeInterceptorState());
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }

    gnc_interfaces::msg::TargetEstimate stale;
    stale.status = gnc_interfaces::msg::TargetEstimate::STALE_TIMESTAMP;
    stale.state.position_m.x = 500.0;
    for (int i = 0; i < 20; ++i)
    {
        estimate_pub_->publish(stale);
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    EXPECT_FALSE(latest_command_.has_value());
}

}  // namespace
