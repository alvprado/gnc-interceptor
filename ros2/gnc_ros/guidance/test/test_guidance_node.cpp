#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <thread>

#include "common/converters.hpp"
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
        estimate_pub_ =
            observer_->create_publisher<gnc_interfaces::msg::TargetEstimate>("target/estimate", 10);

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
    ASSERT_TRUE(spinUntil(
        [&]
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

TEST_F(GuidanceNodeTest, ExtrapolatesTargetToInterceptorTimestamp)
{
    auto interceptor = makeInterceptorState();
    interceptor.header.stamp.sec = 1;
    interceptor.velocity_mps.x = 100.0;
    auto estimate = makeValidEstimate(0, 500.0);
    estimate.state.position_m.y = 20.0;
    estimate.state.position_m.z = 30.0;
    estimate.state.velocity_mps.y = 3.0;
    estimate.state.acceleration_mps2.y = 2.0;

    auto aligned_target = gnc_ros::fromMsg(estimate.state);
    aligned_target.position_m +=
        aligned_target.velocity_mps + 0.5 * aligned_target.acceleration_mps2;
    aligned_target.velocity_mps += aligned_target.acceleration_mps2;
    auto controller = gnc_ros::makeGuidanceController(gnc_ros::ControllerConfig{});
    auto const expected =
        controller.step(aligned_target, gnc_ros::fromMsg(interceptor).cartesian, 0.1);

    ASSERT_TRUE(spinUntil(
        [&]
        {
            interceptor_pub_->publish(interceptor);
            estimate_pub_->publish(estimate);
            return latest_command_.has_value();
        }));
    EXPECT_NEAR(latest_command_->thrust_n, expected[0], 1.0e-9);
    EXPECT_NEAR(latest_command_->load_factor, expected[1], 1.0e-9);
    EXPECT_NEAR(latest_command_->bank_angle_rad, expected[2], 1.0e-9);
}

TEST_F(GuidanceNodeTest, NewRunResetsPredictiveBoostAndIgnoresOldInputs)
{
    executor_.remove_node(node_);
    node_.reset();
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.parameter_overrides({{"controller.type", "predictive"},
                                 {"controller.boost_phase_switch_speed_mps", 33.33},
                                 {"controller.boost_phase_thrust_n", 30.0}});
    node_ = std::make_shared<gnc_ros::GuidanceNode>(options);
    executor_.add_node(node_);

    auto state = makeInterceptorState();
    auto estimate = makeValidEstimate(1, 400.0);
    estimate.state.position_m.z = 100.0;
    state.header.stamp.sec = 1;
    state.run_id = estimate.state.run_id = 1;
    state.velocity_mps.x = 40.0;
    ASSERT_TRUE(spinUntil(
        [&]
        {
            interceptor_pub_->publish(state);
            estimate_pub_->publish(estimate);
            return latest_command_ && latest_command_->run_id == 1;
        }));
    EXPECT_LT(latest_command_->thrust_n, 30.0);

    state.run_id = estimate.state.run_id = 2;
    state.velocity_mps.x = 0.0;
    ASSERT_TRUE(spinUntil(
        [&]
        {
            estimate_pub_->publish(estimate);
            interceptor_pub_->publish(state);
            return latest_command_->run_id == 2;
        }));
    EXPECT_DOUBLE_EQ(latest_command_->thrust_n, 30.0);
    EXPECT_DOUBLE_EQ(latest_command_->load_factor, 1.0);

    state.run_id = estimate.state.run_id = 1;
    state.velocity_mps.x = 40.0;
    for (int i = 0; i < 20; ++i)
    {
        interceptor_pub_->publish(state);
        estimate_pub_->publish(estimate);
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    EXPECT_EQ(latest_command_->run_id, 2U);
    EXPECT_DOUBLE_EQ(latest_command_->thrust_n, 30.0);
}

}  // namespace
