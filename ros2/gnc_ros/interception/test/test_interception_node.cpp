#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "common/converters.hpp"
#include "interception/interception_node.hpp"
#include "simulation/simulation_node.hpp"

namespace
{

class InterceptionNodeTest : public ::testing::Test
{
protected:
    using Intercept = gnc_interfaces::action::Intercept;
    using ClientGoal = rclcpp_action::ClientGoalHandle<Intercept>;
    // The radar retains a 1e-9 standard deviation even with zero configured variance.
    static constexpr double radar_tolerance_m = 1.0e-7;

    struct Sample
    {
        bool clock_received{false};
        std::optional<gnc_interfaces::msg::TargetState> target;
        std::optional<gnc_interfaces::msg::InterceptorState> interceptor;
        std::optional<gnc_interfaces::msg::RadarMeasurement> radar;

        bool complete() const { return clock_received && target && interceptor && radar; }
    };

    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    static std::int64_t key(builtin_interfaces::msg::Time const& stamp)
    {
        return static_cast<std::int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
    }

    void SetUp() override
    {
        rclcpp::NodeOptions observer_options;
        observer_options.use_intra_process_comms(true);
        observer_ = std::make_shared<rclcpp::Node>("simulation_observer", observer_options);
        target_sub_ = observer_->create_subscription<gnc_interfaces::msg::TargetState>(
            "target/ground_truth", 10,
            [this](gnc_interfaces::msg::TargetState::ConstSharedPtr message)
            { samples_[key(message->header.stamp)].target = *message; });
        interceptor_sub_ = observer_->create_subscription<gnc_interfaces::msg::InterceptorState>(
            "interceptor/state", 10,
            [this](gnc_interfaces::msg::InterceptorState::ConstSharedPtr message)
            { samples_[key(message->header.stamp)].interceptor = *message; });
        radar_sub_ = observer_->create_subscription<gnc_interfaces::msg::RadarMeasurement>(
            "radar/measurement", 10,
            [this](gnc_interfaces::msg::RadarMeasurement::ConstSharedPtr message)
            { samples_[key(message->header.stamp)].radar = *message; });
        clock_sub_ = observer_->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::ClockQoS{}, [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message)
            { samples_[key(message->clock)].clock_received = true; });
        command_pub_ = observer_->create_publisher<gnc_interfaces::msg::GuidanceCommand>(
            "guidance/command", 10);
        action_client_ = rclcpp_action::create_client<Intercept>(observer_, "interception/start");
        reset_client_ =
            observer_->create_client<gnc_interfaces::srv::SimulationControl>("simulation/control");

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        options.parameter_overrides({
            {"use_sim_time", true},
            {"simulation.dt_s", 0.01},
            {"target.type", "constant_velocity"},
            {"target.position_m", std::vector<double>{100.0, 20.0, 30.0}},
            {"target.velocity_mps", std::vector<double>{3.0, 0.0, 0.0}},
            {"sensor.range_var", 0.0},
            {"sensor.range_rate_var", 0.0},
            {"sensor.azimuth_var", 0.0},
            {"sensor.elevation_var", 0.0},
        });
        simulation_ = std::make_shared<gnc_ros::SimulationNode>(options);
        executor_.add_node(observer_);
        executor_.add_node(simulation_);
        interception_ = std::make_shared<gnc_ros::InterceptionNode>(observer_options);
        executor_.add_node(interception_);
        estimate_pub_ =
            observer_->create_publisher<gnc_interfaces::msg::TargetEstimate>("target/estimate", 10);
        estimate_timer_ = observer_->create_wall_timer(
            std::chrono::milliseconds{10},
            [this]
            {
                auto state = latestInterceptor();
                if (!publish_estimate_ || !state) return;
                gnc_interfaces::msg::TargetEstimate estimate;
                estimate.state.header = state->header;
                estimate.state.run_id = state->run_id;
                estimate.state.position_m.x = 100.0;
                estimate.state.position_m.y = -40.0;
                estimate.state.position_m.z = 60.0;
                estimate.status = gnc_interfaces::msg::TargetEstimate::VALID;
                estimate_pub_->publish(estimate);
            });
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

    bool hasSample(std::int64_t stamp) const
    {
        auto const it = samples_.find(stamp);
        return it != samples_.end() && it->second.complete();
    }

    ClientGoal::SharedPtr startGoal(double radius = 1.0)
    {
        if (!spinUntil([&] { return action_client_->action_server_is_ready(); })) return nullptr;
        Intercept::Goal goal;
        goal.interception_distance_m = radius;
        rclcpp_action::Client<Intercept>::SendGoalOptions options;
        options.feedback_callback =
            [this](ClientGoal::SharedPtr, std::shared_ptr<Intercept::Feedback const> feedback)
        { feedback_ = *feedback; };
        auto future = action_client_->async_send_goal(goal, options);
        if (!spinUntil(
                [&]
                { return future.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }))
            return nullptr;
        return future.get();
    }

    std::optional<gnc_interfaces::msg::InterceptorState> latestInterceptor() const
    {
        for (auto it = samples_.rbegin(); it != samples_.rend(); ++it)
            if (it->second.interceptor) return it->second.interceptor;
        return std::nullopt;
    }

    bool publish_estimate_{true};
    std::shared_ptr<gnc_ros::InterceptionNode> interception_;
    rclcpp::Publisher<gnc_interfaces::msg::TargetEstimate>::SharedPtr estimate_pub_;
    rclcpp::TimerBase::SharedPtr estimate_timer_;
    std::map<std::int64_t, Sample> samples_;
    rclcpp::Node::SharedPtr observer_;
    std::shared_ptr<gnc_ros::SimulationNode> simulation_;
    rclcpp::Subscription<gnc_interfaces::msg::TargetState>::SharedPtr target_sub_;
    rclcpp::Subscription<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_sub_;
    rclcpp::Subscription<gnc_interfaces::msg::RadarMeasurement>::SharedPtr radar_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Publisher<gnc_interfaces::msg::GuidanceCommand>::SharedPtr command_pub_;
    rclcpp_action::Client<Intercept>::SharedPtr action_client_;
    rclcpp::Client<gnc_interfaces::srv::SimulationControl>::SharedPtr reset_client_;
    std::optional<Intercept::Feedback> feedback_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(InterceptionNodeTest, SuccessReturnsDistanceAndResetsForAnotherGoal)
{
    auto goal = startGoal(200.0);
    ASSERT_NE(goal, nullptr);
    auto result = action_client_->async_get_result(goal);
    ASSERT_TRUE(spinUntil(
        [&] { return result.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }));
    auto const outcome = result.get();
    EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(outcome.result->success);
    EXPECT_NEAR(outcome.result->final_distance_m, std::sqrt(11300.0), 0.2);
    EXPECT_LT(outcome.result->elapsed_time_s, 0.1);
    ASSERT_TRUE(spinUntil([&] { return feedback_.has_value(); }));
    EXPECT_DOUBLE_EQ(feedback_->distance_m, outcome.result->final_distance_m);
    ASSERT_TRUE(spinUntil([&] { return latestInterceptor() && latestInterceptor()->run_id == 3; }));
    EXPECT_DOUBLE_EQ(latestInterceptor()->position_m.x, 0.0);
    EXPECT_DOUBLE_EQ(latestInterceptor()->position_m.z, 0.0);
    auto second = startGoal(200.0);
    ASSERT_NE(second, nullptr);
    auto second_result = action_client_->async_get_result(second);
    ASSERT_TRUE(spinUntil(
        [&]
        { return second_result.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }));
    EXPECT_EQ(second_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(InterceptionNodeTest, CrashAbortsAndClearsPreviousCommands)
{
    auto goal = startGoal();
    ASSERT_NE(goal, nullptr);
    auto result = action_client_->async_get_result(goal);
    gnc_interfaces::msg::GuidanceCommand command;
    command.run_id = 2;
    command.thrust_n = 50.0;
    command.load_factor = 0.0;
    ASSERT_TRUE(spinUntil(
        [&]
        {
            command_pub_->publish(command);
            return result.wait_for(std::chrono::seconds{0}) == std::future_status::ready;
        }));
    auto const outcome = result.get();
    EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_FALSE(outcome.result->success);
    EXPECT_GT(outcome.result->elapsed_time_s, 0.0);
    ASSERT_TRUE(spinUntil([&] { return feedback_ && feedback_->interceptor_altitude_m < 0.0; }));
    ASSERT_TRUE(spinUntil([&] { return latestInterceptor() && latestInterceptor()->run_id == 3; }));
    EXPECT_DOUBLE_EQ(latestInterceptor()->position_m.z, 0.0);

    auto second = startGoal();
    ASSERT_NE(second, nullptr);
    ASSERT_TRUE(spinUntil(
        [&]
        {
            command_pub_->publish(command);
            return feedback_ && feedback_->elapsed_time_s > 0.2 && latestInterceptor()->run_id == 5;
        }));
    EXPECT_DOUBLE_EQ(latestInterceptor()->velocity_mps.x, 0.0);
    EXPECT_DOUBLE_EQ(latestInterceptor()->position_m.z, 0.0);
}

TEST_F(InterceptionNodeTest, RejectsConcurrentGoalsAndSupportsCancellation)
{
    auto goal = startGoal();
    ASSERT_NE(goal, nullptr);
    EXPECT_EQ(startGoal(), nullptr);
    auto result = action_client_->async_get_result(goal);
    ASSERT_TRUE(spinUntil([&] { return latestInterceptor() && latestInterceptor()->run_id == 2; }));
    (void)action_client_->async_cancel_goal(goal);
    ASSERT_TRUE(spinUntil(
        [&] { return result.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    ASSERT_TRUE(spinUntil([&] { return latestInterceptor() && latestInterceptor()->run_id == 3; }));
    EXPECT_DOUBLE_EQ(latestInterceptor()->position_m.x, 0.0);
}

TEST_F(InterceptionNodeTest, RejectsInvalidInterceptionDistances)
{
    for (double radius : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()})
    {
        EXPECT_EQ(startGoal(radius), nullptr);
    }
}

TEST_F(InterceptionNodeTest, ResetServiceAbortsActiveGoal)
{
    auto goal = startGoal();
    ASSERT_NE(goal, nullptr);
    auto result = action_client_->async_get_result(goal);
    ASSERT_TRUE(spinUntil([&] { return reset_client_->service_is_ready(); }));
    ASSERT_TRUE(spinUntil([&] { return latestInterceptor() && latestInterceptor()->run_id == 2; }));
    auto request = std::make_shared<gnc_interfaces::srv::SimulationControl::Request>();
    request->command = gnc_interfaces::srv::SimulationControl::Request::RESET;
    auto reset = reset_client_->async_send_request(request);
    ASSERT_TRUE(spinUntil(
        [&] { return reset.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }));
    EXPECT_TRUE(reset.get()->success);
    ASSERT_TRUE(spinUntil(
        [&] { return result.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(InterceptionNodeTest, WaitsForEstimateBeforeLaunchingAndCanCancelPreparation)
{
    publish_estimate_ = false;
    auto goal = startGoal();
    ASSERT_NE(goal, nullptr);
    auto result = action_client_->async_get_result(goal);
    ASSERT_TRUE(spinUntil(
        [&] {
            return latestInterceptor() && latestInterceptor()->run_id == 1 && samples_.size() > 20;
        }));
    EXPECT_DOUBLE_EQ(latestInterceptor()->position_m.z, 0.0);
    EXPECT_DOUBLE_EQ(latestInterceptor()->attitude.w, 1.0);
    (void)action_client_->async_cancel_goal(goal);
    ASSERT_TRUE(spinUntil(
        [&] { return result.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(InterceptionNodeTest, LaunchOrientationUsesEstimatedPosition)
{
    auto goal = startGoal();
    ASSERT_NE(goal, nullptr);
    ASSERT_TRUE(spinUntil([&] { return latestInterceptor() && latestInterceptor()->run_id == 2; }));
    auto const vehicle = gnc_ros::fromMsg(*latestInterceptor());
    Eigen::Vector3d const expected = Eigen::Vector3d{100.0, -40.0, 60.0}.normalized();
    EXPECT_TRUE((vehicle.attitude * Eigen::Vector3d::UnitX()).isApprox(expected));
    EXPECT_TRUE(vehicle.cartesian.velocity_mps.isZero());
}

TEST_F(InterceptionNodeTest, RequiresMatchedTimestampsAndRunIdsAndPrioritizesCrash)
{
    auto goal = startGoal();
    ASSERT_NE(goal, nullptr);
    auto result = action_client_->async_get_result(goal);
    ASSERT_TRUE(spinUntil([&] { return feedback_ && latestInterceptor()->run_id == 2; }));
    executor_.remove_node(simulation_);
    auto ego_pub =
        observer_->create_publisher<gnc_interfaces::msg::InterceptorState>("interceptor/state", 10);
    auto target_pub =
        observer_->create_publisher<gnc_interfaces::msg::TargetState>("target/ground_truth", 10);
    auto ego = *latestInterceptor();
    ego.header.stamp.sec += 1;
    ego.position_m.z = -1.0;
    gnc_interfaces::msg::TargetState target;
    target.header = ego.header;
    target.run_id = ego.run_id;
    target.position_m = ego.position_m;
    target.header.stamp.sec += 1;
    ego_pub->publish(ego);
    target_pub->publish(target);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{100};
    ASSERT_TRUE(spinUntil([&] { return std::chrono::steady_clock::now() >= deadline; }));
    EXPECT_EQ(result.wait_for(std::chrono::seconds{0}), std::future_status::timeout);
    EXPECT_GE(feedback_->interceptor_altitude_m, 0.0);

    target.header = ego.header;
    target.run_id = ego.run_id - 1;
    target_pub->publish(target);
    deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{100};
    ASSERT_TRUE(spinUntil([&] { return std::chrono::steady_clock::now() >= deadline; }));
    EXPECT_GE(feedback_->interceptor_altitude_m, 0.0);

    ego.header.stamp.sec += 2;
    target.header = ego.header;
    target.run_id = ego.run_id;
    ego_pub->publish(ego);
    target_pub->publish(target);
    ASSERT_TRUE(spinUntil([&] { return feedback_->interceptor_altitude_m < 0.0; }));
    executor_.add_node(simulation_);
    ASSERT_TRUE(spinUntil(
        [&] { return result.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }));
    auto const outcome = result.get();
    EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_DOUBLE_EQ(outcome.result->final_distance_m, 0.0);
}

}  // namespace
