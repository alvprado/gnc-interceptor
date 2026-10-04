#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include "common/converters.hpp"
#include "simulation/simulation_node.hpp"

namespace
{

class SimulationNodeTest : public ::testing::Test
{
protected:
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
            "/clock", rclcpp::ClockQoS{},
            [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message)
            { samples_[key(message->clock)].clock_received = true; });
        command_pub_ = observer_->create_publisher<gnc_interfaces::msg::GuidanceCommand>(
            "guidance/command", 10);

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        options.parameter_overrides({
            {"use_sim_time", true}, {"simulation.dt_s", 0.01},
            {"target.type", "constant_velocity"},
            {"target.position_m", std::vector<double>{100.0, 20.0, 30.0}},
            {"target.velocity_mps", std::vector<double>{3.0, 0.0, 0.0}},
            {"sensor.range_var", 0.0}, {"sensor.range_rate_var", 0.0},
            {"sensor.azimuth_var", 0.0}, {"sensor.elevation_var", 0.0},
        });
        simulation_ = std::make_shared<gnc_ros::SimulationNode>(options);
        executor_.add_node(observer_);
        executor_.add_node(simulation_);
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

    std::map<std::int64_t, Sample> samples_;
    rclcpp::Node::SharedPtr observer_;
    std::shared_ptr<gnc_ros::SimulationNode> simulation_;
    rclcpp::Subscription<gnc_interfaces::msg::TargetState>::SharedPtr target_sub_;
    rclcpp::Subscription<gnc_interfaces::msg::InterceptorState>::SharedPtr interceptor_sub_;
    rclcpp::Subscription<gnc_interfaces::msg::RadarMeasurement>::SharedPtr radar_sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Publisher<gnc_interfaces::msg::GuidanceCommand>::SharedPtr command_pub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(SimulationNodeTest, PublishesAllSamplesAndClockBeforeFirstCommand)
{
    ASSERT_TRUE(spinUntil([&] { return hasSample(0) && hasSample(10000000) && hasSample(20000000); }));
    for (std::int64_t const stamp : {0LL, 10000000LL, 20000000LL})
    {
        auto const& sample = samples_.at(stamp);
        double const x = 100.0 + 3.0 * static_cast<double>(stamp) / 1.0e9;
        EXPECT_NEAR(sample.target->position_m.x, x, 1.0e-12);
        EXPECT_EQ(sample.target->header.frame_id, "world");
        EXPECT_EQ(sample.interceptor->header.frame_id, "world");
        EXPECT_EQ(sample.radar->header.frame_id, "interceptor_body");
        EXPECT_DOUBLE_EQ(sample.interceptor->position_m.x, 0.0);
        EXPECT_DOUBLE_EQ(sample.interceptor->velocity_mps.x, 0.0);
        EXPECT_DOUBLE_EQ(sample.interceptor->attitude.w, 1.0);
        EXPECT_NEAR(sample.radar->range_m, std::sqrt(x * x + 20.0 * 20.0 + 30.0 * 30.0),
                    radar_tolerance_m);
    }
}

TEST_F(SimulationNodeTest, ReceivedCommandAdvancesPersistentInterceptorState)
{
    ASSERT_TRUE(spinUntil([&] { return hasSample(0) && hasSample(10000000); }));
    gnc_interfaces::msg::GuidanceCommand command;
    command.thrust_n = 50.0;
    command.load_factor = 1.0;
    command_pub_->publish(command);
    ASSERT_TRUE(spinUntil([&]
    {
        for (auto const& [stamp, sample] : samples_)
        {
            if (sample.complete() && sample.interceptor->position_m.x > 0.02) return true;
        }
        return false;
    }));

    simulation::UAVSimulator<simulation::UAV3DofModel, math::RK4Step> const reference{
        simulation::UAV3DofModel{simulation::UAV3DofModelParams{}}, math::RK4Step{}};
    bool checked_step = false;
    for (auto it = samples_.begin(); it != samples_.end(); ++it)
    {
        auto const next = samples_.find(it->first + 10000000);
        if (!it->second.complete() || next == samples_.end() || !next->second.complete() ||
            it->second.interceptor->velocity_mps.x == 0.0) continue;
        auto const expected = reference.step(gnc_ros::fromMsg(*it->second.interceptor),
                                             gnc_ros::fromMsg(command), 0.01);
        auto const actual = gnc_ros::fromMsg(*next->second.interceptor);
        EXPECT_TRUE(actual.cartesian.position_m.isApprox(expected.cartesian.position_m));
        EXPECT_TRUE(actual.cartesian.velocity_mps.isApprox(expected.cartesian.velocity_mps));
        auto const target = gnc_ros::fromMsg(*next->second.target);
        EXPECT_NEAR(next->second.radar->range_m,
                    (target.position_m - actual.cartesian.position_m).norm(), radar_tolerance_m);
        checked_step = true;
    }
    EXPECT_TRUE(checked_step);
}

}  // namespace
