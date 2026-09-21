#pragma once

#include <concepts>

#include "math/cartesian_state.hpp"
#include "math/concepts.hpp"

namespace simulation
{

/// @brief A model the simulator can advance: supplies the dynamics, the
/// limits of its own state and control, and a bidirectional mapping between
/// its own state and a model-agnostic Cartesian state.
/// @tparam Model_T The model type to check.
template <typename Model_T>
concept SimulatableModel =
    math::StateTransition<Model_T, typename Model_T::StateVec, typename Model_T::ControlVec> &&
    requires(Model_T const& m, typename Model_T::StateVec const& x,
             typename Model_T::ControlVec const& u, math::CartesianState const& k) {
        {
            m.clampControl(u)
        } -> std::convertible_to<typename Model_T::ControlVec>;
        {
            m.clampState(x)
        } -> std::convertible_to<typename Model_T::StateVec>;
        {
            m.toCartesianState(x)
        } -> std::convertible_to<math::CartesianState>;
        {
            m.fromCartesianState(k)
        } -> std::convertible_to<typename Model_T::StateVec>;
    };

/// @brief Advances a vehicle model in time by repeatedly applying an
/// integration policy.
/// @details Holds the model and the integration scheme by value.
/// @tparam Model_T A callable ẋ = f(x, u) exposing ::StateVec and ::ControlVec.
/// @tparam Integrator_T An integration policy invoked as
/// integrator(model, state, control, dt).
template <SimulatableModel Model_T, typename Integrator_T>
class UAVSimulator
{
public:
    using StateVec = typename Model_T::StateVec;
    using ControlVec = typename Model_T::ControlVec;
    using Scalar = typename StateVec::Scalar;

    /// @brief Construct a simulator from a dynamics model and an integration
    /// scheme.
    /// @param[in] model The model supplying the dynamics ẋ = f(x, u).
    /// @param[in] integrator The integration policy applied at every step.
    UAVSimulator(Model_T const& model, Integrator_T const& integrator);

    /// @brief Advance the state by one timestep.
    /// @param[in] state The Cartesian state at the start of the step.
    /// @param[in] control The control held constant over the step.
    /// @param[in] dt The timestep.
    /// @returns The Cartesian state after the step.
    [[nodiscard]] math::CartesianState step(math::CartesianState const& state,
                                            ControlVec const& control, Scalar dt) const noexcept;

private:
    Model_T model_;
    [[no_unique_address]] Integrator_T integrator_;
};

}  // namespace simulation

#include "simulation/simulator.inl"
