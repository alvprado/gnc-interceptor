#pragma once

namespace simulation
{

template <SimulatableModel Model_T, typename Integrator_T>
UAVSimulator<Model_T, Integrator_T>::UAVSimulator(Model_T const &model,
                                                  Integrator_T const &integrator)
    : model_(model), integrator_(integrator)
{
}

template <SimulatableModel Model_T, typename Integrator_T>
math::CartesianState UAVSimulator<Model_T, Integrator_T>::step(math::CartesianState const &state,
                                                               ControlVec const &control,
                                                               Scalar dt) const noexcept
{
    StateVec const internal_state = model_.fromCartesianState(state);
    ControlVec const clamped_control = model_.clampControl(control);
    StateVec const next_internal_state = integrator_(model_, internal_state, clamped_control, dt);
    StateVec const clamped_internal_state = model_.clampState(next_internal_state);
    return model_.toCartesianState(clamped_internal_state);
}

}  // namespace simulation
