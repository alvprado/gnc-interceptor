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
typename UAVSimulator<Model_T, Integrator_T>::StateVec UAVSimulator<Model_T, Integrator_T>::step(
    StateVec const &state, ControlVec const &control, Scalar dt) const noexcept
{
    ControlVec const clamped_control = model_.clampControl(control);
    StateVec const next_state = integrator_(model_, state, clamped_control, dt);
    return model_.clampState(next_state);
}

}  // namespace simulation
