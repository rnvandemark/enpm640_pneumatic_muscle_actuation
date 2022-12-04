#pragma once

#include "pma_hardware/description/pneumatic_muscle_segment_configuration.hpp"
#include "pma_hardware/description/pneumatic_muscle_segment_telemetry.hpp"

namespace pma_hardware
{
void compute_individual_force_contribution(
    const IndividualPneumaticMuscleConfiguration& pm_configuration,
    const IndividualPneumaticMuscleTelemetry& pm_telemetry,
    double& force_nominal_pressure,
    double& force_factor_input_pressure
)
{
    const double length = pm_telemetry.length;
    const double lengthening_rate = pm_telemetry.lengthening_rate;
    const bool is_inflating = pm_telemetry.is_inflating();
    const double P0 = pm_configuration.nominal_pressure;
    const double F0 = pm_configuration.dynamics_F_0;
    const double F1 = pm_configuration.dynamics_F_1;
    const double K0 = pm_configuration.dynamics_K_0;
    const double K1 = pm_configuration.dynamics_K_1;
    const double B0 = is_inflating ? pm_configuration.dynamics_B_0_inflating : pm_configuration.dynamics_B_0_deflating;
    const double B1 = is_inflating ? pm_configuration.dynamics_B_1_inflating : pm_configuration.dynamics_B_1_deflating;

    force_nominal_pressure = (F0 + (F1 * P0)) - (length * (K0 + (K1 * P0))) - (lengthening_rate * (B0 + (B1 * P0)));
    force_factor_input_pressure = F1 - (length * K1) - (lengthening_rate * B1);
}

void compute_force_component(
    const IndividualPneumaticMuscleConfiguration& pm_configuration_positive_contribution,
    const IndividualPneumaticMuscleTelemetry& pm_telemetry_positive_contribution,
    const IndividualPneumaticMuscleConfiguration& pm_configuration_negative_contribution,
    const IndividualPneumaticMuscleTelemetry& pm_telemetry_negative_contribution,
    double& force_nominal_pressure,
    double& force_factor_input_pressure
)
{
    double force_nominal_pressure_positive_contribution;
    double force_factor_input_pressure_positive_contribution;
    double force_nominal_pressure_negative_contribution;
    double force_factor_input_pressure_negative_contribution;

    compute_individual_force_contribution(
        pm_configuration_positive_contribution,
        pm_telemetry_positive_contribution,
        force_nominal_pressure_positive_contribution,
        force_factor_input_pressure_positive_contribution
    );
    compute_individual_force_contribution(
        pm_configuration_negative_contribution,
        pm_telemetry_negative_contribution,
        force_nominal_pressure_negative_contribution,
        force_factor_input_pressure_negative_contribution
    );

    force_nominal_pressure = force_nominal_pressure_positive_contribution - force_nominal_pressure_negative_contribution;
    force_factor_input_pressure = force_factor_input_pressure_positive_contribution - force_factor_input_pressure_negative_contribution;
}

// Requires ros params:
// - From (5.10):
//  - k_1
//  - k_2
//  - Gamma_1
//  - Gamma_2
//  - From (3.9)
//   - From (A2)
//    - m_1
//    - m_2
//    - l_1
//    - l_c_1 (notably, does not need l_c_2)
//   - From (A4): None
//   - From (A5):
//    - g
//   - From (3.7a): (BEWARE! POSSIBLE TYPO??)
//    - n_s
//    - r_s
//    - F_0_s
//    - F_1_s
//    - P_0_b_s
//    - P_0_t_s
//    - K_0_s
//    - K_1_s
//    - B_0_b_s
//    - B_0_t_s
//    - B_1_b_s
//    - B_1_t_s
//   - From (3.7c): same as (3.7a), but replace 's' (shoulder) subscript with 'e' (elbow)
//  - From (5.3):
//   - mu_1
//   - mu_2
//  - From (3.10):
//   - From (3.7b): None
//   - From (3.7d): same as (3.7b), but replace 's' (shoulder) subscript with 'e' (elbow)
// Requires inputs:
// - From (A2):
//  - actual joint positions
// - From (A4):
//  - actual joint velocities
// - From (3.7a):
//  - x_t_s
//  - x_dot_t_s
//  - x_b_s
//  - x_dot_b_s
// - From (3.7c): same as (3.7a), but replace 's' (shoulder) subscript with 'e' (elbow)
// - From (5.3):
//  - desired joint positions
// - From (5.4):
//  - desired joint velocities
void compute_generic_segment_torque_components(
    const PneumaticMuscleSegmentConfiguration& segment_configuration,
    const PneumaticMuscleSegmentTelemetry& segment_telemetry,
    const bool bicep_is_positive_contribution,
    double& torque_nominal_pressure,
    double& torque_factor_input_pressure
)
{
    const IndividualPneumaticMuscleConfiguration* pm_configuration_positive_contribution_ptr = nullptr;
    const IndividualPneumaticMuscleTelemetry* pm_telemetry_positive_contribution_ptr = nullptr;
    const IndividualPneumaticMuscleConfiguration* pm_configuration_negative_contribution_ptr = nullptr;
    const IndividualPneumaticMuscleTelemetry* pm_telemetry_negative_contribution_ptr = nullptr;

    if (bicep_is_positive_contribution)
    {
        pm_configuration_positive_contribution_ptr = &(segment_configuration.bicep_configuration);
        pm_telemetry_positive_contribution_ptr = &(segment_telemetry.bicep_telemetry);
        pm_configuration_negative_contribution_ptr = &(segment_configuration.tricep_configuration);
        pm_telemetry_negative_contribution_ptr = &(segment_telemetry.tricep_telemetry);
    }
    else
    {
        pm_configuration_positive_contribution_ptr = &(segment_configuration.tricep_configuration);
        pm_telemetry_positive_contribution_ptr = &(segment_telemetry.tricep_telemetry);
        pm_configuration_negative_contribution_ptr = &(segment_configuration.bicep_configuration);
        pm_telemetry_negative_contribution_ptr = &(segment_telemetry.bicep_telemetry);
    }

    compute_force_component(
        *pm_configuration_positive_contribution_ptr,
        *pm_telemetry_positive_contribution_ptr,
        *pm_configuration_negative_contribution_ptr,
        *pm_telemetry_negative_contribution_ptr,
        torque_nominal_pressure,
        torque_factor_input_pressure
    );

    const double effort_factor = segment_configuration.pm_antagonist_pair_count * segment_configuration.pulley_radius;
    torque_nominal_pressure *= effort_factor;
    torque_factor_input_pressure *= effort_factor;
}

void compute_shoulder_segment_torque_components(
    const PneumaticMuscleSegmentConfiguration& shoulder_segment_configuration,
    const PneumaticMuscleSegmentTelemetry& shoulder_segment_telemetry,
    double& shoulder_torque_nominal_pressure,
    double& shoulder_torque_factor_input_pressure
)
{
    compute_generic_segment_torque_components(
        shoulder_segment_configuration,
        shoulder_segment_telemetry,
        false,
        shoulder_torque_nominal_pressure,
        shoulder_torque_factor_input_pressure
    );
}

void compute_elbow_segment_torque_components(
    const PneumaticMuscleSegmentConfiguration& elbow_segment_configuration,
    const PneumaticMuscleSegmentTelemetry& elbow_segment_telemetry,
    double& elbow_torque_nominal_pressure,
    double& elbow_torque_factor_input_pressure
)
{
    compute_generic_segment_torque_components(
        elbow_segment_configuration,
        elbow_segment_telemetry,
        true,
        elbow_torque_nominal_pressure,
        elbow_torque_factor_input_pressure
    );
}
}   // namespace pma_hardware
