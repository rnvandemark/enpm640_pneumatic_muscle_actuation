#pragma once

#include "pma_hardware/description/individual_pneumatic_muscle_telemetry.hpp"

namespace pma_hardware
{
/**
 * A collection of telemetry which describe some properties of a segment in a
 * robot arm which consists of pneumatic muscle actuators.
 */
struct PneumaticMuscleSegmentTelemetry
{
public:
    /**
     * The current telemetry for the bicep pneumatic muscle.
     */
    IndividualPneumaticMuscleTelemetry bicep_telemetry;

    /**
     * The current telemetry for the tricep pneumatic muscle.
     */
    IndividualPneumaticMuscleTelemetry tricep_telemetry;
};
}   // namespace pma_hardware
