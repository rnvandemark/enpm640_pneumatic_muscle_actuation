#pragma once

namespace pma_hardware
{
/**
 * A collection of telemetry which describe some properties of individual
 * pneumatic muscles.
 */
struct IndividualPneumaticMuscleTelemetry
{
public:
    /**
     * The current length of this pneumatic muscle.
     */
    double length;

    /**
     * The current rate at which the length of this pneumatic muscle is
     * changing.
     */
    double lengthening_rate;

    /**
     * Constructor, fully initializes the telemetry.
     */
    IndividualPneumaticMuscleTelemetry(
        const double l,
        const double lr
    ) :
        length(l),
        lengthening_rate(lr)
    {
    }

    /**
     * Constructor, fully initializes the telemetry to a 'zero' state.
     */
    IndividualPneumaticMuscleTelemetry() :
        IndividualPneumaticMuscleTelemetry(0.0, 0.0)
    {
    }

    /**
     * Given the current rate at which this pneumatic muscle is lengthening or
     * contracting, determine if it inflating or not.
     * @return True if this pneumatic muscle is inflating (the lengthening rate
     * is positive), false otherwise (the lengthening rate is negative and is
     * therefore contracting).
     */
    bool is_inflating() const
    {
        return lengthening_rate > 0.0;
    }
};
}   // namespace pma_hardware
