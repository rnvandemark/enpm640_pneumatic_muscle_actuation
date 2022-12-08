#pragma once

#include <string>

namespace pma_hardware
{
/**
 * A collection of static configuration parameters which describe some
 * properties of individual pneumatic muscles.
 */
struct IndividualPneumaticMuscleConfiguration
{
public:
    /**
     * An arbitrary positive nominal pressure for this pneumatic muscle, chosen
     * so that this segment's joint is well-behaved in the absence of a control
     * pressure and desired joint stiffness is produced.
     */
    const double nominal_pressure;

    /**
     * Constant F0 of the dynamics of a robot arm of this type.
     */
    const double dynamics_F_0;

    /**
     * Constant F1 of the dynamics of a robot arm of this type.
     */
    const double dynamics_F_1;

    /**
     * Constant K0 of the dynamics of a robot arm of this type.
     */
    const double dynamics_K_0;

    /**
     * Constant K1 of the dynamics of a robot arm of this type.
     */
    const double dynamics_K_1;

    /**
     * Constant B0 (while inflating) of the dynamics of a robot arm of this
     * type.
     */
    const double dynamics_B_0_inflating;

    /**
     * Constant B1 (while inflating) of the dynamics of a robot arm of this
     * type.
     */
    const double dynamics_B_1_inflating;

    /**
     * Constant B0 (while deflating) of the dynamics of a robot arm of this
     * type.
     */
    const double dynamics_B_0_deflating;

    /**
     * Constant B1 (while deflating) of the dynamics of a robot arm of this
     * type.
     */
    const double dynamics_B_1_deflating;

    /**
     * Constructor, fully initializes the description.
     */
    IndividualPneumaticMuscleConfiguration(
        const double np,
        const double dF0,
        const double dF1,
        const double dK1,
        const double dK2,
        const double dB0i,
        const double dB1i,
        const double dB0d,
        const double dB1d
    ) :
        nominal_pressure(np),
        dynamics_F_0(dF0),
        dynamics_F_1(dF1),
        dynamics_K_0(dK1),
        dynamics_K_1(dK2),
        dynamics_B_0_inflating(dB0i),
        dynamics_B_1_inflating(dB1i),
        dynamics_B_0_deflating(dB0d),
        dynamics_B_1_deflating(dB1d)
    {
    }

    /**
     * Convert this data to a string.
     * @return The stringified configuration.
     */
    std::string to_string() const
    {
        return std::string("{")
            + "nominal_pressure=" + std::to_string(nominal_pressure)
            + ", dynamics_F_0=" + std::to_string(dynamics_F_0)
            + ", dynamics_F_1=" + std::to_string(dynamics_F_1)
            + ", dynamics_K_0=" + std::to_string(dynamics_K_0)
            + ", dynamics_K_1=" + std::to_string(dynamics_K_1)
            + ", dynamics_B_0_inflating=" + std::to_string(dynamics_B_0_inflating)
            + ", dynamics_B_1_inflating=" + std::to_string(dynamics_B_1_inflating)
            + ", dynamics_B_0_deflating=" + std::to_string(dynamics_B_0_deflating)
            + ", dynamics_B_1_deflating=" + std::to_string(dynamics_B_1_deflating)
            + "}"
        ;
    }
};
}   // namespace pma_hardware
