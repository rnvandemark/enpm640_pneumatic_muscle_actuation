#pragma once

namespace pma_hardware
{
/**
 * A collection of static configuration parameters which describe some
 * properties of a system manaing a robot arm which consists of pneumatic
 * muscle actuators.
 */
struct PneumaticMuscleControlConfiguration
{
public:
    /**
     * The world's acceleration due to gravity.
     */
    const double a_g;

    /**
     * (Positive) design constant k1 of a sliding-mode control law.
     */
    const double k_1;

    /**
     * (Positive) design constant k2 of a sliding-mode control law.
     */
    const double k_2;

    /**
     * Boundary layer thickness Gamma 1 of a sliding-mode control law.
     */
    const double Gamma_1;

    /**
     * Boundary layer thickness Gamma 2 of a sliding-mode control law.
     */
    const double Gamma_2;

    /**
     * (Positive) design constant mu 1 of a sliding-mode control law.
     */
    const double mu_1;

    /**
     * (Positive) design constant mu 2 of a sliding-mode control law.
     */
    const double mu_2;

    /**
     * Constructor, fully initializes the description.
     */
    PneumaticMuscleControlConfiguration(
		const double ag,
        const double k1,
        const double k2,
        const double G1,
        const size_t G2,
        const double m1,
        const double m2
	) :
        a_g(ag),
        k_1(k1),
        k_2(k2),
        Gamma_1(G1),
        Gamma_2(G2),
        mu_1(m1),
        mu_2(m2)
    {
    }
};
}   // namespace pma_hardware
