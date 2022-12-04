#pragma once

#include "pma_hardware/description/individual_pneumatic_muscle_configuration.hpp"

namespace pma_hardware
{
/**
 * A collection of static configuration parameters which describe some
 * properties of a segment in a robot arm which consists of pneumatic muscle
 * actuators.
 */
struct PneumaticMuscleSegmentConfiguration
{
public:
    /**
     * The overall mass of this segment.
     */
    const double mass;

    /**
     * The radius of the pulley, of which antagonist pairs of pneumatic muscles
     * act about to induce motion in the joint.
     */
    const double pulley_radius;

    /**
     * The overall length of this segment, from the proximal to distal ends.
     */
    const double segment_length;

    /**
     * The length from the proximal end of this segment to this segment's
     * center of mass.
     */
    const double segment_com_length;

    /**
     * The number of matched pneumatic muscles that have the same coefficients
     * as each other, same lengths, etc.
     */
    const size_t pm_antagonist_pair_count;

    /**
     * The configuration parameters for the bicep pneumatic muscle.
     */
    const IndividualPneumaticMuscleConfiguration bicep_configuration;

    /**
     * The configuration parameters for the tricep pneumatic muscle.
     */
    const IndividualPneumaticMuscleConfiguration tricep_configuration;

    /**
     * Constructor, fully initializes the description.
     */
    PneumaticMuscleSegmentConfiguration(
        const double m,
        const double pr,
        const double sl,
        const double scl,
        const size_t papc,
        const double bnp,
        const double bdF0,
        const double bdF1,
        const double bdK1,
        const double bdK2,
        const double bdB0i,
        const double bdB1i,
        const double bdB0d,
        const double bdB1d,
        const double tnp,
        const double tdF0,
        const double tdF1,
        const double tdK1,
        const double tdK2,
        const double tdB0i,
        const double tdB1i,
        const double tdB0d,
        const double tdB1d
    ) :
        mass(m),
        pulley_radius(pr),
        segment_length(sl),
        segment_com_length(scl),
        pm_antagonist_pair_count(papc),
        bicep_configuration(
            bnp,
            bdF0,
            bdF1,
            bdK1,
            bdK2,
            bdB0i,
            bdB1i,
            bdB0d,
            bdB1d
        ),
        tricep_configuration(
            tnp,
            tdF0,
            tdF1,
            tdK1,
            tdK2,
            tdB0i,
            tdB1i,
            tdB0d,
            tdB1d
        )
    {
    }
};
}   // namespace pma_hardware
