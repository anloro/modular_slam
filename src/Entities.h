/**
 * @file   Entities.h
 * @brief  Defines the entity classes.
 * @author Ángel Lorente Rogel
 * @date   07/04/2021
 */

#include <cstdint>
#include <limits>

#include "mrpt/math/TPose3D.h"
#include "mrpt/obs/CSensoryFrame.h"
#include "mrpt/core/Clock.h"

// This is from id
/** Unique ID for each Entity in a WorldModel. \ingroup mola_kernel_grp */
using id_tronco = std::uint64_t;

/** Unique ID for each Factor in a WorldModel. \ingroup mola_kernel_grp */
using fid_tronco = std::uint64_t;

/** A numeric value for invalid IDs. \ingroup mola_kernel_grp */
constexpr id_tronco INVALID_ID = std::numeric_limits<id_tronco>::max();

/** A numeric value for invalid IDs. \ingroup mola_kernel_grp */
constexpr fid_tronco INVALID_FID = std::numeric_limits<fid_tronco>::max();

// Entity relative pose key-frame
class RelPose3KF{
    public:
        /** The unique ID of this entity in the world model.
             * Stored here for convenience, notice that it is redundant since entities
             * are already stored in the WorldModel indexed by ID.
             */
        id_tronco my_id_{INVALID_ID};

        /** Entity creation timestamp */
        mrpt::Clock::time_point timestamp_{};
        /** The ID of the base keyframe (entity) */
        id_tronco base_id_{INVALID_ID};
        /** The up-to-date value of this entity. */
        mrpt::math::TPose3D relpose_value;
        // store the raw observations
        mrpt::obs::CSensoryFrame::Ptr raw_observations_;
};