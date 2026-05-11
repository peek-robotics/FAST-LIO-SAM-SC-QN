#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

/**
 * ZHeightFactor
 *
 * Soft constraint on the world-frame Z coordinate of a Pose3 node.
 * Useful as a between-GPS-fix anchor to prevent LIO Z drift on flat or gently
 * sloping terrain. A single scalar residual:
 *
 *   e = pose.translation().z() - z_ref
 *
 * Jacobian (1×6) is computed via Pose3::translation(H) which correctly handles
 * both GTSAM_POSE3_EXPMAP and the default first-order retract convention.
 *
 * Usage:
 *   auto noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma_m);
 *   graph.add(ZHeightFactor(key, z_reference, noise));
 */
class ZHeightFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
    using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

    ZHeightFactor(gtsam::Key key, double z_ref,
                  const gtsam::noiseModel::Base::shared_ptr &noise)
        : Base(noise, key), z_ref_(z_ref) {}

    gtsam::Vector evaluateError(
        const gtsam::Pose3 &pose,
        boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        if (H)
        {
            // Use GTSAM's own Pose3::translation Jacobian so the convention
            // (expmap vs first-order) is handled correctly for whichever build.
            gtsam::Matrix36 Dt;
            pose.translation(Dt);   // fills 3×6 Jacobian of translation w.r.t. Pose3 tangent
            *H = Dt.row(2);         // keep only the Z row → 1×6
        }
        return gtsam::Vector1(pose.translation().z() - z_ref_);
    }

private:
    double z_ref_;
};
