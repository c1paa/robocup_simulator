#pragma once

#include "common.h"
#include <string>
#include <vector>

class Config;

// Catadioptric mirror profile: a surface of revolution defined by a radius
// function r = f(h), where h is measured downward from the mirror's base
// (h = 0 at the base/top, h = mirrorHeight at the tip/vertex, nearest the
// camera). The profile lives in robot-local space: +Y up, mirror base at
// y = baseHeight, tip at y = baseHeight - mirrorHeight.
//
// All config values are millimetres and are converted to metres here, once,
// at load time (see AGENTS.md).
class MirrorProfile
{
public:
    // Profile: a numerically-defined mirror (e.g. one whose radius solves an
    // ODE for the single-viewpoint property, like a real hardware mirror
    // measured/calibrated externally) loaded from a CSV table instead of a
    // closed-form curve. See mirror_profile.cpp for the coordinate mapping.
    enum class Type { Cone, Hyperbola, Profile };

    // cameraHeight (metres, robot-local Y of the physical camera) is only
    // needed for Type::Profile: the CSV's own origin O is defined as the
    // camera's focal point, so the table's z values are positioned in
    // robot-local space as cameraHeight + z. Ignored for Cone/Hyperbola,
    // which position themselves from base_height directly.
    void loadFromConfig(Config& cfg, const std::string& base = "/robot/mirror",
                         float cameraHeight = 0.0f);

    Type type() const { return m_type; }

    float baseHeight() const { return m_baseHeight; }
    float baseRadius() const { return m_baseRadius; }
    float mirrorHeight() const;

    // Profile function r = f(h), h in [0, mirrorHeight] measured from the base.
    float radiusAt(float height) const;

    // Outward surface normal at the given height and radial distance. Returned
    // in the plane containing the +X axis and the mirror's optical axis
    // (i.e. for a point sitting at (+radiusOnSurface, baseHeight - height, 0));
    // the caller rotates it around the Y axis to the actual surface point.
    glm::vec3 normalAt(float height, float radiusOnSurface) const;

    // Generic ray/surface-of-revolution intersection. origin/dir are in
    // robot-local space. Returns true and stores the ray parameter in tHit if
    // the ray crosses the mirror surface within its valid height range.
    bool intersectRay(const glm::vec3& origin, const glm::vec3& dir, float& tHit) const;

    // The point from which reflected rays appear to emanate, in robot-local
    // space. For a hyperbola this is the single-viewpoint focus; for a cone
    // (which has no true SVP) we approximate with the physical camera height.
    glm::vec3 effectiveViewpointLocal(float cameraHeight) const;

private:
    float slopeAt(float height) const; // dr/dh

    Type m_type = Type::Cone;

    float m_baseHeight   = 0.15f; // Y of the base (top)
    float m_baseDiameter = 0.17f;
    float m_baseRadius   = 0.085f;
    float m_coneHeight   = 0.04f;
    float m_hyperbolaA   = 0.035f;
    float m_hyperbolaB   = 0.04f;

    // Derived hyperbola values (recomputed on load).
    float m_c     = 0.0f; // sqrt(a^2 + b^2), focus offset from mirror center
    float m_zBase = 0.0f; // z (from center) of the base edge

    // Type::Profile: CSV table (theta_deg, r_mm, z_mm relative to the camera
    // focus O), converted to metres and sorted ascending by z at load time.
    // m_csvR[i] is the mirror radius at height m_csvZ[i] above O. base_height
    // and base_radius above are derived from this table (base = the z_max
    // end, i.e. the mirror rim) rather than configured independently.
    std::vector<float> m_csvZ;
    std::vector<float> m_csvR;
};
