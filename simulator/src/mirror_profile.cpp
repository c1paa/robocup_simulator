#include "mirror_profile.h"
#include "config.h"
#include <cmath>
#include <algorithm>

static const float MM = 1000.0f;

void MirrorProfile::loadFromConfig(Config& cfg, const std::string& base)
{
    std::string typeStr = cfg.getString(base + "/type", "cone");
    m_type = (typeStr == "hyperbola") ? Type::Hyperbola : Type::Cone;

    m_baseHeight   = cfg.getFloat(base + "/base_height",   150.0f) / MM;
    m_baseDiameter = cfg.getFloat(base + "/base_diameter", 170.0f) / MM;
    m_baseRadius   = m_baseDiameter * 0.5f;
    m_coneHeight   = cfg.getFloat(base + "/cone_height",   40.0f)  / MM;
    m_hyperbolaA   = cfg.getFloat(base + "/hyperbola_a",   35.0f)  / MM;
    m_hyperbolaB   = cfg.getFloat(base + "/hyperbola_b",   40.0f)  / MM;

    // Derived hyperbola geometry. The mirror is the upper branch of
    // (z/a)^2 - (r/b)^2 = 1, z measured upward from the mirror center. The
    // base edge sits at z = zBase where r = baseRadius; the tip is at z = a.
    m_c = std::sqrt(m_hyperbolaA * m_hyperbolaA + m_hyperbolaB * m_hyperbolaB);
    float scaled = (m_hyperbolaA * m_baseRadius) / m_hyperbolaB;
    m_zBase = std::sqrt(m_hyperbolaA * m_hyperbolaA + scaled * scaled);
}

float MirrorProfile::mirrorHeight() const
{
    if (m_type == Type::Hyperbola) return m_zBase - m_hyperbolaA;
    return m_coneHeight;
}

float MirrorProfile::radiusAt(float height) const
{
    float h = std::clamp(height, 0.0f, mirrorHeight());
    if (m_type == Type::Hyperbola) {
        float z = m_zBase - h; // z in [a, zBase]
        float arg = z * z - m_hyperbolaA * m_hyperbolaA;
        if (arg < 0.0f) arg = 0.0f;
        return (m_hyperbolaB / m_hyperbolaA) * std::sqrt(arg);
    }
    // Cone: linear from baseRadius at h=0 to 0 at h=coneHeight.
    return m_baseRadius * (1.0f - h / m_coneHeight);
}

float MirrorProfile::slopeAt(float height) const
{
    float h = std::clamp(height, 0.0f, mirrorHeight());
    if (m_type == Type::Hyperbola) {
        float z = m_zBase - h;
        float arg = z * z - m_hyperbolaA * m_hyperbolaA;
        if (arg <= 0.0f) return -1e9f; // tip: vertical surface
        return -(m_hyperbolaB / m_hyperbolaA) * z / std::sqrt(arg);
    }
    return -m_baseRadius / m_coneHeight;
}

glm::vec3 MirrorProfile::normalAt(float height, float radiusOnSurface) const
{
    (void)radiusOnSurface;
    // In the (r, y) plane the profile tangent is (f'(h), -1), so an outward
    // normal is (1, f'(h)). Return it in 3D with the radial component along +X.
    float slope = slopeAt(height);
    glm::vec3 n(1.0f, slope, 0.0f);
    float len = glm::length(n);
    if (len < 1e-6f) return glm::vec3(0.0f, 1.0f, 0.0f);
    return n / len;
}

bool MirrorProfile::intersectRay(const glm::vec3& origin, const glm::vec3& dir, float& tHit) const
{
    if (dir.y <= 1e-6f) return false;

    float h = mirrorHeight();
    float yTip  = m_baseHeight - h; // tip (narrowest, near camera)
    float yBase = m_baseHeight;     // base (top)

    float tTip  = (yTip  - origin.y) / dir.y;
    float tBase = (yBase - origin.y) / dir.y;
    float tA = std::min(tTip, tBase);
    float tB = std::max(tTip, tBase);
    if (tB < 0.0f) return false; // mirror entirely behind the origin
    tA = std::max(tA, 0.0f);

    auto g = [&](float t) {
        glm::vec3 p = origin + dir * t;
        float hh = yBase - p.y; // height from base
        float rad = std::sqrt(p.x * p.x + p.z * p.z);
        return rad - radiusAt(hh);
    };

    const int N = 64;
    float prevT = tA;
    float prevG = g(prevT);
    for (int i = 1; i <= N; i++) {
        float t = tA + (tB - tA) * (float)i / (float)N;
        float gi = g(t);
        if (prevG * gi <= 0.0f) {
            float lo = prevT, hi = t;
            float glo = prevG;
            for (int k = 0; k < 40; k++) {
                float mid = 0.5f * (lo + hi);
                float gm = g(mid);
                if (glo * gm <= 0.0f) hi = mid;
                else { lo = mid; glo = gm; }
            }
            tHit = 0.5f * (lo + hi);
            return true;
        }
        prevT = t;
        prevG = gi;
    }
    return false;
}

glm::vec3 MirrorProfile::effectiveViewpointLocal(float cameraHeight) const
{
    if (m_type == Type::Hyperbola) {
        // Upper focus of the hyperboloid (the single-viewpoint position).
        return glm::vec3(0.0f, m_baseHeight - m_zBase + m_c, 0.0f);
    }
    // Cone: no true SVP, approximate with the physical camera position.
    return glm::vec3(0.0f, cameraHeight, 0.0f);
}
