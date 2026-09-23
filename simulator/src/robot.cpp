#include "robot.h"
#include "renderer.h"
#include "config.h"
#include <glm/gtc/matrix_inverse.hpp>
#include <algorithm>
#include <cmath>

static const float MM = 1000.0f;
static const float DEG2RAD = glm::pi<float>() / 180.0f;

Robot::~Robot()
{
    if (m_world && m_body) {
        m_world->removeRigidBody(m_body.get());
    }
}

void Robot::init(Config& cfg, btDiscreteDynamicsWorld* world)
{
    m_world = world;

    // Geometry
    m_diameter = cfg.getFloat("/robot/diameter", 180.0f) / MM;
    m_height   = cfg.getFloat("/robot/height",   150.0f) / MM;

    // Wheels
    m_wheelCount          = cfg.getInt("/robot/wheels/count", 3);
    m_wheelCenterDiameter = cfg.getFloat("/robot/wheels/center_diameter", 160.0f) / MM;
    m_wheelDiameter       = cfg.getFloat("/robot/wheels/wheel_diameter",  48.0f)  / MM;
    float angleOffsetDeg  = cfg.getFloat("/robot/wheels/angle_offset", 90.0f);
    m_theta0 = angleOffsetDeg * DEG2RAD;

    // Mirror (single source of truth for mirror geometry)
    m_mirror.loadFromConfig(cfg);

    // Camera
    m_cameraHeight = cfg.getFloat("/robot/camera/height", 110.0f) / MM;

    // Motor
    m_maxLinearSpeed   = cfg.getFloat("/robot/motor/max_linear_speed",  3000.0f) / MM;
    m_maxAngularSpeed  = cfg.getFloat("/robot/motor/max_angular_speed", 6.28f);
    m_motorTimeConstant = cfg.getFloat("/robot/motor/time_constant", 0.05f);

    // Physics. Note: mass is configured in *grams*, not millimetres — it is
    // converted to kg with the same / 1000 pattern as mm -> m, but it is a
    // different physical quantity. Gravity (mm/s^2) converts the same way
    // Physics::init does, so the two agree.
    m_mass = cfg.getFloat("/physics/robot/mass", 2500.0f) / 1000.0f;
    m_gravity = cfg.getFloat("/physics/gravity", 9810.0f) / MM;
    m_wheelFrictionDriven  = cfg.getFloat("/robot/physics/wheel_friction_driven",  0.9f);
    m_wheelFrictionLateral = cfg.getFloat("/robot/physics/wheel_friction_lateral", 0.15f);

    // ---- Omni-wheel geometry (precompute once; angles are fixed) ----
    m_wheelR = m_wheelCenterDiameter * 0.5f;
    float angleStep = (2.0f * glm::pi<float>()) / (float)kOmniWheels;
    for (int i = 0; i < kOmniWheels; i++) {
        float theta = m_theta0 + (float)i * angleStep;
        m_sinTheta[i] = std::sin(theta);
        m_cosTheta[i] = std::cos(theta);
    }
    // Inverse-kinematics matrix: math rows are (-sin_i, cos_i, R). glm is
    // column-major, so M[j] is column j = (row0[j], row1[j], row2[j]).
    glm::mat3 M(1.0f);
    M[0] = glm::vec3(-m_sinTheta[0], -m_sinTheta[1], -m_sinTheta[2]);
    M[1] = glm::vec3( m_cosTheta[0],  m_cosTheta[1],  m_cosTheta[2]);
    M[2] = glm::vec3(-m_wheelR, -m_wheelR, -m_wheelR);
    m_invKinematics = glm::inverse(M);

    // ---- Bullet rigid body (chassis cylinder) ----
    float radius = m_diameter * 0.5f;
    float halfH  = m_height * 0.5f;
    m_collisionShape = std::make_unique<btCylinderShape>(btVector3(radius, halfH, radius));

    btScalar mass = m_mass;
    btVector3 localInertia(0.0f, 0.0f, 0.0f);
    m_collisionShape->calculateLocalInertia(mass, localInertia);

    // Yaw inertia about the vertical (Y) axis drives turning. The config key is
    // named "z" for the spin axis; absent/null means "solid-cylinder default".
    float iz = cfg.getFloat("/robot/physics/moment_of_inertia_z", 0.0f);
    if (iz <= 0.0f) iz = 0.5f * m_mass * radius * radius;
    localInertia.setY(iz);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0.0f, halfH, 0.0f));
    m_motionState = std::make_unique<btDefaultMotionState>(startTransform);

    btRigidBody::btRigidBodyConstructionInfo ci(mass, m_motionState.get(),
                                                m_collisionShape.get(), localInertia);
    // Chassis belly-on-ground drag, not traction — traction comes from the
    // per-wheel friction model in applyDriveForces, so keep this near zero.
    ci.m_friction    = cfg.getFloat("/physics/robot/friction", 0.05f);
    ci.m_restitution = cfg.getFloat("/physics/robot/restitution", 0.0f);
    m_body = std::make_unique<btRigidBody>(ci);
    m_body->setAngularFactor(btVector3(0.0f, 1.0f, 0.0f)); // yaw only, no tipping
    m_body->setDamping(0.0f, 0.0f);
    m_body->setActivationState(DISABLE_DEACTIVATION);
    m_world->addRigidBody(m_body.get());

    m_position = glm::vec3(0.0f, halfH, 0.0f);
    m_yaw = 0.0f;
    m_odomX = 0.0f;
    m_odomZ = 0.0f;
    m_odomYaw = 0.0f;
}

void Robot::setBodyVelocity(float vx, float vy, float omega)
{
    m_targetVx    = std::clamp(vx,    -m_maxLinearSpeed,  m_maxLinearSpeed);
    m_targetVy    = std::clamp(vy,    -m_maxLinearSpeed,  m_maxLinearSpeed);
    m_targetOmega = std::clamp(omega, -m_maxAngularSpeed, m_maxAngularSpeed);
}

void Robot::kick(float power)
{
    (void)power;
}

void Robot::dribble(float speed)
{
    (void)speed;
}

void Robot::applyDriveForces(float dt)
{
    if (!m_body || !m_world) return;
    if (dt <= 0.0f) return;

    // ---- Forward kinematics: commanded body velocity -> wheel targets ----
    const float R = m_wheelR;
    const float vx = m_targetVx, vy = m_targetVy, omega = m_targetOmega;
    for (int i = 0; i < kOmniWheels; i++) {
        m_wheelTarget[i] = -vx * m_sinTheta[i] + vy * m_cosTheta[i] - omega * R;
    }

    // ---- First-order motor lag, per wheel. This lagged value is what a real
    // motor/encoder reports (blind to ground slip), so it feeds both the
    // friction model and odometry. ----
    float alpha = dt / (m_motorTimeConstant + dt);
    for (int i = 0; i < kOmniWheels; i++) {
        m_wheelSpeed[i] = util::lerp(m_wheelSpeed[i], m_wheelTarget[i], alpha);
    }

    // ---- Actual body velocity in the robot's local frame ----
    btTransform trans = m_body->getWorldTransform();
    btVector3 fwd = trans.getBasis() * btVector3(1.0f, 0.0f, 0.0f);
    // This codebase's yaw convention (established by the mirror-camera math in
    // camera.cpp / glm::rotate): +X rotates toward -Z for increasing yaw. That
    // maps to world (cos yaw, -sin yaw), hence the negated Z here.
    float yaw = std::atan2(-fwd.z(), fwd.x());
    float cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);

    btVector3 linWorld = m_body->getLinearVelocity();
    btVector3 angWorld = m_body->getAngularVelocity();
    // world -> local (inverse of yaw rotation about Y; local->world is
    // worldX = cosYaw*localX + sinYaw*localZ, worldZ = -sinYaw*localX + cosYaw*localZ,
    // so this is that matrix's transpose).
    float linX = cosYaw * linWorld.x() - sinYaw * linWorld.z();
    float linZ = sinYaw * linWorld.x() + cosYaw * linWorld.z();
    // Local yaw rate: Bullet's raw angular-velocity Y component already
    // matches this convention directly (no negation) — see the derivation in
    // docs/tasks/omni-wheel-dynamics.md.
    float angY = angWorld.y();

    float wheelMass = m_mass / (float)kOmniWheels;
    float normal = m_mass * m_gravity / (float)kOmniWheels; // even weight split

    float maxRolling = m_wheelFrictionDriven * normal;
    float maxLateral = m_wheelFrictionLateral * normal;

    // Desired force per wheel that would fully close each slip gap this step,
    // computed first for all wheels so the Coulomb limit can be applied as a
    // *common* scale factor. Clamping each wheel independently would break the
    // force ratio the omni geometry needs (wheel 0 carries twice the rear
    // wheels' load for pure forward motion) and produce spurious net torque.
    float desiredRolling[kOmniWheels];
    float desiredLateral[kOmniWheels];
    for (int i = 0; i < kOmniWheels; i++) {
        float sinT = m_sinTheta[i], cosT = m_cosTheta[i];

        // Actual ground-contact velocity in rolling (t) and lateral (n) axes.
        float actualRolling = -linX * sinT + linZ * cosT - angY * R;
        float actualLateral =  linX * cosT + linZ * sinT;

        float slipRolling = m_wheelSpeed[i] - actualRolling;
        float slipLateral = 0.0f - actualLateral; // wheel is never driven sideways

        desiredRolling[i] = wheelMass * slipRolling / dt;
        desiredLateral[i] = wheelMass * slipLateral / dt;
    }

    float maxRoll = 0.0f;
    for (int i = 0; i < kOmniWheels; i++) maxRoll = std::max(maxRoll, std::fabs(desiredRolling[i]));
    float rollScale = (maxRoll > maxRolling) ? (maxRolling / maxRoll) : 1.0f;

    float maxLat = 0.0f;
    for (int i = 0; i < kOmniWheels; i++) maxLat = std::max(maxLat, std::fabs(desiredLateral[i]));
    float latScale = (maxLat > maxLateral) ? (maxLateral / maxLat) : 1.0f;

    for (int i = 0; i < kOmniWheels; i++) {
        float sinT = m_sinTheta[i], cosT = m_cosTheta[i];

        float fRolling = desiredRolling[i] * rollScale;
        float fLateral = desiredLateral[i] * latScale;

        // Local force = fRolling * t_i + fLateral * n_i.
        glm::vec3 localForce(
            -fRolling * sinT + fLateral * cosT,
            0.0f,
             fRolling * cosT + fLateral * sinT);

        // Rotate local -> world (worldX = cosYaw*localX + sinYaw*localZ,
        // worldZ = -sinYaw*localX + cosYaw*localZ — matches Camera::renderView
        // / glm::rotate; see AGENTS.md's local-frame convention note).
        btVector3 worldForce(
            cosYaw * localForce.x + sinYaw * localForce.z,
            0.0f,
            -sinYaw * localForce.x + cosYaw * localForce.z);

        // Contact point offset (world space, relative to center of mass).
        btVector3 contactWorld(
            cosYaw * (R * cosT) + sinYaw * (R * sinT),
            0.0f,
            -sinYaw * (R * cosT) + cosYaw * (R * sinT));

        m_body->applyForce(worldForce, contactWorld);
    }

    // ---- Dead-reckoning odometry: inverse kinematics on the lagged wheel
    // speeds, integrated assuming zero slip. ----
    glm::vec3 est = m_invKinematics * glm::vec3(m_wheelSpeed[0], m_wheelSpeed[1], m_wheelSpeed[2]);
    float vxEst = est.x, vyEst = est.y, wEst = est.z;
    m_odomYaw += wEst * dt;
    float cosO = std::cos(m_odomYaw), sinO = std::sin(m_odomYaw);
    m_odomX += (cosO * vxEst + sinO * vyEst) * dt;
    m_odomZ += (-sinO * vxEst + cosO * vyEst) * dt;
}

void Robot::syncFromPhysics()
{
    if (!m_body) return;

    btTransform trans = m_body->getWorldTransform();
    btVector3 pos = trans.getOrigin();
    m_position = glm::vec3(pos.x(), pos.y(), pos.z());

    btVector3 fwd = trans.getBasis() * btVector3(1.0f, 0.0f, 0.0f);
    // Same convention fix as applyDriveForces: +X rotates toward -Z.
    m_yaw = std::atan2(-fwd.z(), fwd.x());

    btVector3 lin = m_body->getLinearVelocity();
    m_velocity = std::sqrt(lin.x() * lin.x() + lin.z() * lin.z());
    // Bullet's raw angular velocity Y component already matches this
    // codebase's yaw convention directly (see applyDriveForces).
    m_angularVelocity = m_body->getAngularVelocity().y();
}

void Robot::render(Renderer& renderer)
{
    renderBody(renderer);

    // ---- Mirror (cone or hyperbola placeholder, apex down toward camera) ----
    {
        float mirrorHeight = m_mirror.mirrorHeight();
        float baseHeight   = m_mirror.baseHeight();
        float baseRadius   = m_mirror.baseRadius();
        float bodyHalf = m_height * 0.5f;
        float apexY = baseHeight - mirrorHeight;
        float mirrorCenterY = (baseHeight + apexY) * 0.5f;
        float mirrorLocalY = mirrorCenterY - bodyHalf;
        glm::mat4 mirrorLocal = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, mirrorLocalY, 0.0f));
        glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position);
        model = glm::rotate(model, m_yaw, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 mirrorModel = model * mirrorLocal;
        // Hyperbola is drawn as the same bounding cone as a visual placeholder.
        renderer.drawCone(baseRadius, mirrorHeight, mirrorModel, m_mirrorColor);
    }

    // ---- Camera indicator ----
    {
        float bodyHalf = m_height * 0.5f;
        float camLocalY = m_cameraHeight - bodyHalf;
        glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position);
        model = glm::rotate(model, m_yaw, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 camLocal = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, camLocalY, 0.0f));
        glm::mat4 camModel = model * camLocal;
        renderer.drawSphere(0.015f, camModel, glm::vec3(1.0f, 0.2f, 0.2f));
    }
}

void Robot::renderBody(Renderer& renderer)
{
    float radius = m_diameter * 0.5f;
    float wheelRadius = m_wheelDiameter * 0.5f;
    float wheelCenterDist = m_wheelCenterDiameter * 0.5f;
    float bodyHalf = m_height * 0.5f;

    glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position);
    model = glm::rotate(model, m_yaw, glm::vec3(0.0f, 1.0f, 0.0f));

    // ---- Body ----
    renderer.drawCylinder(radius, m_height, model, m_bodyColor);

    // ---- Direction line on top face (center → front edge) ----
    {
        glm::vec3 topCenter = model * glm::vec4(0.0f, bodyHalf, 0.0f, 1.0f);
        glm::vec3 topFront  = model * glm::vec4(radius, bodyHalf, 0.0f, 1.0f);
        renderer.drawLine(topCenter, topFront, glm::vec3(1.0f, 0.8f, 0.2f));
    }

    // ---- Wheels (evenly spaced, vertical disks, offset by angle_offset) ----
    float angleStep = (2.0f * glm::pi<float>()) / (float)m_wheelCount;
    for (int i = 0; i < m_wheelCount; i++) {
        float angle = m_theta0 + (float)i * angleStep;
        float dx = std::cos(angle) * wheelCenterDist;
        float dz = std::sin(angle) * wheelCenterDist;

        // Wheel center: at wheelRadius above ground, relative to robot center
        float wheelLocalY = wheelRadius - bodyHalf;

        // Build wheel local transform: position + orientation
        glm::mat4 wheelLocal = glm::mat4(1.0f);
        wheelLocal = glm::translate(wheelLocal, glm::vec3(dx, wheelLocalY, dz));

        // Rotate cylinder axis from Y-up to radial direction (axis of wheel)
        glm::vec3 radialDir = glm::normalize(glm::vec3(dx, 0.0f, dz));
        glm::vec3 rotAxis   = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), radialDir);
        wheelLocal = glm::rotate(wheelLocal, glm::pi<float>() / 2.0f, rotAxis);

        // Combine with robot world transform
        glm::mat4 wheelModel = model * wheelLocal;
        renderer.drawCylinder(wheelRadius, 0.01f, wheelModel, m_wheelColor);
    }
}
