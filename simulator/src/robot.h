#pragma once

#include "common.h"
#include "mirror_profile.h"
#include <btBulletDynamicsCommon.h>
#include <memory>
#include <vector>

class Config;
class Renderer;

class Robot
{
public:
    ~Robot();

    void init(Config& cfg, btDiscreteDynamicsWorld* world);
    void applyDriveForces(float dt);
    void syncFromPhysics();
    void render(Renderer& renderer);
    void renderBody(Renderer& renderer); // body + wheels only (no mirror/camera)

    // What the robot's own mirror-camera actually sees below the mirror: real
    // hardware is mostly an open frame there (wiring/PCB aside), held up by a
    // few thin support pillars above the wheels — not the solid chassis
    // cylinder renderBody() draws for the debug viewer. Used only in the
    // camera capture pass (see App::render).
    void renderCameraFrame(Renderer& renderer);

    // gRPC-to-robot commands (body-frame: vx forward, vy lateral, omega yaw)
    void setBodyVelocity(float vx, float vy, float omega);

    // Robot state (for gRPC)
    glm::vec3 position() const { return m_position; }
    float orientation() const { return m_yaw; }
    float velocity() const { return m_velocity; }
    float angularVelocity() const { return m_angularVelocity; }
    float diameter() const { return m_diameter; } // m, for shadow-blob radius etc.

    // World-frame linear velocity vector (velocity() above is just its XZ
    // magnitude) — needed by Dribbler to compute the velocity a point
    // rigidly attached to the robot would have (V_robot + omega x offset).
    glm::vec3 linearVelocityWorld() const { return m_linearVelocityWorld; }

    // Dead-reckoning odometry (drifts away from ground truth under slip)
    float odometryX() const { return m_odomX; }
    float odometryZ() const { return m_odomZ; }
    float odometryYaw() const { return m_odomYaw; }

    // Mirror / camera geometry (single source of truth for mirror shape)
    const MirrorProfile& mirrorProfile() const { return m_mirror; }
    float cameraHeight() const { return m_cameraHeight; }

private:
    static constexpr int kOmniWheels = 3;

    // Position in world (read back from the Bullet body after each step)
    glm::vec3 m_position = glm::vec3(0.0f, 0.04f, 0.0f);
    float m_yaw = 0.0f;
    float m_velocity = 0.0f;
    float m_angularVelocity = 0.0f;
    glm::vec3 m_linearVelocityWorld = glm::vec3(0.0f);

    // Commanded body-frame velocity (vx forward, vy lateral, omega yaw rate)
    float m_targetVx    = 0.0f;
    float m_targetVy    = 0.0f;
    float m_targetOmega = 0.0f;

    // Per-wheel motor state (first-order-lagged "encoder" surface speed, m/s)
    float m_wheelTarget[kOmniWheels] = {0.0f, 0.0f, 0.0f};
    float m_wheelSpeed[kOmniWheels]  = {0.0f, 0.0f, 0.0f};

    // Dead-reckoning odometry (integrates wheel speeds assuming zero slip)
    float m_odomX = 0.0f;
    float m_odomZ = 0.0f;
    float m_odomYaw = 0.0f;

    // ---- Robot geometry (meters) ----
    float m_diameter = 0.18f;
    float m_height   = 0.15f;

    // ---- Wheels ----
    int   m_wheelCount          = 3;
    float m_wheelCenterDiameter = 0.16f;  // circle on which wheel centers sit
    float m_wheelDiameter       = 0.048f; // diameter of each wheel
    float m_theta0 = glm::pi<float>() * 0.5f; // wheel 0 mounting angle (rad)

    // ---- Mass / friction / inertia ----
    float m_mass = 2.5f;      // kg (config gives grams)
    float m_gravity = 9.81f;  // m/s^2
    float m_wheelFrictionDriven  = 0.9f;
    float m_wheelFrictionLateral = 0.15f;
    float m_frictionResponseGain = 0.4f;

    // ---- Motor parameters (three MF4015v2 direct-drive BLDC motors, one per
    // wheel — see the comment above the config reads in Robot::init) ----
    float m_maxLinearSpeed  = 1.885f;
    float m_maxAngularSpeed = 23.56f;
    float m_motorTimeConstant = 0.05f;
    float m_motorPeakTorque = 0.65f; // N*m, caps per-wheel drive force regardless of ground friction

    // ---- Precomputed omni-wheel geometry (angles are fixed) ----
    float m_wheelR = 0.08f; // mounting radius = center_diameter / 2
    float m_sinTheta[kOmniWheels];
    float m_cosTheta[kOmniWheels];
    glm::mat3 m_invKinematics = glm::mat3(1.0f);

    // ---- Bullet rigid body (compound: a fan of angular wedge boxes
    // approximating the chassis cylinder, full radius except at the frontal
    // dribbler notch — see the pocket_depth comment in Robot::init) ----
    btDiscreteDynamicsWorld* m_world = nullptr;
    std::vector<std::unique_ptr<btBoxShape>> m_chassisWedgeShapes;
    std::unique_ptr<btCompoundShape> m_collisionShape;
    std::unique_ptr<btDefaultMotionState> m_motionState;
    std::unique_ptr<btRigidBody> m_body;

    // ---- Mirror (cone or hyperbola, apex down toward camera) ----
    MirrorProfile m_mirror;

    // ---- Camera (looks up into mirror) ----
    float m_cameraHeight = 0.11f;

    // ---- Camera-visible frame (support pillars, see renderCameraFrame) ----
    int   m_pillarCount = 3;
    float m_pillarWidth = 0.008f;
    float m_pillarRadialOffset = 0.0f;

    // ---- Colors ----
    glm::vec3 m_bodyColor   = glm::vec3(0.1f, 0.3f, 0.8f);
    glm::vec3 m_wheelColor  = glm::vec3(0.1f, 0.1f, 0.1f);
    glm::vec3 m_mirrorColor = glm::vec3(0.7f, 0.7f, 0.8f);
};
