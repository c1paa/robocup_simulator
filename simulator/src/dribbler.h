#pragma once

#include "common.h"

class Config;
class Robot;
class Ball;

// Dribbler roller: a hand-rolled friction/capture force model (deliberately NOT
// a Bullet collision body — sustained cylinder/roller-vs-sphere contact was
// measured to be solver-unstable, see docs/tasks/dribbler-kicker.md). Every
// frame, if the ball is inside the geometric capture zone in front of the
// roller, an explicit Coulomb friction force is applied at the ball's contact
// point so the roller pulls the ball toward the robot and imparts backspin.
class Dribbler
{
public:
    void init(Config& cfg);

    // Last dribble_speed command, [-1, 1]. Positive = capture direction: the
    // roller surface at the ball contact point moves toward the robot (local
    // -X) and pulls the ball in.
    void setTargetSpeed(float speed);

    // Apply this frame's capture force. Must run before Physics::step so the
    // force takes effect on the next stepSimulation (same contract as
    // Robot::applyDriveForces). dt is the same dt passed to Physics::step.
    void update(const Robot& robot, Ball& ball, float dt);

    // Actual (first-order-lagged, load-sagged) roller speed in RPM, signed like
    // dribble_speed. Published as SensorData.dribbler_rpm.
    float rpm() const;

private:
    // ---- Config (converted to SI once at load time) ----
    float m_maxSpeed = 0.0f;            // rad/s at dribble_speed = +/- 1.0
    float m_radiusCenter = 0.008f;      // m, roller radius at its lateral center
    float m_radiusEdge   = 0.012f;      // m, roller radius at the zone ends
    float m_length        = 0.07f;      // m, roller length along local Z
    float m_forwardOffset = 0.095f;     // m, roller axis forward of chassis center
    float m_heightOffset  = 0.015f;     // m, roller axis height above ground
    float m_captureToleranceForward = 0.015f; // m, +/- forward slack around forward_offset
    float m_captureToleranceHeight  = 0.010f; // m, +/- height slack around height_offset
    float m_friction = 1.2f;           // Coulomb coefficient (roller vs ball)
    float m_normalForce = 0.6f;        // N, effective press force between roller and ball
    float m_motorTimeConstant = 0.03f; // s, first-order lag
    float m_loadSagGain = 0.4f;        // 0..1, speed lost when a ball loads the roller
    float m_responseGain = 0.4f;       // 0..1, see Robot::applyDriveForces's m_frictionResponseGain

    // ---- State ----
    float m_targetSpeed = 0.0f;  // [-1, 1], last commanded dribble_speed
    float m_actualSpeed = 0.0f;  // rad/s, lagged + load-sagged
    float m_loadFraction = 0.0f; // 0..1, from the previous frame's applied force
};
