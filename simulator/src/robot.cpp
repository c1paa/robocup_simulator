#include "robot.h"
#include "renderer.h"
#include "config.h"
#include <algorithm>
#include <cmath>

static const float MM = 1000.0f;

void Robot::init(Config& cfg)
{
    // Geometry
    m_diameter = cfg.getFloat("/robot/diameter", 180.0f) / MM;
    m_height   = cfg.getFloat("/robot/height",   150.0f) / MM;

    // Wheels
    m_wheelCount          = cfg.getInt("/robot/wheels/count",           4);
    m_wheelCenterDiameter = cfg.getFloat("/robot/wheels/center_diameter", 150.0f) / MM;
    m_wheelDiameter       = cfg.getFloat("/robot/wheels/wheel_diameter",  70.0f)  / MM;

    // Mirror
    m_mirrorBaseHeight   = cfg.getFloat("/robot/mirror/base_height",   150.0f) / MM;
    m_mirrorHeight       = cfg.getFloat("/robot/mirror/cone_height",   40.0f)  / MM;
    m_mirrorBaseDiameter = cfg.getFloat("/robot/mirror/base_diameter", 80.0f)  / MM;

    // Camera
    m_cameraHeight = cfg.getFloat("/robot/camera/height", 110.0f) / MM;

    // Motor
    m_maxSpeed          = cfg.getFloat("/robot/motor/max_speed",       3000.0f) / MM;
    m_motorTimeConstant = cfg.getFloat("/robot/motor/time_constant",   0.05f);
    m_wheelSlip         = cfg.getFloat("/robot/motor/slip",            0.05f);

    // Body sits on ground: bottom at y=0, center at y = height/2
    m_position.y = m_height * 0.5f;
}

void Robot::setWheelVelocities(float left, float right)
{
    m_targetLeftWheel  = std::clamp(left,  -m_maxSpeed, m_maxSpeed);
    m_targetRightWheel = std::clamp(right, -m_maxSpeed, m_maxSpeed);
}

void Robot::kick(float power)
{
    (void)power;
}

void Robot::dribble(float speed)
{
    (void)speed;
}

void Robot::update(float dt)
{
    float wheelRadius = m_wheelDiameter * 0.5f;
    float wheelBase   = m_wheelCenterDiameter;
    float alpha = dt / (m_motorTimeConstant + dt);

    // Current wheel velocities from robot motion (for smoother blending)
    float curLeft  = (m_velocity - m_angularVelocity * wheelBase * 0.5f) / wheelRadius;
    float curRight = (m_velocity + m_angularVelocity * wheelBase * 0.5f) / wheelRadius;

    float leftWheel  = util::lerp(curLeft,  m_targetLeftWheel,  alpha);
    float rightWheel = util::lerp(curRight, m_targetRightWheel, alpha);

    // Apply slip
    leftWheel  *= (1.0f - m_wheelSlip);
    rightWheel *= (1.0f - m_wheelSlip);

    // Differential drive kinematics
    float linearVel  = (leftWheel + rightWheel) * 0.5f * wheelRadius;
    float angularVel = (rightWheel - leftWheel) * wheelRadius / wheelBase;

    m_velocity = linearVel;
    m_angularVelocity = angularVel;

    m_position.x += linearVel * std::cos(m_yaw) * dt;
    m_position.z += linearVel * std::sin(m_yaw) * dt;
    m_yaw += angularVel * dt;
}

void Robot::render(Renderer& renderer)
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

    // ---- Wheels (evenly spaced, vertical disks) ----
    float angleStep = (2.0f * glm::pi<float>()) / (float)m_wheelCount;
    for (int i = 0; i < m_wheelCount; i++) {
        float angle = (float)i * angleStep;
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

    // ---- Mirror (cone, apex down toward camera) ----
    {
        float apexY = m_mirrorBaseHeight - m_mirrorHeight;
        float mirrorCenterY = (m_mirrorBaseHeight + apexY) * 0.5f;
        float mirrorLocalY = mirrorCenterY - bodyHalf;
        glm::mat4 mirrorLocal = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, mirrorLocalY, 0.0f));
        glm::mat4 mirrorModel = model * mirrorLocal;
        renderer.drawCone(m_mirrorBaseDiameter * 0.5f, m_mirrorHeight, mirrorModel, m_mirrorColor);
    }

    // ---- Camera indicator ----
    {
        float camLocalY = m_cameraHeight - bodyHalf;
        glm::mat4 camLocal = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, camLocalY, 0.0f));
        glm::mat4 camModel = model * camLocal;
        renderer.drawSphere(0.015f, camModel, glm::vec3(1.0f, 0.2f, 0.2f));
    }
}
