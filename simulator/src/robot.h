#pragma once

#include "common.h"
#include "mirror_profile.h"

class Config;
class Renderer;

class Robot
{
public:
    void init(Config& cfg);
    void update(float dt);
    void render(Renderer& renderer);
    void renderBody(Renderer& renderer); // body + wheels only (no mirror/camera)

    // gRPC-to-robot commands
    void setWheelVelocities(float left, float right);
    void kick(float power);
    void dribble(float speed);

    // Robot state (for gRPC)
    glm::vec3 position() const { return m_position; }
    float orientation() const { return m_yaw; }
    float velocity() const { return m_velocity; }
    float angularVelocity() const { return m_angularVelocity; }

    // Mirror / camera geometry (single source of truth for mirror shape)
    const MirrorProfile& mirrorProfile() const { return m_mirror; }
    float cameraHeight() const { return m_cameraHeight; }

private:
    // Position in world
    glm::vec3 m_position = glm::vec3(0.0f, 0.04f, 0.0f);
    float m_yaw = 0.0f;
    float m_velocity = 0.0f;
    float m_angularVelocity = 0.0f;

    // Target velocities from gRPC
    float m_targetLeftWheel  = 0.0f;
    float m_targetRightWheel = 0.0f;

    // ---- Robot geometry (meters) ----
    float m_diameter = 0.18f;
    float m_height   = 0.15f;

    // ---- Wheels ----
    int   m_wheelCount         = 4;
    float m_wheelCenterDiameter = 0.15f;  // circle on which wheel centers sit
    float m_wheelDiameter       = 0.07f;  // diameter of each wheel

    // ---- Motor parameters ----
    float m_maxSpeed          = 3.0f;
    float m_motorTimeConstant = 0.05f;
    float m_wheelSlip         = 0.05f;

    // ---- Mirror (cone or hyperbola, apex down toward camera) ----
    MirrorProfile m_mirror;

    // ---- Camera (looks up into mirror) ----
    float m_cameraHeight = 0.11f;

    // ---- Colors ----
    glm::vec3 m_bodyColor   = glm::vec3(0.1f, 0.3f, 0.8f);
    glm::vec3 m_wheelColor  = glm::vec3(0.1f, 0.1f, 0.1f);
    glm::vec3 m_mirrorColor = glm::vec3(0.7f, 0.7f, 0.8f);
};
