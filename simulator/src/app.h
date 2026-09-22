#pragma once

#include "common.h"
#include <SDL2/SDL.h>
#include <memory>
#include <string>

class Renderer;
class Physics;
class Field;
class Robot;
class Camera;
class GrpcServer;

class App
{
public:
    App();
    ~App();

    bool init(const std::string& configDir = "configs/");
    void run();
    void shutdown();

private:
    bool initSDL();
    bool initGL();
    void handleEvent(const SDL_Event& e);
    void update(float dt);
    void render();
    void handleKeyboardInput(const Uint8* keys, float dt);

    // Window dimensions
    int m_width = 1280;
    int m_height = 720;

    // SDL
    SDL_Window* m_window = nullptr;
    SDL_GLContext m_glContext = nullptr;
    bool m_running = true;
    bool m_paused = false;

    // Subsystems
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<Physics> m_physics;
    std::unique_ptr<Field> m_field;
    std::unique_ptr<Robot> m_robot;
    std::unique_ptr<Camera> m_camera;
    std::unique_ptr<GrpcServer> m_grpc;

    // Timing
    Uint64 m_lastTick = 0;
    float m_simulationSpeed = 1.0f;

    // Camera for 3D viewer (not robot camera)
    glm::vec3 m_lookTarget = glm::vec3(0.0f, 0.0f, 0.0f);
    float m_viewerYaw = -135.0f;
    float m_viewerPitch = -30.0f;
    float m_cameraDistance = 10.0f;

    // Camera helpers
    glm::vec3 cameraPosition() const;
    glm::vec3 cameraRight() const;
    glm::vec3 cameraForward() const;

    bool m_shiftHeld = false;
    bool m_cmdHeld = false;

    // Smooth pan/orbit velocity
    glm::vec3 m_panVelocity = glm::vec3(0.0f);
    glm::vec2 m_orbitVelocity = glm::vec2(0.0f);  // (yaw, pitch)
    float m_zoomVelocity = 0.0f;

    // Viewer settings (loaded from config)
    float m_panSensitivity = 0.5f;
    float m_orbitSensitivity = 0.15f;
    float m_zoomSensitivity = 0.8f;
    float m_damping = 12.0f;
    float m_panSpeed = 3.0f;
    float m_orbitSpeed = 5.0f;
    float m_zoomSpeed = 3.0f;
    float m_minDistance = 2.0f;
    float m_maxDistance = 60.0f;

    bool m_showGrid = true;
    bool m_showPhysicsDebug = false;
    bool m_showCameraPreview = false;

    // Manual robot drive (arrow keys) — independent of gRPC SendCommand.
    // Only sent on press/release edges so it doesn't fight an active gRPC
    // client by re-issuing a command every frame.
    float m_manualDriveSpeed = 4.0f;
    float m_lastManualDrive = 0.0f;
    float m_lastManualTurn  = 0.0f;
};
