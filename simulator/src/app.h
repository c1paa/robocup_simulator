#pragma once

#include "common.h"
#include "config.h"
#include <SDL2/SDL.h>
#include <memory>
#include <string>
#include <vector>

class Renderer;
class Physics;
class Field;
class Robot;
class Ball;
class Camera;
class LidarSensor;
class Dribbler;
class Kicker;
class ImuSensor;
class GrpcServer;
class DebugOverlay;

// One robot on the field, plus everything it owns individually (its own
// camera, lidar, dribbler, kicker, IMU, and its own slice of Config -- see
// App::addRobot). The ball is the one thing NOT in here: there's only one on
// the field, shared by every robot, owned directly by App.
struct RobotEntry
{
    int id = 0;
    Config config; // this robot's own /robot/* and /camera/* tree (see Config::withRobotOverlay)

    std::unique_ptr<Robot> robot;
    std::unique_ptr<Camera> camera;
    std::unique_ptr<LidarSensor> lidar;
    std::unique_ptr<Dribbler> dribbler;
    std::unique_ptr<Kicker> kicker;
    std::unique_ptr<ImuSensor> imu;

    // Manual-keyboard edge-state (see App::handleKeyboardInput) -- kept
    // per-robot so switching the active robot with 1/2/3/4 doesn't cause a
    // spurious edge transition on either robot involved.
    float lastManualDrive  = 0.0f;
    float lastManualStrafe = 0.0f;
    float lastManualTurn   = 0.0f;
    bool lastManualDribbleHeld = false;
    bool lastManualKickHeld    = false;
};

class App
{
public:
    App();
    ~App();

    bool init(const std::string& configDir = "configs/");
    void run();
    void shutdown();

    // Constructs a new robot from its own config file (merged on top of the
    // shared project config -- see Config::withRobotOverlay) and adds it to
    // the field; this is the "in code" way to build up a team (own robots
    // and/or opponents), one call per robot. spawnX/spawnZ (millimetres) and
    // spawnYawDeg override the robot's own config file's /robot/spawn/* (use
    // this to place several robots spawned from the *same* config file
    // without overlapping -- otherwise they all start at the file's own
    // spawn point, default the origin).
    //
    // Safe to call both before App::run() (the usual "build the whole team
    // up front" case -- main.cpp does this) and after, from the main/sim
    // thread only: GrpcServer's AddRobot RPC handler (any gRPC thread) calls
    // this indirectly, via the callback GrpcServer::update() invokes on the
    // main thread once per frame (see App::init's setAddRobotHandler and
    // docs/tasks/multi-robot.md's "Dynamic robot creation" section) --
    // Bullet/GL construction must happen there, not on a gRPC handler thread.
    // Never call this directly from any other thread. Returns the new
    // robot's id (its RobotCommand/SensorRequest.robot_id), assigned in
    // call order starting at 0.
    int addRobot(const std::string& robotConfigPath,
                 float spawnX = 0.0f, float spawnZ = 0.0f, float spawnYawDeg = 0.0f);

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
    std::unique_ptr<Ball> m_ball;
    std::unique_ptr<GrpcServer> m_grpc;
    std::unique_ptr<DebugOverlay> m_debugOverlay;

    // The field starts empty; addRobot() populates this. Stored by value in
    // a vector (not vector<unique_ptr<RobotEntry>>) -- the Robot/Camera/etc.
    // pointers handed to GrpcServer are the heap addresses their own
    // unique_ptr members hold, which stay valid even if the vector itself
    // reallocates when a later addRobot() grows it.
    std::vector<RobotEntry> m_robots;

    // Which m_robots[] index the keyboard (arrows/space/F) currently drives,
    // selected with the 1/2/3/4 keys (see handleEvent). Only meaningful when
    // !m_robots.empty().
    int m_activeRobot = 0;

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
    bool m_showTelemetryOverlay = false;

    // Manual robot drive (arrow keys + Q/E) — independent of gRPC SendCommand.
    // Only sent on press/release edges so it doesn't fight an active gRPC
    // client by re-issuing a command every frame.
    // press -- fixed impulse, bypasses the capacitor entirely, see Kicker::
    // debugFire). Both act on m_robots[m_activeRobot]; press/release-edge
    // state for both lives on RobotEntry itself (see its own comment).
    float m_manualDriveSpeed = 4.0f;
    float m_manualTurnSpeed  = 2.5f;
};
