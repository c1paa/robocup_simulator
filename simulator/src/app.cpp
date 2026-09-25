#include "app.h"
#include "renderer.h"
#include "physics.h"
#include "field.h"
#include "robot.h"
#include "ball.h"
#include "camera.h"
#include "lidar_sensor.h"
#include "dribbler.h"
#include "kicker.h"
#include "imu_sensor.h"
#include "config.h"
#include "grpc_server.h"
#include "debug_overlay.h"

#include <iostream>
#include <cmath>
#include <sstream>
#include <iomanip>

App::App() = default;

App::~App()
{
    shutdown();
}

bool App::init(const std::string& configDir)
{
    Config& cfg = Config::instance();
    cfg.setConfigDir(configDir);

    try {
        cfg.loadProject(configDir + "project.json");
        cfg.loadRobot(configDir + "robot.json");
    } catch (const std::exception& e) {
        std::cerr << "[Config] Warning: " << e.what() << ". Using defaults." << std::endl;
    }

    if (!initSDL()) return false;
    if (!initGL())  return false;

    m_panSensitivity   = cfg.getFloat("/viewer/pan_sensitivity",   0.5f);
    m_orbitSensitivity = cfg.getFloat("/viewer/orbit_sensitivity", 0.15f);
    m_zoomSensitivity  = cfg.getFloat("/viewer/zoom_sensitivity",  0.8f);
    m_damping          = cfg.getFloat("/viewer/damping",           12.0f);
    m_panSpeed         = cfg.getFloat("/viewer/pan_speed",         3.0f);
    m_orbitSpeed       = cfg.getFloat("/viewer/orbit_speed",       5.0f);
    m_zoomSpeed        = cfg.getFloat("/viewer/zoom_speed",        3.0f);
    m_minDistance      = cfg.getFloat("/viewer/min_distance",      2.0f);
    m_maxDistance      = cfg.getFloat("/viewer/max_distance",      60.0f);
    m_showGrid         = cfg.getInt("/viewer/show_grid",           1) != 0;
    m_showPhysicsDebug  = cfg.getInt("/viewer/show_physics_debug", 0) != 0;

    m_renderer = std::make_unique<Renderer>();
    m_physics  = std::make_unique<Physics>();
    m_field    = std::make_unique<Field>();
    m_robot    = std::make_unique<Robot>();
    m_ball     = std::make_unique<Ball>();
    m_camera   = std::make_unique<Camera>();
    m_lidar    = std::make_unique<LidarSensor>();
    m_dribbler = std::make_unique<Dribbler>();
    m_kicker   = std::make_unique<Kicker>();
    m_imu      = std::make_unique<ImuSensor>();
    m_grpc     = std::make_unique<GrpcServer>();
    m_debugOverlay = std::make_unique<DebugOverlay>();

    m_physics->init(cfg);
    m_field->init(cfg);
    m_robot->init(cfg, m_physics->world());
    m_ball->init(cfg, m_physics->world());
    m_camera->init(cfg, m_robot->mirrorProfile(), m_robot->cameraHeight(), m_renderer.get());
    m_lidar->init(cfg, m_physics->world());
    m_dribbler->init(cfg);
    m_kicker->init(cfg);
    m_kicker->setActors(m_robot.get(), m_ball.get());
    m_imu->init(cfg);
    m_debugOverlay->init();
    m_renderer->init(m_width, m_height);
    m_renderer->setLighting(
        glm::vec3(cfg.getFloat("/scene/light/direction/0", 0.5f),
                  cfg.getFloat("/scene/light/direction/1", -1.0f),
                  cfg.getFloat("/scene/light/direction/2", 0.3f)),
        glm::vec3(cfg.getFloat("/scene/light/ambient/0", 0.25f),
                  cfg.getFloat("/scene/light/ambient/1", 0.25f),
                  cfg.getFloat("/scene/light/ambient/2", 0.28f)),
        glm::vec3(cfg.getFloat("/scene/light/diffuse/0", 0.85f),
                  cfg.getFloat("/scene/light/diffuse/1", 0.83f),
                  cfg.getFloat("/scene/light/diffuse/2", 0.78f)));
    m_grpc->setRobot(m_robot.get());
    m_grpc->setCamera(m_camera.get());
    m_grpc->setLidar(m_lidar.get());
    m_grpc->setBall(m_ball.get());
    m_grpc->setDribbler(m_dribbler.get());
    m_grpc->setKicker(m_kicker.get());
    m_grpc->setImu(m_imu.get());
    m_grpc->start();

    std::cout << "[App] Simulator ready." << std::endl;
    return true;
}

void App::shutdown()
{
    if (m_grpc)         m_grpc->stop();
    if (m_camera)       m_camera->shutdown();
    if (m_debugOverlay) m_debugOverlay->shutdown();
    if (m_renderer)     m_renderer->shutdown();
    if (m_glContext) SDL_GL_DeleteContext(m_glContext);
    if (m_window)    SDL_DestroyWindow(m_window);

    SDL_Quit();

    m_window   = nullptr;
    m_glContext = nullptr;
}

bool App::initSDL()
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
        std::cerr << "[SDL] Init failed: " << SDL_GetError() << std::endl;
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    m_window = SDL_CreateWindow(
        "RoboCup Simulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        m_width, m_height,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
    );

    if (!m_window) {
        std::cerr << "[SDL] Window creation failed: " << SDL_GetError() << std::endl;
        return false;
    }

    m_glContext = SDL_GL_CreateContext(m_window);
    if (!m_glContext) {
        std::cerr << "[SDL] GL context creation failed: " << SDL_GetError() << std::endl;
        return false;
    }

    SDL_GL_SetSwapInterval(1); // VSync

    std::cout << "[SDL] Window " << m_width << "x" << m_height << " created." << std::endl;
    return true;
}

bool App::initGL()
{
    // GL is initialized by SDL context creation above
    // On macOS, we need GLAD or similar, but we use OpenGL 3.3 core
    std::cout << "[OpenGL] Vendor:   " << glGetString(GL_VENDOR)   << std::endl;
    std::cout << "[OpenGL] Renderer: " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "[OpenGL] Version:  " << glGetString(GL_VERSION)  << std::endl;

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.1f, 0.12f, 0.15f, 1.0f);

    return true;
}

void App::run()
{
    m_lastTick = SDL_GetTicks64();

    while (m_running) {
        Uint64 now = SDL_GetTicks64();
        float dt = (now - m_lastTick) / 1000.0f;
        m_lastTick = now;

        // Cap dt to avoid spiral of death
        if (dt > 0.1f) dt = 0.1f;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            handleEvent(e);
        }

        const Uint8* keys = SDL_GetKeyboardState(nullptr);
        handleKeyboardInput(keys, dt);

        // --- Smooth camera damping (always, even when paused) ---
        float damping = std::exp(-m_damping * dt);

        m_lookTarget += m_panVelocity * dt * m_panSpeed;
        m_panVelocity *= damping;

        m_viewerYaw   += m_orbitVelocity.x * dt * m_orbitSpeed;
        m_viewerPitch += m_orbitVelocity.y * dt * m_orbitSpeed;
        m_viewerPitch = glm::clamp(m_viewerPitch, -89.0f, -5.0f);
        m_orbitVelocity *= damping;

        float newDist = m_cameraDistance + m_zoomVelocity * dt * m_zoomSpeed;
        if (newDist >= m_minDistance && newDist <= m_maxDistance) {
            m_cameraDistance = newDist;
        }
        m_zoomVelocity *= damping;

        if (!m_paused) {
            update(dt * m_simulationSpeed);
        }

        render();
        SDL_GL_SwapWindow(m_window);
    }
}

void App::handleEvent(const SDL_Event& e)
{
    switch (e.type) {
    case SDL_QUIT:
        m_running = false;
        break;
    case SDL_WINDOWEVENT:
        if (e.window.event == SDL_WINDOWEVENT_RESIZED) {
            m_width  = e.window.data1;
            m_height = e.window.data2;
            glViewport(0, 0, m_width, m_height);
        }
        break;

    case SDL_KEYDOWN:
        if (e.key.keysym.scancode == SDL_SCANCODE_LSHIFT ||
            e.key.keysym.scancode == SDL_SCANCODE_RSHIFT) {
            m_shiftHeld = true;
        }
        if (e.key.keysym.scancode == SDL_SCANCODE_LGUI ||
            e.key.keysym.scancode == SDL_SCANCODE_RGUI) {
            m_cmdHeld = true;
        }
        if (e.key.keysym.scancode == SDL_SCANCODE_C) {
            m_showCameraPreview = !m_showCameraPreview;
        }
        if (e.key.keysym.scancode == SDL_SCANCODE_L) {
            m_showTelemetryOverlay = !m_showTelemetryOverlay;
        }
        break;
    case SDL_KEYUP:
        if (e.key.keysym.scancode == SDL_SCANCODE_LSHIFT ||
            e.key.keysym.scancode == SDL_SCANCODE_RSHIFT) {
            m_shiftHeld = false;
        }
        if (e.key.keysym.scancode == SDL_SCANCODE_LGUI ||
            e.key.keysym.scancode == SDL_SCANCODE_RGUI) {
            m_cmdHeld = false;
        }
        break;

    case SDL_MOUSEMOTION:
        if (e.motion.state & SDL_BUTTON(SDL_BUTTON_RIGHT)) {
            m_orbitVelocity.x -= e.motion.xrel * m_orbitSensitivity * 0.15f;
            m_orbitVelocity.y += e.motion.yrel * m_orbitSensitivity * 0.15f;
        }
        break;

    case SDL_MOUSEWHEEL: {
        float sx = e.wheel.x;
        float sy = e.wheel.y;

        if (e.wheel.direction == SDL_MOUSEWHEEL_FLIPPED) {
            sx = -sx;
            sy = -sy;
        }

        bool cmd   = m_cmdHeld   || (SDL_GetModState() & KMOD_GUI);
        bool shift = m_shiftHeld || (SDL_GetModState() & KMOD_SHIFT);

        if (cmd) {
            m_zoomVelocity -= sy * m_zoomSensitivity;
        } else if (shift) {
            m_orbitVelocity.x -= sx * m_orbitSensitivity;
            m_orbitVelocity.y += sy * m_orbitSensitivity;
        } else {
            // Pan speed scales with distance: close = slow, far = fast
            float distScale = m_cameraDistance / 5.0f;
            m_panVelocity += cameraRight() * sx * m_panSensitivity * distScale
                          +  glm::cross(cameraRight(), cameraForward()) * sy * m_panSensitivity * distScale;
        }
        break;
    }
    }
}

void App::handleKeyboardInput(const Uint8* keys, float dt)
{
    if (keys[SDL_SCANCODE_ESCAPE]) {
        m_running = false;
    }

    // WASD to pan the look target in camera plane
    float speed = 5.0f * dt;
    if (m_shiftHeld) speed *= 3.0f;

    glm::vec3 forward = cameraForward();
    glm::vec3 right   = cameraRight();

    if (keys[SDL_SCANCODE_W]) m_lookTarget += forward * speed;
    if (keys[SDL_SCANCODE_S]) m_lookTarget -= forward * speed;
    if (keys[SDL_SCANCODE_A]) m_lookTarget -= right * speed;
    if (keys[SDL_SCANCODE_D]) m_lookTarget += right * speed;

    // Arrow keys + Q/E drive the robot directly (manual testing, independent of
    // gRPC SendCommand). Only send a new target on press/release edges so
    // this doesn't stomp on commands an active gRPC client is sending.
    float drive = 0.0f, strafe = 0.0f, turn = 0.0f;
    if (keys[SDL_SCANCODE_UP])    drive += 1.0f;
    if (keys[SDL_SCANCODE_DOWN])  drive -= 1.0f;
    // Signs here are the manual-drive keybinding only (arbitrary by nature,
    // just needs to match what feels intuitive from the viewer window) — not
    // the underlying omega/yaw convention, which stays as documented in
    // AGENTS.md and is what the lidar/camera/odometry all agree on.
    if (keys[SDL_SCANCODE_LEFT])  turn  += 1.0f;
    if (keys[SDL_SCANCODE_RIGHT]) turn  -= 1.0f;
    if (keys[SDL_SCANCODE_Q])     strafe -= 1.0f;
    if (keys[SDL_SCANCODE_E])     strafe += 1.0f;

    if (m_robot && (drive != m_lastManualDrive || strafe != m_lastManualStrafe ||
                    turn != m_lastManualTurn)) {
        m_robot->setBodyVelocity(
            drive  * m_manualDriveSpeed,
            strafe * m_manualDriveSpeed,
            turn   * m_manualTurnSpeed);
        m_lastManualDrive  = drive;
        m_lastManualStrafe = strafe;
        m_lastManualTurn   = turn;
    }

    // Space = spin the dribbler at full capture speed while held, stop on
    // release. F = debug-fire the kicker at a fixed impulse, once per press
    // (edge-triggered — holding it down must not fire every frame). This is
    // the debug shortcut only: it bypasses the capacitor/electrical model
    // completely (see Kicker::debugFire) -- real client code drives the
    // kicker through open_capacitor()/close_capacitor()/open_kicker()/
    // close_kicker() over gRPC instead, same as SimRobotHAL.
    bool dribbleHeld = keys[SDL_SCANCODE_SPACE] != 0;
    if (m_dribbler && dribbleHeld != m_lastManualDribbleHeld) {
        m_dribbler->setTargetSpeed(dribbleHeld ? 1.0f : 0.0f);
        m_lastManualDribbleHeld = dribbleHeld;
    }

    bool kickHeld = keys[SDL_SCANCODE_F] != 0;
    if (kickHeld && !m_lastManualKickHeld && m_kicker && m_robot && m_ball) {
        m_kicker->debugFire(*m_robot, *m_ball);
    }
    m_lastManualKickHeld = kickHeld;
}

void App::update(float dt)
{
    // Ordering matters: the dribbler capture force and the kicker impulse must
    // be applied before m_physics->step(dt) so they take effect on the next
    // stepSimulation (the dribbler's applyForce is consumed by the step; the
    // kicker's applyImpulse changes velocity immediately). Same reason
    // Robot::applyDriveForces runs before the step.
    m_dribbler->update(*m_robot, *m_ball, dt);
    m_kicker->update(dt);
    m_robot->applyDriveForces(dt);
    m_physics->step(dt);
    m_robot->syncFromPhysics();
    m_ball->syncFromPhysics();
    m_lidar->update(m_robot->position(), m_robot->orientation(), dt);
    m_imu->update(*m_robot, dt);
    m_grpc->update(dt);
}

void App::render()
{
    // ---- Robot mirror-camera pass (offscreen) ----
    m_camera->renderView(m_robot->position(), m_robot->orientation(),
        [&](Renderer& r) {
            m_field->render(r);
            r.drawShadowBlob(m_ball->position(), m_ball->radius());
            r.drawShadowBlob(m_robot->position(), m_robot->diameter() * 0.5f);
            m_robot->renderCameraFrame(r); // pillars only, not the full body — see Robot.h
            m_ball->render(r);
        });
    glViewport(0, 0, m_width, m_height);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Build viewer camera matrices
    glm::mat4 projection = glm::perspective(
        glm::radians(50.0f),
        (float)m_width / (float)m_height,
        0.1f, 100.0f
    );

    glm::vec3 cameraPos = cameraPosition();

    glm::mat4 view = glm::lookAt(cameraPos, m_lookTarget, glm::vec3(0.0f, 1.0f, 0.0f));

    m_renderer->beginFrame(projection, view);

    if (m_showGrid) {
        float gs = std::max(m_field->length(), m_field->width()) + 2.0f;
        m_renderer->drawGrid(gs, 1.0f);
    }
    m_field->render(*m_renderer);
    m_renderer->drawShadowBlob(m_ball->position(), m_ball->radius());
    m_renderer->drawShadowBlob(m_robot->position(), m_robot->diameter() * 0.5f);
    m_robot->render(*m_renderer);
    m_ball->render(*m_renderer);
    if (m_showPhysicsDebug) {
        m_physics->debugDraw(*m_renderer);
    }

    m_renderer->endFrame();

    // ---- Camera preview overlay ----
    if (m_showCameraPreview) {
        glViewport(0, 0, m_width, m_height);
        m_camera->drawPreview(m_width, m_height);
    }

    // ---- Telemetry overlay (L key) ----
    if (m_showTelemetryOverlay && m_debugOverlay) {
        glViewport(0, 0, m_width, m_height);

        glm::vec3 pos = m_robot->position();
        glm::vec3 ballPos = m_ball->position();
        float yaw = m_robot->orientation();
        float cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);
        float dx = ballPos.x - pos.x, dz = ballPos.z - pos.z;
        float ballLocalX = cosYaw * dx - sinYaw * dz;
        float ballLocalZ = sinYaw * dx + cosYaw * dz;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        std::vector<std::string> lines;

        ss.str(""); ss << "pos x=" << pos.x << " z=" << pos.z << " yaw=" << yaw;
        lines.push_back(ss.str());
        ss.str(""); ss << "vel=" << m_robot->velocity() << " m/s  omega=" << m_robot->angularVelocity() << " rad/s";
        lines.push_back(ss.str());
        ss.str(""); ss << "odom x=" << m_robot->odometryX() << " z=" << m_robot->odometryZ() << " yaw=" << m_robot->odometryYaw();
        lines.push_back(ss.str());
        ss.str(""); ss << "dribbler rpm=" << std::setprecision(0) << m_dribbler->rpm();
        lines.push_back(ss.str());
        ss.str(""); ss << std::setprecision(3) << "cap=" << m_kicker->capacitorVoltage() << "V ("
            << m_kicker->charge() * 100.0f << "%)  bus=" << m_kicker->busVoltage() << "V";
        lines.push_back(ss.str());
        ss.str(""); ss << "ball local fwd=" << ballLocalX << " lat=" << ballLocalZ;
        lines.push_back(ss.str());
        ss.str(""); ss << "imu rpy=" << m_imu->roll() << "," << m_imu->pitch() << "," << m_imu->yaw()
            << "  gyro_y=" << m_imu->angularVelocity().y;
        lines.push_back(ss.str());

        m_debugOverlay->draw(m_width, m_height, lines);
    }
}

glm::vec3 App::cameraPosition() const
{
    float yawRad   = glm::radians(m_viewerYaw);
    float pitchRad = glm::radians(m_viewerPitch);

    return m_lookTarget + glm::vec3(
        m_cameraDistance * std::cos(pitchRad) * std::sin(yawRad),
        m_cameraDistance * std::sin(-pitchRad),
        m_cameraDistance * std::cos(pitchRad) * std::cos(yawRad)
    );
}

glm::vec3 App::cameraRight() const
{
    return glm::normalize(glm::cross(
        cameraPosition() - m_lookTarget,
        glm::vec3(0.0f, 1.0f, 0.0f)
    ));
}

glm::vec3 App::cameraForward() const
{
    glm::vec3 fwd = glm::normalize(m_lookTarget - cameraPosition());
    fwd.y = 0.0f;
    return glm::length(fwd) > 0.001f ? glm::normalize(fwd) : glm::vec3(0.0f, 0.0f, -1.0f);
}
