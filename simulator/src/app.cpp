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
    m_ball     = std::make_unique<Ball>();
    m_grpc     = std::make_unique<GrpcServer>();
    m_debugOverlay = std::make_unique<DebugOverlay>();

    m_physics->init(cfg);
    m_field->init(cfg);
    m_ball->init(cfg, m_physics->world());
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

    // The field itself (walls/ball/lighting) is ready; no robot exists yet --
    // see addRobot(). GrpcServer exists so addRobot() can register robots
    // into it as they're created, but it doesn't start serving until run()
    // (once the caller's whole team is registered).
    m_grpc->setBall(m_ball.get());

    // Lets a client's gRPC AddRobot call actually create a robot at runtime
    // (once the sim loop is running) instead of only "in code" before
    // run() -- GrpcServer::update() calls this on the sim/main thread when
    // it drains a queued request. See docs/tasks/multi-robot.md's "Dynamic
    // robot creation" section.
    m_grpc->setAddRobotHandler([this](const std::string& path, float x, float z, float yawDeg) {
        return addRobot(path, x, z, yawDeg);
    });

    std::cout << "[App] Simulator ready (field empty -- call addRobot() to add robots)." << std::endl;
    return true;
}

int App::addRobot(const std::string& robotConfigPath, float spawnX, float spawnZ, float spawnYawDeg)
{
    if (!m_physics || !m_renderer || !m_ball || !m_grpc) {
        std::cerr << "[App] addRobot() called before init()." << std::endl;
        return -1;
    }

    int id = (int)m_robots.size();

    RobotEntry entry;
    entry.id = id;
    try {
        entry.config = Config::instance().withRobotOverlay(robotConfigPath);
    } catch (const std::exception& e) {
        // loadRobot throws on a missing/unreadable file. A bad path from
        // main.cpp's --robot would previously just abort at startup (an
        // acceptable crash for a typo in a one-shot CLI arg), but this same
        // path now also runs mid-session from a gRPC AddRobot handler (see
        // App::init's setAddRobotHandler) -- there, letting the exception
        // propagate would crash the whole simulator (and every other
        // robot's session) over one bad client request. Report it back to
        // that one caller instead.
        std::cerr << "[App] addRobot(\"" << robotConfigPath << "\") failed: " << e.what() << std::endl;
        return -1;
    }
    if (spawnX != 0.0f || spawnZ != 0.0f || spawnYawDeg != 0.0f) {
        entry.config.setRobotSpawn(spawnX, spawnZ, spawnYawDeg);
    }

    entry.robot    = std::make_unique<Robot>();
    entry.camera   = std::make_unique<Camera>();
    entry.lidar    = std::make_unique<LidarSensor>();
    entry.dribbler = std::make_unique<Dribbler>();
    entry.kicker   = std::make_unique<Kicker>();
    entry.imu      = std::make_unique<ImuSensor>();

    entry.robot->init(entry.config, m_physics->world());
    entry.camera->init(entry.config, entry.robot->mirrorProfile(), entry.robot->cameraHeight(), m_renderer.get());
    entry.lidar->init(entry.config, m_physics->world());
    entry.dribbler->init(entry.config);
    entry.kicker->init(entry.config);
    entry.kicker->setActors(entry.robot.get(), m_ball.get());
    entry.imu->init(entry.config);

    m_grpc->registerRobot(id, entry.robot.get(), entry.camera.get(), entry.lidar.get(),
                           entry.dribbler.get(), entry.kicker.get(), entry.imu.get());

    m_robots.push_back(std::move(entry));
    std::cout << "[App] Robot " << id << " added from " << robotConfigPath << std::endl;
    return id;
}

void App::shutdown()
{
    if (m_grpc)         m_grpc->stop();
    for (auto& entry : m_robots) {
        if (entry.camera) entry.camera->shutdown();
    }
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

void App::setInitialPhase(int phase)
{
    if (m_grpc) m_grpc->setPhase(phase);
    if (phase == 0) {
        std::cout << "[Match] Waiting for robots -- press Enter in the window to start the game." << std::endl;
    }
}

void App::beginQuit()
{
    if (m_quitting) return;
    m_quitting = true;
    static constexpr Uint64 kQuitGraceMs = 1500;
    m_quitDeadline = SDL_GetTicks64() + kQuitGraceMs;
    m_grpc->setPhase(2);
    std::cout << "[Match] Stopping: telling robot programs to finish..." << std::endl;
}

void App::run()
{
    // Finalize the robot registry and start serving now that the caller has
    // made all its addRobot() calls (see GrpcServer's own comment on why the
    // registry must be fixed before its RPC handler threads start reading it).
    m_grpc->start();

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

        if (m_grpc->shutdownRequested()) m_running = false; // hard stop (Shutdown RPC)
        if (m_grpc->phase() == 2) beginQuit();              // SetGamePhase(STOPPING) from a client
        if (m_quitting && SDL_GetTicks64() >= m_quitDeadline) m_running = false;

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

        // Always, even when paused, so AddRobot RPCs never block on a pause.
        m_grpc->processPending();

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
        if (m_quitting) m_running = false; // second close request: don't wait
        else beginQuit();
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
        // 1/2/3/4 select which robot the keyboard (arrows/space/F below, plus
        // the camera preview/telemetry overlay) currently acts on -- a no-op
        // past however many robots addRobot() actually registered.
        if (e.key.keysym.scancode >= SDL_SCANCODE_1 && e.key.keysym.scancode <= SDL_SCANCODE_4) {
            int index = (int)(e.key.keysym.scancode - SDL_SCANCODE_1);
            if (index < (int)m_robots.size()) {
                m_activeRobot = index;
                std::cout << "[App] Active robot: " << m_robots[m_activeRobot].id << std::endl;
            }
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
            // Ground-plane pan (the orbit pivot must stay on the floor): the old
            // cross(right, forward) term was vertical, so vertical scrolling
            // lifted/sank the pivot instead of moving it forward/back.
            m_panVelocity += (-cameraForward() * sy - cameraRight() * sx) * m_panSensitivity * distScale;
        }
        break;
    }
    }
}

void App::handleKeyboardInput(const Uint8* keys, float dt)
{
    if (keys[SDL_SCANCODE_ESCAPE]) beginQuit();
    if (keys[SDL_SCANCODE_RETURN] && m_grpc->phase() == 0) {
        m_grpc->setPhase(1);
        std::cout << "[Match] Game started." << std::endl;
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

    if (m_robots.empty()) return;
    RobotEntry& active = m_robots[m_activeRobot];

    // Arrow keys + Q/E drive the active robot directly (manual testing,
    // independent of gRPC SendCommand). Only send a new target on
    // press/release edges so this doesn't stomp on commands an active gRPC
    // client is sending.
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

    if (drive != active.lastManualDrive || strafe != active.lastManualStrafe ||
        turn != active.lastManualTurn) {
        active.robot->setBodyVelocity(
            drive  * m_manualDriveSpeed,
            strafe * m_manualDriveSpeed,
            turn   * m_manualTurnSpeed);
        active.lastManualDrive  = drive;
        active.lastManualStrafe = strafe;
        active.lastManualTurn   = turn;
    }

    // Space = spin the dribbler at full capture speed while held, stop on
    // release. F = debug-fire the kicker at a fixed impulse, once per press
    // (edge-triggered — holding it down must not fire every frame). This is
    // the debug shortcut only: it bypasses the capacitor/electrical model
    // completely (see Kicker::debugFire) -- real client code drives the
    // kicker through open_capacitor()/close_capacitor()/open_kicker()/
    // close_kicker() over gRPC instead, same as SimRobotHAL.
    bool dribbleHeld = keys[SDL_SCANCODE_SPACE] != 0;
    if (dribbleHeld != active.lastManualDribbleHeld) {
        active.dribbler->setTargetSpeed(dribbleHeld ? 1.0f : 0.0f);
        active.lastManualDribbleHeld = dribbleHeld;
    }

    bool kickHeld = keys[SDL_SCANCODE_F] != 0;
    if (kickHeld && !active.lastManualKickHeld) {
        active.kicker->debugFire(*active.robot, *m_ball);
    }
    active.lastManualKickHeld = kickHeld;
}

void App::update(float dt)
{
    // Ordering matters: the dribbler capture force and the kicker impulse must
    // be applied before m_physics->step(dt) so they take effect on the next
    // stepSimulation (the dribbler's applyForce is consumed by the step; the
    // kicker's applyImpulse changes velocity immediately). Same reason
    // Robot::applyDriveForces runs before the step. All robots share one
    // physics step (one world), so this loop runs for every robot before the
    // single m_physics->step(dt) call below.
    for (auto& entry : m_robots) {
        entry.dribbler->update(*entry.robot, *m_ball, dt);
        entry.kicker->update(dt);
        entry.robot->applyDriveForces(dt);
    }
    m_physics->step(dt);
    for (auto& entry : m_robots) {
        entry.robot->syncFromPhysics();
    }
    m_ball->syncFromPhysics();
    for (auto& entry : m_robots) {
        entry.lidar->update(entry.robot->position(), entry.robot->orientation(), dt);
        entry.imu->update(*entry.robot, dt);
    }
    m_grpc->update(dt);
}

void App::render()
{
    // ---- Each robot's own mirror-camera pass (offscreen) ----
    for (auto& entry : m_robots) {
        Robot* self = entry.robot.get();
        entry.camera->renderView(self->position(), self->orientation(),
            [&](Renderer& r) {
                m_field->render(r);
                r.drawShadowBlob(m_ball->position(), m_ball->radius());
                r.drawShadowBlob(self->position(), self->diameter() * 0.5f);
                self->renderCameraFrame(r); // pillars only, not the full body — see Robot.h
                // Every other robot on the field is a real obstacle this
                // camera should see too (a vision-league robot's own camera
                // sees teammates/opponents, not just the ball) -- drawn as
                // renderBody() (body + wheels, same wireframe style the
                // debug viewer already uses for it), not the stripped
                // support-frame the camera owner draws for itself above.
                for (auto& other : m_robots) {
                    if (other.robot.get() == self) continue;
                    r.drawShadowBlob(other.robot->position(), other.robot->diameter() * 0.5f);
                    other.robot->renderBody(r);
                }
                m_ball->render(r);
            });
    }
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
    for (auto& entry : m_robots) {
        m_renderer->drawShadowBlob(entry.robot->position(), entry.robot->diameter() * 0.5f);
        entry.robot->render(*m_renderer);
    }
    m_ball->render(*m_renderer);
    if (m_showPhysicsDebug) {
        m_physics->debugDraw(*m_renderer);
    }

    m_renderer->endFrame();

    // ---- Camera preview overlay (active robot only) ----
    if (m_showCameraPreview && !m_robots.empty()) {
        glViewport(0, 0, m_width, m_height);
        m_robots[m_activeRobot].camera->drawPreview(m_width, m_height);
    }

    // ---- Telemetry overlay (L key, active robot only) ----
    if (m_showTelemetryOverlay && m_debugOverlay && !m_robots.empty()) {
        glViewport(0, 0, m_width, m_height);

        RobotEntry& active = m_robots[m_activeRobot];
        glm::vec3 pos = active.robot->position();
        glm::vec3 ballPos = m_ball->position();
        float yaw = active.robot->orientation();
        float cosYaw = std::cos(yaw), sinYaw = std::sin(yaw);
        float dx = ballPos.x - pos.x, dz = ballPos.z - pos.z;
        float ballLocalX = cosYaw * dx - sinYaw * dz;
        float ballLocalZ = sinYaw * dx + cosYaw * dz;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        std::vector<std::string> lines;

        ss.str(""); ss << "robot " << active.id << " / " << m_robots.size();
        lines.push_back(ss.str());
        ss.str(""); ss << "pos x=" << pos.x << " z=" << pos.z << " yaw=" << yaw;
        lines.push_back(ss.str());
        ss.str(""); ss << "vel=" << active.robot->velocity() << " m/s  omega=" << active.robot->angularVelocity() << " rad/s";
        lines.push_back(ss.str());
        ss.str(""); ss << "odom x=" << active.robot->odometryX() << " z=" << active.robot->odometryZ() << " yaw=" << active.robot->odometryYaw();
        lines.push_back(ss.str());
        ss.str(""); ss << "dribbler rpm=" << std::setprecision(0) << active.dribbler->rpm();
        lines.push_back(ss.str());
        ss.str(""); ss << std::setprecision(3) << "cap=" << active.kicker->capacitorVoltage() << "V ("
            << active.kicker->charge() * 100.0f << "%)  bus=" << active.kicker->busVoltage() << "V";
        lines.push_back(ss.str());
        ss.str(""); ss << "ball local fwd=" << ballLocalX << " lat=" << ballLocalZ;
        lines.push_back(ss.str());
        ss.str(""); ss << "imu rpy=" << active.imu->roll() << "," << active.imu->pitch() << "," << active.imu->yaw()
            << "  gyro_y=" << active.imu->angularVelocity().y;
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
    // Screen-right on the ground plane (forward x up). The old
    // cross(camera - target, up) pointed to screen-LEFT.
    return glm::normalize(glm::cross(
        m_lookTarget - cameraPosition(),
        glm::vec3(0.0f, 1.0f, 0.0f)
    ));
}

glm::vec3 App::cameraForward() const
{
    glm::vec3 fwd = glm::normalize(m_lookTarget - cameraPosition());
    fwd.y = 0.0f;
    return glm::length(fwd) > 0.001f ? glm::normalize(fwd) : glm::vec3(0.0f, 0.0f, -1.0f);
}
