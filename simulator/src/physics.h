#pragma once

#include "common.h"
#include <btBulletDynamicsCommon.h>
#include <memory>
#include <vector>

class Config;
class Renderer;

class Physics
{
public:
    Physics();
    ~Physics();

    void init(Config& cfg);
    void step(float dt);
    void shutdown();

    void debugDraw(Renderer& renderer);

    btDiscreteDynamicsWorld* world() { return m_world.get(); }

private:
    void debugDrawShape(btCollisionShape* shape, const glm::mat4& model, Renderer& renderer);

    std::unique_ptr<btDefaultCollisionConfiguration> m_collisionConfig;
    std::unique_ptr<btCollisionDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::unique_ptr<btDiscreteDynamicsWorld> m_world;

    // Fixed internal substep size (see /physics/timestep) and how many of
    // them stepSimulation may run to fully cover one frame's dt — sized so
    // even App's own worst-case dt clamp (0.1s) is always fully caught up,
    // never silently truncated. See Physics::step.
    float m_fixedTimeStep = 0.004166f;
    int   m_maxSubSteps = 60;

    std::unique_ptr<btRigidBody> m_groundBody;

    // Static field boundary walls + goal structures (side/back walls of both
    // goals). Static bodies with full contact response (no longer raycast-only)
    // so the ball bounces off them and the robot can't drive through them —
    // see docs/tasks/ball-physics.md.
    std::vector<std::unique_ptr<btBoxShape>> m_wallShapes;
    std::vector<std::unique_ptr<btDefaultMotionState>> m_wallMotionStates;
    std::vector<std::unique_ptr<btRigidBody>> m_wallBodies;
};
