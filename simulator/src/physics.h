#pragma once

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
    std::unique_ptr<btDefaultCollisionConfiguration> m_collisionConfig;
    std::unique_ptr<btCollisionDispatcher> m_dispatcher;
    std::unique_ptr<btBroadphaseInterface> m_broadphase;
    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::unique_ptr<btDiscreteDynamicsWorld> m_world;

    std::unique_ptr<btRigidBody> m_groundBody;

    // Static field boundary walls + goal structures (side/back walls of both
    // goals). Static bodies with full contact response (no longer raycast-only)
    // so the ball bounces off them and the robot can't drive through them —
    // see docs/tasks/ball-physics.md.
    std::vector<std::unique_ptr<btBoxShape>> m_wallShapes;
    std::vector<std::unique_ptr<btDefaultMotionState>> m_wallMotionStates;
    std::vector<std::unique_ptr<btRigidBody>> m_wallBodies;
};
