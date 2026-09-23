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

    // Four static field boundary walls. Raycast targets only (see ROADMAP item 8
    // / the lidar task) — CF_NO_CONTACT_RESPONSE keeps them from blocking the
    // robot, which is a tracked item-7 gap.
    std::vector<std::unique_ptr<btBoxShape>> m_wallShapes;
    std::vector<std::unique_ptr<btDefaultMotionState>> m_wallMotionStates;
    std::vector<std::unique_ptr<btRigidBody>> m_wallBodies;
};
