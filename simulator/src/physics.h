#pragma once

#include <btBulletDynamicsCommon.h>
#include <memory>

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
};
