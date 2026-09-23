#pragma once

#include "common.h"
#include <btBulletDynamicsCommon.h>
#include <memory>

class Config;
class Renderer;

class Ball
{
public:
    ~Ball();

    void init(Config& cfg, btDiscreteDynamicsWorld* world);
    void syncFromPhysics();
    void render(Renderer& renderer);

    glm::vec3 position() const { return m_position; }
    float radius() const { return m_radius; }
    float mass() const { return m_mass; }

    // Direct Bullet handle for the dribbler/kicker force model (same pattern
    // as Physics::world()). The dribbler applies a capture force at the ball's
    // contact point every frame; the kicker applies an impulse.
    btRigidBody* body() const { return m_body.get(); }

private:
    glm::vec3 m_position = glm::vec3(0.0f, 0.0f, 0.0f);

    float m_radius = 0.0215f;
    float m_mass = 0.046f;
    glm::vec3 m_color = glm::vec3(1.0f, 0.45f, 0.0f);

    btDiscreteDynamicsWorld* m_world = nullptr;
    std::unique_ptr<btCollisionShape> m_collisionShape;
    std::unique_ptr<btDefaultMotionState> m_motionState;
    std::unique_ptr<btRigidBody> m_body;
};
