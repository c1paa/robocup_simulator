#include "ball.h"
#include "renderer.h"
#include "config.h"

static const float MM = 1000.0f;

Ball::~Ball()
{
    if (m_world && m_body) {
        m_world->removeRigidBody(m_body.get());
    }
}

void Ball::init(Config& cfg, btDiscreteDynamicsWorld* world)
{
    m_world = world;

    // radius is a length (mm -> m); mass is configured in grams, converted to
    // kg with the same /1000 pattern but it is a different physical quantity.
    m_radius = cfg.getFloat("/physics/ball/radius", 21.5f) / MM;
    float mass = cfg.getFloat("/physics/ball/mass", 0.046f) / 1000.0f;
    float friction = cfg.getFloat("/physics/ball/friction", 0.07f);
    float restitution = cfg.getFloat("/physics/ball/restitution", 0.8f);
    float rollingFriction = cfg.getFloat("/physics/ball/rolling_friction", 0.02f);
    float linearDamping = cfg.getFloat("/physics/ball/linear_damping", 0.05f);
    float angularDamping = cfg.getFloat("/physics/ball/angular_damping", 0.1f);

    m_color.r = cfg.getFloat("/physics/ball/color/0", 1.0f);
    m_color.g = cfg.getFloat("/physics/ball/color/1", 0.45f);
    m_color.b = cfg.getFloat("/physics/ball/color/2", 0.0f);

    m_collisionShape = std::make_unique<btSphereShape>(m_radius);

    btVector3 localInertia(0.0f, 0.0f, 0.0f);
    m_collisionShape->calculateLocalInertia(mass, localInertia);

    // Spawn just forward of the robot (which also spawns at field center) so
    // the ball starts resting on the ground instead of interpenetrating the
    // robot chassis, then settle to a stop after Bullet resolves the drop.
    float robotDiameter = cfg.getFloat("/robot/diameter", 180.0f) / MM;
    float spawnX = robotDiameter * 0.5f + m_radius + 0.02f;
    float spawnY = m_radius + 0.001f;

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(spawnX, spawnY, 0.0f));
    m_motionState = std::make_unique<btDefaultMotionState>(startTransform);

    btRigidBody::btRigidBodyConstructionInfo ci(mass, m_motionState.get(),
                                                m_collisionShape.get(), localInertia);
    ci.m_friction = friction;
    ci.m_restitution = restitution;
    ci.m_rollingFriction = rollingFriction;
    ci.m_linearDamping = linearDamping;
    ci.m_angularDamping = angularDamping;
    m_body = std::make_unique<btRigidBody>(ci);
    m_world->addRigidBody(m_body.get());

    m_position = glm::vec3(spawnX, spawnY, 0.0f);
}

void Ball::syncFromPhysics()
{
    if (!m_body) return;

    btTransform trans = m_body->getWorldTransform();
    btVector3 pos = trans.getOrigin();
    m_position = glm::vec3(pos.x(), pos.y(), pos.z());
}

void Ball::render(Renderer& renderer)
{
    glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position);
    renderer.drawSphere(m_radius, model, m_color);
}
