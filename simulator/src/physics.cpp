#include "physics.h"
#include "renderer.h"
#include "config.h"
#include <iostream>

static const float MM = 1000.0f;

Physics::Physics() = default;
Physics::~Physics() { shutdown(); }

void Physics::init(Config& cfg)
{
    m_collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher      = std::make_unique<btCollisionDispatcher>(m_collisionConfig.get());
    m_broadphase      = std::make_unique<btDbvtBroadphase>();
    m_solver          = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world           = std::make_unique<btDiscreteDynamicsWorld>(
        m_dispatcher.get(), m_broadphase.get(), m_solver.get(), m_collisionConfig.get()
    );

    float grav = cfg.getFloat("/physics/gravity", 9810.0f) / MM;
    m_world->setGravity(btVector3(0, -grav, 0));

    // Ground plane
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    btDefaultMotionState* groundMotion = new btDefaultMotionState(
        btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 0))
    );
    btRigidBody::btRigidBodyConstructionInfo groundCI(0, groundMotion, groundShape);
    groundCI.m_restitution = cfg.getFloat("/physics/field/restitution", 0.5f);
    groundCI.m_friction = cfg.getFloat("/physics/field/friction", 0.8f);
    m_groundBody = std::make_unique<btRigidBody>(groundCI);
    m_world->addRigidBody(m_groundBody.get());

    std::cout << "[Physics] World created. Gravity: " << m_world->getGravity().y() << std::endl;
}

void Physics::step(float dt)
{
    m_world->stepSimulation(dt, 3, 1.0f / 240.0f);
}

void Physics::shutdown()
{
    if (m_groundBody) {
        m_world->removeRigidBody(m_groundBody.get());
        m_groundBody.reset();
    }
    m_world.reset();
    m_solver.reset();
    m_broadphase.reset();
    m_dispatcher.reset();
    m_collisionConfig.reset();
}

void Physics::debugDraw(Renderer& renderer)
{
    if (!m_world) return;

    // Draw all collision objects as wireframes
    for (int i = 0; i < m_world->getNumCollisionObjects(); i++) {
        btCollisionObject* obj = m_world->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (!body) continue;

        btTransform trans;
        if (body->getMotionState()) {
            body->getMotionState()->getWorldTransform(trans);
        } else {
            trans = body->getWorldTransform();
        }

        glm::mat4 model(1.0f);
        trans.getOpenGLMatrix(glm::value_ptr(model));

        btCollisionShape* shape = body->getCollisionShape();
        btVector3 halfExt;
        glm::vec3 color(0.2f, 0.8f, 0.2f);

        switch (shape->getShapeType()) {
        case BOX_SHAPE_PROXYTYPE:
            halfExt = ((btBoxShape*)shape)->getHalfExtentsWithMargin();
            renderer.drawBox(glm::vec3(halfExt.x(), halfExt.y(), halfExt.z()), model, color);
            break;
        case SPHERE_SHAPE_PROXYTYPE: {
            float r = ((btSphereShape*)shape)->getRadius();
            renderer.drawSphere(r, model, color);
            break;
        }
        case CYLINDER_SHAPE_PROXYTYPE: {
            btVector3 ext = ((btCylinderShape*)shape)->getHalfExtentsWithMargin();
            float r = std::max(ext.x(), ext.z());
            float h = ext.y() * 2.0f;
            renderer.drawCylinder(r, h, model, color);
            break;
        }
        default:
            break;
        }
    }
}
