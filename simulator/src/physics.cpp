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

    // Field boundary walls: four static boxes sized from the same /field/*
    // values Field::render uses, so the drawn walls and the raycast targets
    // coincide. These are raycast-only (no contact response with the robot);
    // full wall collision is ROADMAP item 7.
    float length = cfg.getFloat("/field/length", 12600.0f) / MM;
    float width  = cfg.getFloat("/field/width",   9600.0f) / MM;
    float wallH  = cfg.getFloat("/field/wall_height",    150.0f) / MM;
    float wallT  = cfg.getFloat("/field/wall_thickness",  10.0f) / MM;

    float hl     = length * 0.5f;
    float hw     = width * 0.5f;
    float halfH  = wallH * 0.5f;
    float halfT  = wallT * 0.5f;

    struct Wall {
        btVector3 center;
        btVector3 halfExtents;
    };
    Wall walls[4] = {
        { btVector3( 0.0f, halfH, -hw), btVector3(hl,     halfH, halfT) }, // back  (z = -hw)
        { btVector3( 0.0f, halfH,  hw), btVector3(hl,     halfH, halfT) }, // front (z = +hw)
        { btVector3(-hl,   halfH, 0.0f), btVector3(halfT, halfH, hw)     }, // left  (x = -hl)
        { btVector3( hl,   halfH, 0.0f), btVector3(halfT, halfH, hw)     }, // right (x = +hl)
    };

    // Goal structures: two side walls + a back wall per goal, sized/positioned
    // from the same /field/goal_* values Field::render's drawGoal lambda uses,
    // so the drawn goals and the raycast targets coincide. Same raycast-only
    // treatment as the boundary walls above (CF_NO_CONTACT_RESPONSE) — full
    // collision response for the robot/ball is still ROADMAP item 7.
    float boundaryWidth = cfg.getFloat("/field/boundary_width",      300.0f) / MM;
    float goalWidth      = cfg.getFloat("/field/goal_width",         1000.0f) / MM;
    float goalDepth       = cfg.getFloat("/field/goal_depth",          180.0f) / MM;
    float goalHeight      = cfg.getFloat("/field/goal_height",         160.0f) / MM;
    float goalWallOffset  = cfg.getFloat("/field/goal_wall_offset",      0.0f) / MM;
    float goalWallThick   = cfg.getFloat("/field/goal_wall_thickness",  10.0f) / MM;

    float playHalfLength = hl - boundaryWidth;
    float ghw            = goalWidth * 0.5f;
    float goalHalfH       = goalHeight * 0.5f;
    float goalHalfT       = goalWallThick * 0.5f;

    std::vector<Wall> goalWalls;
    for (float sign : {-1.0f, 1.0f}) {
        float gx  = sign * (playHalfLength + goalWallOffset); // goal mouth (front)
        float gbx = gx + sign * goalDepth;                    // back wall inner face
        float obx = gbx + sign * goalWallThick;               // back wall outer face

        // Side walls (left z<0, right z>0), spanning front (gx) to back outer (obx).
        float sideHalfX = std::fabs(obx - gx) * 0.5f;
        float sideCenterX = (gx + obx) * 0.5f;
        for (float zSign : {-1.0f, 1.0f}) {
            float zCenter = zSign * (ghw + goalHalfT);
            goalWalls.push_back({ btVector3(sideCenterX, goalHalfH, zCenter),
                                   btVector3(sideHalfX, goalHalfH, goalHalfT) });
        }

        // Back wall, spanning the outer faces of both side walls.
        float backHalfX = goalWallThick * 0.5f;
        float backCenterX = gbx + sign * backHalfX;
        goalWalls.push_back({ btVector3(backCenterX, goalHalfH, 0.0f),
                               btVector3(backHalfX, goalHalfH, ghw + goalWallThick) });
    }

    for (const Wall& w : walls) {
        auto shape = std::make_unique<btBoxShape>(w.halfExtents);
        auto motion = std::make_unique<btDefaultMotionState>(
            btTransform(btQuaternion::getIdentity(), w.center));
        btRigidBody::btRigidBodyConstructionInfo ci(0.0f, motion.get(), shape.get());
        auto body = std::make_unique<btRigidBody>(ci);
        body->setCollisionFlags(body->getCollisionFlags() |
                                btCollisionObject::CF_STATIC_OBJECT |
                                btCollisionObject::CF_NO_CONTACT_RESPONSE);
        m_world->addRigidBody(body.get());
        m_wallShapes.push_back(std::move(shape));
        m_wallMotionStates.push_back(std::move(motion));
        m_wallBodies.push_back(std::move(body));
    }

    for (const Wall& w : goalWalls) {
        auto shape = std::make_unique<btBoxShape>(w.halfExtents);
        auto motion = std::make_unique<btDefaultMotionState>(
            btTransform(btQuaternion::getIdentity(), w.center));
        btRigidBody::btRigidBodyConstructionInfo ci(0.0f, motion.get(), shape.get());
        auto body = std::make_unique<btRigidBody>(ci);
        body->setCollisionFlags(body->getCollisionFlags() |
                                btCollisionObject::CF_STATIC_OBJECT |
                                btCollisionObject::CF_NO_CONTACT_RESPONSE);
        m_world->addRigidBody(body.get());
        m_wallShapes.push_back(std::move(shape));
        m_wallMotionStates.push_back(std::move(motion));
        m_wallBodies.push_back(std::move(body));
    }

    std::cout << "[Physics] World created. Gravity: " << m_world->getGravity().y() << std::endl;
}

void Physics::step(float dt)
{
    // Single variable-timestep step per frame, so the per-frame wheel friction
    // forces Robot::applyDriveForces applies are integrated over exactly the
    // same dt they were computed for. (Bullet clears accumulated forces after
    // each internal substep; stepping once per frame keeps the hand-rolled
    // Coulomb model and the integrator in agreement.)
    m_world->stepSimulation(dt, 0, 0.0f);
}

void Physics::shutdown()
{
    if (m_groundBody) {
        m_world->removeRigidBody(m_groundBody.get());
        m_groundBody.reset();
    }
    for (auto& body : m_wallBodies) {
        m_world->removeRigidBody(body.get());
    }
    m_wallBodies.clear();
    m_wallMotionStates.clear();
    m_wallShapes.clear();
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
