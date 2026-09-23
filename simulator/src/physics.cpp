#include "physics.h"
#include "renderer.h"
#include "config.h"
#include <cmath>
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

    m_fixedTimeStep = cfg.getFloat("/physics/timestep", 0.004166f);
    if (m_fixedTimeStep <= 0.0f) m_fixedTimeStep = 0.004166f;
    // Enough substeps to fully cover App's own dt spike clamp (0.1s) with
    // margin, so a slow frame never silently truncates simulated time.
    m_maxSubSteps = (int)std::ceil(0.1f / m_fixedTimeStep) + 4;

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
    // values Field::render uses, so the drawn walls, the raycast targets and
    // the contact-response bodies all coincide. Full collision response (not
    // just raycast) — see ROADMAP item 7, closed as a side effect of the ball
    // task (docs/tasks/ball-physics.md).
    float length = cfg.getFloat("/field/length", 12600.0f) / MM;
    float width  = cfg.getFloat("/field/width",   9600.0f) / MM;
    float wallH  = cfg.getFloat("/field/wall_height",    150.0f) / MM;
    float wallT  = cfg.getFloat("/field/wall_thickness",  10.0f) / MM;

    float wallFriction    = cfg.getFloat("/physics/field/wall_friction",    0.3f);
    float wallRestitution = cfg.getFloat("/physics/field/wall_restitution", 0.4f);

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
    // so the drawn goals, the raycast targets and the contact-response bodies
    // all coincide. Same contact-response treatment as the boundary walls above.
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
        ci.m_friction = wallFriction;
        ci.m_restitution = wallRestitution;
        auto body = std::make_unique<btRigidBody>(ci);
        body->setCollisionFlags(body->getCollisionFlags() |
                                btCollisionObject::CF_STATIC_OBJECT);
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
        ci.m_friction = wallFriction;
        ci.m_restitution = wallRestitution;
        auto body = std::make_unique<btRigidBody>(ci);
        body->setCollisionFlags(body->getCollisionFlags() |
                                btCollisionObject::CF_STATIC_OBJECT);
        m_world->addRigidBody(body.get());
        m_wallShapes.push_back(std::move(shape));
        m_wallMotionStates.push_back(std::move(motion));
        m_wallBodies.push_back(std::move(body));
    }

    std::cout << "[Physics] World created. Gravity: " << m_world->getGravity().y() << std::endl;
}

void Physics::step(float dt)
{
    // Fixed-size internal substeps (previously a single variable-length step
    // per frame — maxSubSteps=0), so fast bodies (the ball) and the now-solid
    // walls/goals don't tunnel through each other or resolve a deep, one-shot
    // penetration into a violent ejection ("robot/ball flies off the field").
    // clearForces() only runs once, after all of Bullet's own internal
    // substeps for this single stepSimulation() call, not per substep — so
    // Robot::applyDriveForces's one-shot per-frame applyForce() still
    // integrates to the same total impulse (F * dt) as the old single-step
    // call, just spread across smaller, stable sub-intervals instead of one
    // big one. Verified empirically after this change: forward/backward/
    // strafe/turn magnitudes and the post-turn oscillation behavior from
    // docs/tasks/omni-wheel-dynamics.md's tests are unchanged.
    m_world->stepSimulation(dt, m_maxSubSteps, m_fixedTimeStep);
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
