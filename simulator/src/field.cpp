#include "field.h"
#include "renderer.h"
#include "config.h"
#include <cmath>

static const float MM = 1000.0f;

void Field::init(Config& cfg)
{
    m_length            = cfg.getFloat("/field/length",             12600.0f) / MM;
    m_width             = cfg.getFloat("/field/width",              9600.0f)  / MM;
    m_boundaryWidth     = cfg.getFloat("/field/boundary_width",      300.0f)  / MM;
    m_goalWidth         = cfg.getFloat("/field/goal_width",         1000.0f)  / MM;
    m_goalDepth         = cfg.getFloat("/field/goal_depth",          180.0f)  / MM;
    m_goalHeight        = cfg.getFloat("/field/goal_height",         160.0f)  / MM;
    m_goalWallOffset    = cfg.getFloat("/field/goal_wall_offset",      0.0f)  / MM;
    m_goalWallThickness = cfg.getFloat("/field/goal_wall_thickness",  10.0f)  / MM;
    m_penaltyWidth      = cfg.getFloat("/field/penalty_area_width", 3000.0f)  / MM;
    m_penaltyDepth      = cfg.getFloat("/field/penalty_area_depth", 2000.0f)  / MM;
    m_penaltyRadius     = cfg.getFloat("/field/penalty_area_radius",   0.0f)  / MM;
    m_centerCircleR     = cfg.getFloat("/field/center_circle_radius", 500.0f) / MM;
    m_lineWidth         = cfg.getFloat("/field/line_width",           10.0f)  / MM;
    m_wallHeight        = cfg.getFloat("/field/wall_height",         150.0f)  / MM;
    m_wallThickness     = cfg.getFloat("/field/wall_thickness",       10.0f)  / MM;

    m_floorColor.r = cfg.getFloat("/field/floor_color/0", 0.08f);
    m_floorColor.g = cfg.getFloat("/field/floor_color/1", 0.42f);
    m_floorColor.b = cfg.getFloat("/field/floor_color/2", 0.15f);
    m_lineColor.r  = cfg.getFloat("/field/line_color/0",  1.0f);
    m_lineColor.g  = cfg.getFloat("/field/line_color/1",  1.0f);
    m_lineColor.b  = cfg.getFloat("/field/line_color/2",  1.0f);
    m_wallColor.r  = cfg.getFloat("/field/wall_color/0",  0.05f);
    m_wallColor.g  = cfg.getFloat("/field/wall_color/1",  0.05f);
    m_wallColor.b  = cfg.getFloat("/field/wall_color/2",  0.05f);
    m_goalFrameColor.r = cfg.getFloat("/field/goal_color/0",  0.75f);
    m_goalFrameColor.g = cfg.getFloat("/field/goal_color/1",  0.75f);
    m_goalFrameColor.b = cfg.getFloat("/field/goal_color/2",  0.75f);
    m_goalLeftColor.r  = cfg.getFloat("/field/goal_left_color/0",  1.0f);
    m_goalLeftColor.g  = cfg.getFloat("/field/goal_left_color/1",  1.0f);
    m_goalLeftColor.b  = cfg.getFloat("/field/goal_left_color/2",  0.0f);
    m_goalRightColor.r = cfg.getFloat("/field/goal_right_color/0", 0.0f);
    m_goalRightColor.g = cfg.getFloat("/field/goal_right_color/1", 0.3f);
    m_goalRightColor.b = cfg.getFloat("/field/goal_right_color/2", 1.0f);
}

void Field::render(Renderer& renderer)
{
    float hw = halfWidth();
    float hl = halfLength();
    float phw = playHalfWidth();
    float phl = playHalfLength();
    float ly  = 0.0005f;

    // ---- Floor ----
    {
        std::vector<Vertex> v = {
            {{-hl, 0.0f, -hw}, {0,1,0}, m_floorColor},
            {{ hl, 0.0f, -hw}, {0,1,0}, m_floorColor},
            {{ hl, 0.0f,  hw}, {0,1,0}, m_floorColor},
            {{-hl, 0.0f,  hw}, {0,1,0}, m_floorColor},
        };
        std::vector<unsigned int> idx = {0,2,1, 0,3,2};
        renderer.drawMesh(v, idx, glm::mat4(1.0f));
    }
    glDisable(GL_POLYGON_OFFSET_FILL);

    // ---- Field lines (thick quads) ----
    float lw = m_lineWidth;

    // Outer boundary (4 sides)
    drawThickLine(renderer, {-phl, ly, -phw}, {phl, ly, -phw}, ly, lw, m_lineColor);
    drawThickLine(renderer, { phl, ly, -phw}, {phl, ly,  phw}, ly, lw, m_lineColor);
    drawThickLine(renderer, { phl, ly,  phw}, {-phl, ly,  phw}, ly, lw, m_lineColor);
    drawThickLine(renderer, {-phl, ly,  phw}, {-phl, ly, -phw}, ly, lw, m_lineColor);

    // Center line
    drawThickLine(renderer, {0.0f, ly, -phw}, {0.0f, ly, phw}, ly, lw, m_lineColor);

    // Center circle (approximated as thick segments)
    int seg = 80;
    for (int i = 0; i < seg; i++) {
        float a0 = (float)i / seg * 2.0f * glm::pi<float>();
        float a1 = (float)(i + 1) / seg * 2.0f * glm::pi<float>();
        float x0 = std::cos(a0) * m_centerCircleR;
        float z0 = std::sin(a0) * m_centerCircleR;
        float x1 = std::cos(a1) * m_centerCircleR;
        float z1 = std::sin(a1) * m_centerCircleR;
        drawThickLine(renderer, {x0, ly, z0}, {x1, ly, z1}, ly, lw, m_lineColor);
    }

    // Penalty areas
    drawPenaltyArea(renderer, -phl, 0.0f, m_penaltyDepth, m_penaltyWidth, m_penaltyRadius, m_lineColor);
    drawPenaltyArea(renderer,  phl, 0.0f, m_penaltyDepth, m_penaltyWidth, m_penaltyRadius, m_lineColor);

    // ---- Walls ----
    float why = m_wallHeight;

    drawWallQuad(renderer, {-hl,0,-hw},{hl,0,-hw},{hl,why,-hw},{-hl,why,-hw}, m_wallColor);
    drawWallQuad(renderer, {-hl,0, hw},{hl,0, hw},{hl,why, hw},{-hl,why, hw}, m_wallColor);
    drawWallQuad(renderer, {-hl,0,-hw},{-hl,0,hw},{-hl,why,hw},{-hl,why,-hw}, m_wallColor);
    drawWallQuad(renderer, { hl,0,-hw},{ hl,0,hw},{ hl,why,hw},{ hl,why,-hw}, m_wallColor);

    // ---- Goals ----
    float gt  = m_goalWallThickness;
    float ghw = m_goalWidth * 0.5f;
    glm::vec3 edgeColor(0.3f, 0.3f, 0.3f);

    auto drawGoal = [&](float sign, const glm::vec3& innerColor) {
        float gx  = sign * (playHalfLength() + m_goalWallOffset);  // front (boundary)
        float gbx = gx + sign * m_goalDepth;                       // back (inner)
        float obx = gbx + sign * gt;                               // back (outer)
        float gy  = m_goalHeight;

        // Side walls: extend from front (gx) to back outer (obx)
        auto drawSideWall = [&](float zInner, float zOuter) {
            // Inner face
            drawWallQuad(renderer, {obx,0,zInner},{gx,0,zInner},{gx,gy,zInner},{obx,gy,zInner}, innerColor);
            // Outer face
            drawWallQuad(renderer, {obx,0,zOuter},{gx,0,zOuter},{gx,gy,zOuter},{obx,gy,zOuter}, m_wallColor);
            // Top edge
            drawWallQuad(renderer, {obx,gy,zInner},{gx,gy,zInner},{gx,gy,zOuter},{obx,gy,zOuter}, edgeColor);
            // Front edge (goal mouth)
            drawWallQuad(renderer, {gx,0,zInner},{gx,0,zOuter},{gx,gy,zOuter},{gx,gy,zInner}, edgeColor);
            // Back edge (meets back wall)
            drawWallQuad(renderer, {obx,0,zOuter},{obx,0,zInner},{obx,gy,zInner},{obx,gy,zOuter}, edgeColor);
        };

        drawSideWall(-ghw, -(ghw + gt));  // left
        drawSideWall( ghw,   ghw + gt );  // right

        // Back wall: extends between side wall outer faces
        float bz1 = -(ghw + gt);
        float bz2 =   ghw + gt;

        // Inner face (colored)
        drawWallQuad(renderer, {gbx,0,bz1},{gbx,0,bz2},{gbx,gy,bz2},{gbx,gy,bz1}, innerColor);
        // Outer face (black)
        drawWallQuad(renderer, {obx,0,bz1},{obx,0,bz2},{obx,gy,bz2},{obx,gy,bz1}, m_wallColor);
        // Top edge
        drawWallQuad(renderer, {gbx,gy,bz1},{gbx,gy,bz2},{obx,gy,bz2},{obx,gy,bz1}, edgeColor);
    };
    drawGoal(-1.0f, m_goalLeftColor);
    drawGoal( 1.0f, m_goalRightColor);
}

// ---- Helpers ----

void Field::drawWallQuad(Renderer& r, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d, const glm::vec3& color) const
{
    glm::vec3 normal = glm::normalize(glm::cross(b - a, c - a));
    std::vector<Vertex> v = {
        {a, normal, color}, {b, normal, color}, {c, normal, color}, {d, normal, color},
    };
    std::vector<unsigned int> idx = {0,2,1, 0,3,2};
    r.drawMesh(v, idx, glm::mat4(1.0f));

    glm::vec3 rn = -normal;
    std::vector<Vertex> vr = {
        {a, rn, color}, {b, rn, color}, {c, rn, color}, {d, rn, color},
    };
    std::vector<unsigned int> idxr = {1,2,0, 2,3,0};
    r.drawMesh(vr, idxr, glm::mat4(1.0f));
}

void Field::drawThickLine(Renderer& r, const glm::vec3& a, const glm::vec3& b, float y, float width, const glm::vec3& color) const
{
    // Draw a line segment a→b as a quad on the XZ plane with given width
    float dx = b.x - a.x;
    float dz = b.z - a.z;
    float len = std::sqrt(dx * dx + dz * dz);
    if (len < 0.0001f) return;

    float nx = -dz / len;
    float nz =  dx / len;
    float hw = width * 0.5f;

    glm::vec3 p0(a.x + nx * hw, y, a.z + nz * hw);
    glm::vec3 p1(a.x - nx * hw, y, a.z - nz * hw);
    glm::vec3 p2(b.x - nx * hw, y, b.z - nz * hw);
    glm::vec3 p3(b.x + nx * hw, y, b.z + nz * hw);

    glm::vec3 normal(0.0f, 1.0f, 0.0f);
    std::vector<Vertex> v = {
        {p0, normal, color}, {p1, normal, color}, {p2, normal, color}, {p3, normal, color},
    };
    std::vector<unsigned int> idx = {0,2,1, 0,3,2};
    r.drawMesh(v, idx, glm::mat4(1.0f));
}

void Field::drawPenaltyArea(Renderer& r, float bx, float bz, float depth, float width, float radius, const glm::vec3& color) const
{
    float hz  = width * 0.5f;
    float ly  = 0.0005f;
    float lw  = m_lineWidth;

    float frontX = bx;
    float backX  = bx + ((bx < 0.0f) ? 1.0f : -1.0f) * depth;
    float leftZ  = bz - hz;
    float rightZ = bz + hz;

    float dir = (backX > frontX) ? 1.0f : -1.0f;
    float cr  = std::min(radius, std::min(depth, hz));
    float stopX = backX - dir * cr;

    if (radius <= 0.001f) {
        drawThickLine(r, {frontX, ly, leftZ},  {backX,  ly, leftZ},  ly, lw, color);
        drawThickLine(r, {backX,  ly, leftZ},  {backX,  ly, rightZ}, ly, lw, color);
        drawThickLine(r, {backX,  ly, rightZ}, {frontX, ly, rightZ}, ly, lw, color);
        drawThickLine(r, {frontX, ly, rightZ}, {frontX, ly, leftZ},  ly, lw, color);
        return;
    }

    // Helper: draw a circular arc between two angles (shortest path)
    auto drawArc = [&](float cx, float cz, float a0, float a1) {
        float sweep = a1 - a0;
        if (sweep >  glm::pi<float>()) sweep -= 2.0f * glm::pi<float>();
        if (sweep < -glm::pi<float>()) sweep += 2.0f * glm::pi<float>();
        int seg = 16;
        for (int i = 0; i < seg; i++) {
            float t0 = a0 + sweep * (float)i / seg;
            float t1 = a0 + sweep * (float)(i + 1) / seg;
            float x0 = cx + std::cos(t0) * cr, z0 = cz + std::sin(t0) * cr;
            float x1 = cx + std::cos(t1) * cr, z1 = cz + std::sin(t1) * cr;
            drawThickLine(r, {x0, ly, z0}, {x1, ly, z1}, ly, lw, color);
        }
    };

    float endA = (dir > 0.0f) ? 0.0f : glm::pi<float>();

    // Right side
    drawThickLine(r, {frontX, ly, rightZ}, {stopX, ly, rightZ}, ly, lw, color);

    // Upper-right arc
    drawArc(stopX, rightZ - cr, glm::pi<float>() * 0.5f, endA);

    // Back side
    drawThickLine(r, {backX, ly, rightZ - cr}, {backX, ly, leftZ + cr}, ly, lw, color);

    // Lower-left arc
    drawArc(stopX, leftZ + cr, -glm::pi<float>() * 0.5f, endA);

    // Left side
    drawThickLine(r, {stopX, ly, leftZ}, {frontX, ly, leftZ}, ly, lw, color);

    // Front line
    drawThickLine(r, {frontX, ly, rightZ}, {frontX, ly, leftZ}, ly, lw, color);
}
