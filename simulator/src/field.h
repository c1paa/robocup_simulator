#pragma once

#include "common.h"
#include <string>

class Config;
class Renderer;

class Field
{
public:
    void init(Config& cfg);
    void render(Renderer& renderer);

    float length()      const { return m_length; }
    float width()       const { return m_width; }
    float halfLength()  const { return m_length * 0.5f; }
    float halfWidth()   const { return m_width * 0.5f; }

    float playHalfLength() const { return halfLength() - m_boundaryWidth; }
    float playHalfWidth()  const { return halfWidth()  - m_boundaryWidth; }

private:
    float m_length          = 12.6f;
    float m_width           = 9.6f;
    float m_boundaryWidth   = 0.3f;
    float m_goalWidth       = 1.0f;
    float m_goalDepth       = 0.18f;
    float m_goalHeight      = 0.16f;
    float m_goalWallOffset  = 0.0f;
    float m_goalWallThickness = 0.01f;
    float m_penaltyWidth    = 3.0f;
    float m_penaltyDepth    = 2.0f;
    float m_penaltyRadius   = 0.0f;
    float m_centerCircleR   = 0.5f;
    float m_lineWidth       = 0.01f;
    float m_wallHeight      = 0.15f;
    float m_wallThickness   = 0.01f;

    glm::vec3 m_floorColor = glm::vec3(0.08f, 0.42f, 0.15f);
    glm::vec3 m_lineColor  = glm::vec3(1.0f, 1.0f, 1.0f);
    glm::vec3 m_wallColor  = glm::vec3(0.05f, 0.05f, 0.05f);
    glm::vec3 m_goalLeftColor  = glm::vec3(1.0f, 1.0f, 0.0f);
    glm::vec3 m_goalRightColor = glm::vec3(0.0f, 0.3f, 1.0f);
    glm::vec3 m_goalFrameColor = glm::vec3(0.75f, 0.75f, 0.75f);

    void drawWallQuad(Renderer& r, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d, const glm::vec3& color) const;
    void drawThickLine(Renderer& r, const glm::vec3& a, const glm::vec3& b, float ly, float width, const glm::vec3& color) const;
    void drawPenaltyArea(Renderer& r, float bx, float bz, float depth, float width, float radius, const glm::vec3& color) const;
};
