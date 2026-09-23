#pragma once

#include "common.h"
#include <vector>
#include <string>

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
};

class Renderer
{
public:
    bool init(int width, int height);
    void shutdown();

    void beginFrame(const glm::mat4& projection, const glm::mat4& view);
    void endFrame();

    // Directional light + ambient/diffuse tint, applied to every lit draw
    // call (drawMesh and everything built on it) until changed again. Call
    // once after init() — see App::init reading /scene/light.
    void setLighting(const glm::vec3& direction, const glm::vec3& ambient, const glm::vec3& diffuseColor);

    // Basic primitives
    void drawGrid(float size, float step);
    void drawLine(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color);
    void drawBox(const glm::vec3& halfExtents, const glm::mat4& transform, const glm::vec3& color); // wireframe
    void drawSolidBox(const glm::vec3& halfExtents, const glm::mat4& transform, const glm::vec3& color); // filled, lit
    void drawSphere(float radius, const glm::mat4& transform, const glm::vec3& color); // filled, lit (see .cpp)
    void drawCylinder(float radius, float height, const glm::mat4& transform, const glm::vec3& color); // wireframe
    void drawCone(float baseRadius, float height, const glm::mat4& transform, const glm::vec3& color); // wireframe
    void drawMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const glm::mat4& model);

    // A cheap directional-light ground shadow: a flat, multiplicatively-
    // darkened disc projected from objPos along the current light direction
    // (not a real shadow map — see the .cpp for why that's a deliberate
    // scope call). Draw after the field/floor and before the casting object
    // itself so the object isn't drawn under its own shadow.
    void drawShadowBlob(const glm::vec3& objPos, float objRadius);

private:
    void createGridMesh(float size, float step);

    unsigned int m_shaderProgram = 0;
    unsigned int m_gridVAO = 0, m_gridVBO = 0;
    unsigned int m_gridVertexCount = 0;

    glm::mat4 m_projection;
    glm::mat4 m_view;

    glm::vec3 m_lightDir = glm::vec3(0.5f, -1.0f, 0.3f);
    glm::vec3 m_ambient = glm::vec3(0.25f, 0.25f, 0.28f);
    glm::vec3 m_diffuseColor = glm::vec3(0.85f, 0.83f, 0.78f);
};
