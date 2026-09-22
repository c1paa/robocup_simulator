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

    // Basic primitives
    void drawGrid(float size, float step);
    void drawLine(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color);
    void drawBox(const glm::vec3& halfExtents, const glm::mat4& transform, const glm::vec3& color);
    void drawSphere(float radius, const glm::mat4& transform, const glm::vec3& color);
    void drawCylinder(float radius, float height, const glm::mat4& transform, const glm::vec3& color);
    void drawCone(float baseRadius, float height, const glm::mat4& transform, const glm::vec3& color);
    void drawMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const glm::mat4& model);

private:
    void createGridMesh(float size, float step);

    unsigned int m_shaderProgram = 0;
    unsigned int m_gridVAO = 0, m_gridVBO = 0;
    unsigned int m_gridVertexCount = 0;

    glm::mat4 m_projection;
    glm::mat4 m_view;
};
