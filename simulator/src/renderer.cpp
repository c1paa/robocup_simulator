#include "renderer.h"

#include <iostream>
#include <cmath>

static unsigned int compileShader(GLenum type, const char* source)
{
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info[512];
        glGetShaderInfoLog(shader, 512, nullptr, info);
        std::cerr << "[Shader] Compile error: " << info << std::endl;
    }
    return shader;
}

static unsigned int createShaderProgram(const char* vertSrc, const char* fragSrc)
{
    unsigned int vert = compileShader(GL_VERTEX_SHADER, vertSrc);
    unsigned int frag = compileShader(GL_FRAGMENT_SHADER, fragSrc);

    unsigned int program = glCreateProgram();
    glAttachShader(program, vert);
    glAttachShader(program, frag);
    glLinkProgram(program);

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char info[512];
        glGetProgramInfoLog(program, 512, nullptr, info);
        std::cerr << "[Shader] Link error: " << info << std::endl;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);

    return program;
}

// ============================================================
// Default shaders (embedded)
// ============================================================
static const char* DEFAULT_VERT = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uModel;

out vec3 vNormal;
out vec3 vColor;
out vec3 vWorldPos;

void main()
{
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vWorldPos = worldPos.xyz;
    vNormal = mat3(uModel) * aNormal;
    vColor = aColor;
    gl_Position = uProjection * uView * worldPos;
}
)GLSL";

static const char* DEFAULT_FRAG = R"GLSL(
#version 330 core
in vec3 vNormal;
in vec3 vColor;
in vec3 vWorldPos;

out vec4 FragColor;

uniform vec3 uLightDir = vec3(0.5, -1.0, 0.3);
uniform vec3 uAmbient = vec3(0.2, 0.2, 0.25);

void main()
{
    vec3 normal = normalize(vNormal);
    vec3 lightDir = normalize(-uLightDir);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 result = vColor * (uAmbient + diff * 0.8);
    FragColor = vec4(result, 1.0);
}
)GLSL";

// ============================================================

bool Renderer::init(int width, int height)
{
    (void)width;
    (void)height;

    m_shaderProgram = createShaderProgram(DEFAULT_VERT, DEFAULT_FRAG);
    createGridMesh(20.0f, 1.0f);

    std::cout << "[Renderer] Ready." << std::endl;
    return true;
}

void Renderer::shutdown()
{
    if (m_gridVAO) glDeleteVertexArrays(1, &m_gridVAO);
    if (m_gridVBO) glDeleteBuffers(1, &m_gridVBO);
    if (m_shaderProgram) glDeleteProgram(m_shaderProgram);
}

void Renderer::beginFrame(const glm::mat4& projection, const glm::mat4& view)
{
    m_projection = projection;
    m_view = view;
}

void Renderer::endFrame()
{
    // Nothing to flush — immediate mode drawing for now
}

void Renderer::drawGrid(float size, float step)
{
    glUseProgram(m_shaderProgram);

    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(m_projection));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uView"),       1, GL_FALSE, glm::value_ptr(m_view));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uModel"),      1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));

    glBindVertexArray(m_gridVAO);
    glDrawArrays(GL_LINES, 0, m_gridVertexCount);
    glBindVertexArray(0);
}

void Renderer::drawLine(const glm::vec3& a, const glm::vec3& b, const glm::vec3& color)
{
    float vertices[] = {
        a.x, a.y, a.z, 0.0f, 1.0f, 0.0f, color.r, color.g, color.b,
        b.x, b.y, b.z, 0.0f, 1.0f, 0.0f, color.r, color.g, color.b,
    };

    unsigned int vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glUseProgram(m_shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(m_projection));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uView"),       1, GL_FALSE, glm::value_ptr(m_view));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uModel"),      1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));

    glDrawArrays(GL_LINES, 0, 2);

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
}

void Renderer::drawBox(const glm::vec3& halfExtents, const glm::mat4& transform, const glm::vec3& color)
{
    // Wireframe box using 12 lines
    float hx = halfExtents.x, hy = halfExtents.y, hz = halfExtents.z;

    glm::vec3 corners[8] = {
        transform * glm::vec4(-hx, -hy, -hz, 1.0f),
        transform * glm::vec4( hx, -hy, -hz, 1.0f),
        transform * glm::vec4( hx,  hy, -hz, 1.0f),
        transform * glm::vec4(-hx,  hy, -hz, 1.0f),
        transform * glm::vec4(-hx, -hy,  hz, 1.0f),
        transform * glm::vec4( hx, -hy,  hz, 1.0f),
        transform * glm::vec4( hx,  hy,  hz, 1.0f),
        transform * glm::vec4(-hx,  hy,  hz, 1.0f),
    };

    int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
    };

    for (auto& e : edges) {
        drawLine(corners[e[0]], corners[e[1]], color);
    }
}

void Renderer::drawSphere(float radius, const glm::mat4& transform, const glm::vec3& color)
{
    // Wireframe sphere approximation using 3 circles
    int seg = 32;
    glm::vec3 center = transform * glm::vec4(0, 0, 0, 1);

    for (int axis = 0; axis < 3; axis++) {
        for (int i = 0; i < seg; i++) {
            float a0 = (float)i / seg * (2.0f * glm::pi<float>());
            float a1 = (float)(i + 1) / seg * (2.0f * glm::pi<float>());

            glm::vec3 p0(0), p1(0);
            if (axis == 0) { // YZ
                p0 = glm::vec3(0, std::cos(a0), std::sin(a0));
                p1 = glm::vec3(0, std::cos(a1), std::sin(a1));
            } else if (axis == 1) { // XZ
                p0 = glm::vec3(std::cos(a0), 0, std::sin(a0));
                p1 = glm::vec3(std::cos(a1), 0, std::sin(a1));
            } else { // XY
                p0 = glm::vec3(std::cos(a0), std::sin(a0), 0);
                p1 = glm::vec3(std::cos(a1), std::sin(a1), 0);
            }

            drawLine(center + p0 * radius, center + p1 * radius, color);
        }
    }
}

void Renderer::drawCylinder(float radius, float height, const glm::mat4& transform, const glm::vec3& color)
{
    int seg = 32;
    for (int i = 0; i < seg; i++) {
        float a0 = (float)i / seg * (2.0f * glm::pi<float>());
        float a1 = (float)(i + 1) / seg * (2.0f * glm::pi<float>());

        glm::vec3 top0 = transform * glm::vec4(std::cos(a0) * radius, height * 0.5f, std::sin(a0) * radius, 1.0f);
        glm::vec3 top1 = transform * glm::vec4(std::cos(a1) * radius, height * 0.5f, std::sin(a1) * radius, 1.0f);
        glm::vec3 bot0 = transform * glm::vec4(std::cos(a0) * radius, -height * 0.5f, std::sin(a0) * radius, 1.0f);
        glm::vec3 bot1 = transform * glm::vec4(std::cos(a1) * radius, -height * 0.5f, std::sin(a1) * radius, 1.0f);

        drawLine(top0, top1, color);
        drawLine(bot0, bot1, color);
        drawLine(top0, bot0, color);
    }
}

void Renderer::drawCone(float baseRadius, float height, const glm::mat4& transform, const glm::vec3& color)
{
    // Cone: base at y=+height/2, apex at y=-height/2 (pointing down)
    int seg = 32;
    glm::vec3 apexLocal = glm::vec3(0.0f, -height * 0.5f, 0.0f);

    for (int i = 0; i < seg; i++) {
        float a0 = (float)i / seg * (2.0f * glm::pi<float>());
        float a1 = (float)(i + 1) / seg * (2.0f * glm::pi<float>());

        glm::vec3 b0Local(std::cos(a0) * baseRadius, height * 0.5f, std::sin(a0) * baseRadius);
        glm::vec3 b1Local(std::cos(a1) * baseRadius, height * 0.5f, std::sin(a1) * baseRadius);

        glm::vec3 apex = transform * glm::vec4(apexLocal, 1.0f);
        glm::vec3 b0   = transform * glm::vec4(b0Local, 1.0f);
        glm::vec3 b1   = transform * glm::vec4(b1Local, 1.0f);

        // Base circle segment
        drawLine(b0, b1, color);
        // Lines from base to apex
        drawLine(b0, apex, color);
    }
}

void Renderer::drawMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const glm::mat4& model)
{
    if (vertices.empty()) return;

    unsigned int vao, vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);

    glUseProgram(m_shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(m_projection));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uView"),       1, GL_FALSE, glm::value_ptr(m_view));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uModel"),      1, GL_FALSE, glm::value_ptr(model));

    glDrawElements(GL_TRIANGLES, (int)indices.size(), GL_UNSIGNED_INT, nullptr);

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
}

void Renderer::createGridMesh(float size, float step)
{
    std::vector<float> vertices;
    float half = size * 0.5f;

    for (float x = -half; x <= half; x += step) {
        // Line along Z
        vertices.insert(vertices.end(), {x, 0.0f, -half, 0,1,0, 0.3f,0.3f,0.3f});
        vertices.insert(vertices.end(), {x, 0.0f,  half, 0,1,0, 0.3f,0.3f,0.3f});
    }
    for (float z = -half; z <= half; z += step) {
        // Line along X
        vertices.insert(vertices.end(), {-half, 0.0f, z, 0,1,0, 0.3f,0.3f,0.3f});
        vertices.insert(vertices.end(), { half, 0.0f, z, 0,1,0, 0.3f,0.3f,0.3f});
    }

    m_gridVertexCount = (int)(vertices.size() / 9);

    glGenVertexArrays(1, &m_gridVAO);
    glGenBuffers(1, &m_gridVBO);

    glBindVertexArray(m_gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}
