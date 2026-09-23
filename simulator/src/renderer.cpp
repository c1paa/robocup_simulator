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

// Uniform initializers aren't valid GLSL (only tolerated by some drivers) —
// these are always set explicitly from Renderer::setLighting before every
// draw call now, see drawMesh/drawGrid/drawLine.
uniform vec3 uLightDir;
uniform vec3 uAmbient;
uniform vec3 uDiffuseColor;

void main()
{
    vec3 normal = normalize(vNormal);
    vec3 lightDir = normalize(-uLightDir);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 result = vColor * (uAmbient + diff * uDiffuseColor);
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

void Renderer::setLighting(const glm::vec3& direction, const glm::vec3& ambient, const glm::vec3& diffuseColor)
{
    m_lightDir = (glm::length(direction) > 1e-6f) ? glm::normalize(direction) : glm::vec3(0.5f, -1.0f, 0.3f);
    m_ambient = ambient;
    m_diffuseColor = diffuseColor;
}

// Uploads the light uniforms; assumes m_shaderProgram is already bound
// (glUseProgram called by the caller right before this).
static void uploadLighting(unsigned int program, const glm::vec3& dir, const glm::vec3& ambient, const glm::vec3& diffuse)
{
    glUniform3f(glGetUniformLocation(program, "uLightDir"), dir.x, dir.y, dir.z);
    glUniform3f(glGetUniformLocation(program, "uAmbient"), ambient.x, ambient.y, ambient.z);
    glUniform3f(glGetUniformLocation(program, "uDiffuseColor"), diffuse.x, diffuse.y, diffuse.z);
}

void Renderer::drawGrid(float size, float step)
{
    glUseProgram(m_shaderProgram);

    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(m_projection));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uView"),       1, GL_FALSE, glm::value_ptr(m_view));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "uModel"),      1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
    uploadLighting(m_shaderProgram, m_lightDir, m_ambient, m_diffuseColor);

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
    uploadLighting(m_shaderProgram, m_lightDir, m_ambient, m_diffuseColor);

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
    // A filled, lit UV-sphere mesh — this used to be a 3-great-circle
    // wireframe, which is why the ball looked "transparent, made of
    // stripes" to a vision pipeline reading the mirror-camera image: three
    // thin rings have almost no filled area for contour/color-based ball
    // detection to find. A solid mesh with real normals (shaded by
    // Renderer's lighting, see setLighting) fixes both the ball's own
    // appearance and gives it something for shadows/highlights to land on.
    const int latSeg = 12, lonSeg = 24;
    std::vector<Vertex> verts;
    verts.reserve((size_t)(latSeg + 1) * (lonSeg + 1));
    glm::mat3 normalMat = glm::mat3(transform);
    for (int lat = 0; lat <= latSeg; lat++) {
        float theta = glm::pi<float>() * (float)lat / (float)latSeg; // 0 (top) .. pi (bottom)
        float sinT = std::sin(theta), cosT = std::cos(theta);
        for (int lon = 0; lon <= lonSeg; lon++) {
            float phi = 2.0f * glm::pi<float>() * (float)lon / (float)lonSeg;
            glm::vec3 n(sinT * std::cos(phi), cosT, sinT * std::sin(phi));
            glm::vec3 worldPos = glm::vec3(transform * glm::vec4(n * radius, 1.0f));
            glm::vec3 worldNormal = glm::normalize(normalMat * n);
            verts.push_back({worldPos, worldNormal, color});
        }
    }

    std::vector<unsigned int> idx;
    idx.reserve((size_t)latSeg * lonSeg * 6);
    for (int lat = 0; lat < latSeg; lat++) {
        for (int lon = 0; lon < lonSeg; lon++) {
            unsigned int a = (unsigned int)(lat * (lonSeg + 1) + lon);
            unsigned int b = a + (unsigned int)(lonSeg + 1);
            idx.push_back(a);     idx.push_back(b);     idx.push_back(a + 1);
            idx.push_back(a + 1); idx.push_back(b);     idx.push_back(b + 1);
        }
    }

    // Disable culling rather than hand-deriving the exact winding order this
    // lat/long triangulation produces — a sphere is always viewed from
    // outside in this scene, so there's no correctness cost, just a few
    // hundred extra (trivial) fragment shades.
    glDisable(GL_CULL_FACE);
    drawMesh(verts, idx, glm::mat4(1.0f));
    glEnable(GL_CULL_FACE);
}

void Renderer::drawSolidBox(const glm::vec3& he, const glm::mat4& transform, const glm::vec3& color)
{
    glm::vec3 local[8] = {
        {-he.x, -he.y, -he.z}, { he.x, -he.y, -he.z}, { he.x,  he.y, -he.z}, {-he.x,  he.y, -he.z},
        {-he.x, -he.y,  he.z}, { he.x, -he.y,  he.z}, { he.x,  he.y,  he.z}, {-he.x,  he.y,  he.z},
    };
    // Each face as 4 local-space corner indices + its outward local normal.
    struct Face { int c[4]; glm::vec3 n; };
    Face faces[6] = {
        {{4,5,6,7}, { 0, 0, 1}}, // +Z
        {{1,0,3,2}, { 0, 0,-1}}, // -Z
        {{1,5,6,2}, { 1, 0, 0}}, // +X
        {{0,4,7,3}, {-1, 0, 0}}, // -X
        {{3,2,6,7}, { 0, 1, 0}}, // +Y
        {{0,1,5,4}, { 0,-1, 0}}, // -Y
    };

    glm::mat3 normalMat = glm::mat3(transform);
    std::vector<Vertex> verts;
    std::vector<unsigned int> idx;
    verts.reserve(24);
    idx.reserve(36);
    for (auto& f : faces) {
        glm::vec3 worldNormal = glm::normalize(normalMat * f.n);
        unsigned int base = (unsigned int)verts.size();
        for (int c : f.c) {
            glm::vec3 worldPos = glm::vec3(transform * glm::vec4(local[c], 1.0f));
            verts.push_back({worldPos, worldNormal, color});
        }
        idx.push_back(base);     idx.push_back(base + 1); idx.push_back(base + 2);
        idx.push_back(base);     idx.push_back(base + 2); idx.push_back(base + 3);
    }

    glDisable(GL_CULL_FACE); // same reasoning as drawSphere
    drawMesh(verts, idx, glm::mat4(1.0f));
    glEnable(GL_CULL_FACE);
}

void Renderer::drawShadowBlob(const glm::vec3& objPos, float objRadius)
{
    // Deliberately not a real shadow map (would need a light-space depth
    // pass shared across the 6 cubemap faces the mirror-camera already
    // renders, plus every solid object contributing real depth — a much
    // bigger change). Instead: project objPos straight down the configured
    // light direction onto the ground and paint a flat disc there, darkening
    // whatever's underneath via a *multiplicative* blend (dst * src, no
    // alpha channel needed) so it correctly darkens the floor, field lines,
    // or anything else already drawn — not just a fixed "shadow color" that
    // would only look right over the green floor.
    if (m_lightDir.y >= -0.05f) return; // light ~horizontal or from below: skip, no sane projection

    float t = objPos.y / (-m_lightDir.y);
    glm::vec3 center = objPos + m_lightDir * t;

    // Clamp how far a shallow light angle can stretch the shadow away from
    // the object, so a near-horizontal light doesn't smear it across the
    // whole field.
    glm::vec2 offset(center.x - objPos.x, center.z - objPos.z);
    float maxOffset = objRadius * 5.0f;
    float offLen = glm::length(offset);
    if (offLen > maxOffset && offLen > 1e-6f) {
        offset *= maxOffset / offLen;
        center.x = objPos.x + offset.x;
        center.z = objPos.z + offset.y;
    }
    center.y = 0.0015f; // just above field lines (0.0005) to avoid z-fighting

    const int seg = 20;
    const float shadowRadius = objRadius * 1.15f;
    const glm::vec3 darken(0.35f, 0.35f, 0.35f); // multiplicative factor, not an RGB color
    std::vector<Vertex> verts;
    verts.reserve(seg + 2);
    verts.push_back({center, {0.0f, 1.0f, 0.0f}, darken});
    for (int i = 0; i <= seg; i++) {
        float a = (float)i / (float)seg * 2.0f * glm::pi<float>();
        glm::vec3 p = center + glm::vec3(std::cos(a) * shadowRadius, 0.0f, std::sin(a) * shadowRadius);
        verts.push_back({p, {0.0f, 1.0f, 0.0f}, darken});
    }
    std::vector<unsigned int> idx;
    idx.reserve(seg * 3);
    for (int i = 1; i <= seg; i++) {
        idx.push_back(0);
        idx.push_back((unsigned int)i);
        idx.push_back((unsigned int)(i == seg ? 1 : i + 1));
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_DST_COLOR, GL_ZERO);
    glDepthMask(GL_FALSE);
    drawMesh(verts, idx, glm::mat4(1.0f));
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
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
    uploadLighting(m_shaderProgram, m_lightDir, m_ambient, m_diffuseColor);

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
