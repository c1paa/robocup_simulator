#include "debug_overlay.h"
#include "common.h"

#include <iostream>
#include <cstring>
#include <algorithm>
#include <utility>

#include "stb_easy_font.h"

// ------------------------------------------------------------
// Shader helpers (mirror the pattern in camera.cpp)
// ------------------------------------------------------------
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
        std::cerr << "[DebugOverlay Shader] Compile error: " << info << std::endl;
    }
    return shader;
}

static unsigned int createProgram(const char* vertSrc, const char* fragSrc)
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
        std::cerr << "[DebugOverlay Shader] Link error: " << info << std::endl;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);
    return program;
}

// Text: vertex data comes straight from stb_easy_font_print in pixel space
// (x right, y down); the vertex shader maps it to NDC using the window
// resolution, same transform Camera::drawPreview does on the CPU for its
// quad corners.
static const char* TEXT_VERT = R"GLSL(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform vec2 uResolution;
out vec4 vColor;
void main()
{
    vec2 ndc = vec2(aPos.x / uResolution.x * 2.0 - 1.0,
                     1.0 - aPos.y / uResolution.y * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
    vColor = aColor;
}
)GLSL";

static const char* TEXT_FRAG = R"GLSL(
#version 330 core
in vec4 vColor;
out vec4 FragColor;
void main() { FragColor = vColor; }
)GLSL";

// Background box: same NDC quad pattern as camera.cpp's preview quad, just a
// flat translucent color instead of a texture sample.
static const char* BOX_VERT = R"GLSL(
#version 330 core
layout(location = 0) in vec2 aPos;
void main() { gl_Position = vec4(aPos, 0.0, 1.0); }
)GLSL";

static const char* BOX_FRAG = R"GLSL(
#version 330 core
out vec4 FragColor;
uniform vec4 uColor;
void main() { FragColor = uColor; }
)GLSL";

// stb_easy_font_print's docs say to budget ~270 bytes/char; this caps the HUD
// at a generous ~4000 characters per frame (plenty for a telemetry readout).
static const int kMaxQuads = 4096;
static const int kVertsPerQuad = 4;
static const int kVertexStride = 16; // 3 floats (pos) + 4 bytes (color), per stb_easy_font.h

DebugOverlay::~DebugOverlay()
{
    shutdown();
}

void DebugOverlay::init()
{
    m_textProgram = createProgram(TEXT_VERT, TEXT_FRAG);
    m_boxProgram  = createProgram(BOX_VERT, BOX_FRAG);
    m_maxQuads = kMaxQuads;

    // ---- Text VAO/VBO/EBO ----
    glGenVertexArrays(1, &m_textVao);
    glGenBuffers(1, &m_textVbo);
    glGenBuffers(1, &m_textEbo);

    glBindVertexArray(m_textVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_textVbo);
    glBufferData(GL_ARRAY_BUFFER, (size_t)m_maxQuads * kVertsPerQuad * kVertexStride,
                 nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, kVertexStride, (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, kVertexStride, (void*)12);
    glEnableVertexAttribArray(1);

    // Index pattern per quad (0,1,2, 0,2,3) is fixed and independent of
    // content, so it's generated once here rather than every frame.
    std::vector<unsigned int> indices;
    indices.reserve((size_t)m_maxQuads * 6);
    for (int q = 0; q < m_maxQuads; q++) {
        unsigned int base = (unsigned int)q * kVertsPerQuad;
        indices.push_back(base + 0);
        indices.push_back(base + 1);
        indices.push_back(base + 2);
        indices.push_back(base + 0);
        indices.push_back(base + 2);
        indices.push_back(base + 3);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_textEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 indices.data(), GL_STATIC_DRAW);
    glBindVertexArray(0);

    // ---- Background box VAO/VBO (dynamic corners, filled in draw()) ----
    glGenVertexArrays(1, &m_boxVao);
    glGenBuffers(1, &m_boxVbo);
    glBindVertexArray(m_boxVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_boxVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 8, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void DebugOverlay::shutdown()
{
    if (m_textVbo) glDeleteBuffers(1, &m_textVbo);
    if (m_textEbo) glDeleteBuffers(1, &m_textEbo);
    if (m_textVao) glDeleteVertexArrays(1, &m_textVao);
    if (m_boxVbo)  glDeleteBuffers(1, &m_boxVbo);
    if (m_boxVao)  glDeleteVertexArrays(1, &m_boxVao);
    if (m_textProgram) glDeleteProgram(m_textProgram);
    if (m_boxProgram)  glDeleteProgram(m_boxProgram);
    m_textVbo = m_textEbo = m_textVao = m_boxVbo = m_boxVao = 0;
    m_textProgram = m_boxProgram = 0;
}

void DebugOverlay::draw(int windowWidth, int windowHeight, const std::vector<std::string>& lines)
{
    if (windowWidth <= 0 || windowHeight <= 0 || lines.empty()) return;
    if (!m_textProgram) return; // init() not called / failed

    const float margin = 10.0f;
    const float lineHeight = 14.0f; // stb_easy_font glyphs are ~7px tall at scale 1; pad for legibility
    const float padding = 8.0f;

    // Widest line decides the box width (stb_easy_font_width measures at its
    // default scale, matching what stb_easy_font_print will emit below).
    float maxWidth = 0.0f;
    for (const std::string& line : lines) {
        float w = stb_easy_font_width(const_cast<char*>(line.c_str()));
        maxWidth = std::max(maxWidth, w);
    }

    float boxW = maxWidth + padding * 2.0f;
    float boxH = lineHeight * (float)lines.size() + padding * 2.0f;
    float x0 = (float)windowWidth - margin - boxW; // top-right corner
    float y0 = margin;

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // ---- Background box ----
    auto toNdc = [&](float x, float y) -> std::pair<float, float> {
        float nx = x / (float)windowWidth * 2.0f - 1.0f;
        float ny = 1.0f - y / (float)windowHeight * 2.0f;
        return { nx, ny };
    };
    auto tl = toNdc(x0, y0);
    auto tr = toNdc(x0 + boxW, y0);
    auto br = toNdc(x0 + boxW, y0 + boxH);
    auto bl = toNdc(x0, y0 + boxH);
    float boxVerts[] = {
        tl.first, tl.second, tr.first, tr.second, br.first, br.second, bl.first, bl.second,
    };

    glUseProgram(m_boxProgram);
    glUniform4f(glGetUniformLocation(m_boxProgram, "uColor"), 0.0f, 0.0f, 0.0f, 0.55f);
    glBindVertexArray(m_boxVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_boxVbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(boxVerts), boxVerts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glBindVertexArray(0);

    // ---- Text ----
    std::vector<unsigned char> buf((size_t)m_maxQuads * kVertsPerQuad * kVertexStride);
    unsigned char color[4] = { 255, 255, 255, 255 };
    int totalQuads = 0;
    float textY = y0 + padding;
    for (const std::string& line : lines) {
        int remainingBytes = (int)buf.size() - totalQuads * kVertsPerQuad * kVertexStride;
        if (remainingBytes <= 0) break;
        int quads = stb_easy_font_print(
            x0 + padding, textY, const_cast<char*>(line.c_str()), color,
            buf.data() + (size_t)totalQuads * kVertsPerQuad * kVertexStride, remainingBytes);
        totalQuads += quads;
        textY += lineHeight;
    }
    if (totalQuads > m_maxQuads) totalQuads = m_maxQuads;

    if (totalQuads > 0) {
        glUseProgram(m_textProgram);
        glUniform2f(glGetUniformLocation(m_textProgram, "uResolution"),
                    (float)windowWidth, (float)windowHeight);
        glBindVertexArray(m_textVao);
        glBindBuffer(GL_ARRAY_BUFFER, m_textVbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, (size_t)totalQuads * kVertsPerQuad * kVertexStride,
                         buf.data());
        glDrawElements(GL_TRIANGLES, totalQuads * 6, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
}
