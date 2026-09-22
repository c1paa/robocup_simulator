#include "camera.h"
#include "config.h"
#include "renderer.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <utility>

// ------------------------------------------------------------
// Shader helpers (mirror the inline pattern in renderer.cpp)
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
        std::cerr << "[Camera Shader] Compile error: " << info << std::endl;
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
        std::cerr << "[Camera Shader] Link error: " << info << std::endl;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);
    return program;
}

static const char* QUAD_VERT = R"GLSL(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
out vec2 vUV;
void main()
{
    vUV = aUV;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)GLSL";

static const char* COMPOSITE_FRAG = R"GLSL(
#version 330 core
in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uLut;
uniform samplerCube uCubemap;
uniform float uYaw;
uniform vec3 uBackground;

void main()
{
    vec4 lut = texture(uLut, vUV);
    if (lut.a < 0.5) {
        FragColor = vec4(uBackground, 1.0);
        return;
    }

    vec3 d = lut.rgb;
    float c = cos(uYaw);
    float s = sin(uYaw);
    vec3 wd;
    wd.x = c * d.x + s * d.z;
    wd.y = d.y;
    wd.z = -s * d.x + c * d.z;

    FragColor = vec4(texture(uCubemap, wd).rgb, 1.0);
}
)GLSL";

static const char* PREVIEW_FRAG = R"GLSL(
#version 330 core
in vec2 vUV;
out vec4 FragColor;

uniform sampler2D uTex;

void main()
{
    FragColor = texture(uTex, vUV);
}
)GLSL";

// ------------------------------------------------------------

void Camera::init(Config& cfg, const MirrorProfile& mirror, float cameraHeight, Renderer* renderer)
{
    m_width  = cfg.getInt("/camera/width",  640);
    m_height = cfg.getInt("/camera/height", 480);
    m_fov    = cfg.getFloat("/camera/fov",  120.0f);
    m_noiseStd   = cfg.getFloat("/camera/noise_std",   0.02f);
    m_pixelNoise = cfg.getFloat("/camera/pixel_noise", 0.01f);
    m_cubemapRes = cfg.getInt("/camera/cubemap_resolution", 256);
    m_streamFps  = cfg.getFloat("/camera/stream_fps", 30.0f);
    m_background.r = cfg.getFloat("/camera/background_color/0", 0.0f);
    m_background.g = cfg.getFloat("/camera/background_color/1", 0.0f);
    m_background.b = cfg.getFloat("/camera/background_color/2", 0.0f);

    m_mirror       = mirror;
    m_cameraHeight = cameraHeight;
    m_renderer     = renderer;

    m_imageData.resize((size_t)m_width * m_height * 3);

    createFramebuffers();
    bakeLut();
    initComposite();
    initPreview();

    // Force an immediate render on the first frame.
    m_lastRender = std::chrono::steady_clock::now() - std::chrono::seconds(10);

    std::cout << "[Camera] " << m_width << "x" << m_height << " FOV=" << m_fov
              << " cubemap=" << m_cubemapRes << " fps=" << m_streamFps << std::endl;
}

void Camera::shutdown()
{
    if (m_lutTexture)       glDeleteTextures(1, &m_lutTexture);
    if (m_cubemapTexture)   glDeleteTextures(1, &m_cubemapTexture);
    if (m_renderTexture)    glDeleteTextures(1, &m_renderTexture);
    if (m_cubemapFbo)       glDeleteFramebuffers(1, &m_cubemapFbo);
    if (m_fbo)              glDeleteFramebuffers(1, &m_fbo);
    if (m_depthRbo)         glDeleteRenderbuffers(1, &m_depthRbo);
    if (m_cubemapDepthRbo)  glDeleteRenderbuffers(1, &m_cubemapDepthRbo);
    if (m_compositeProgram) glDeleteProgram(m_compositeProgram);
    if (m_previewProgram)   glDeleteProgram(m_previewProgram);
    if (m_quadVao)          glDeleteVertexArrays(1, &m_quadVao);
    if (m_quadVbo)          glDeleteBuffers(1, &m_quadVbo);
    if (m_quadEbo)          glDeleteBuffers(1, &m_quadEbo);
    if (m_previewVao)       glDeleteVertexArrays(1, &m_previewVao);
    if (m_previewVbo)       glDeleteBuffers(1, &m_previewVbo);
    if (m_previewEbo)       glDeleteBuffers(1, &m_previewEbo);
}

void Camera::createFramebuffers()
{
    // ---- Final composited image FBO (width x height) ----
    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    glGenTextures(1, &m_renderTexture);
    glBindTexture(GL_TEXTURE_2D, m_renderTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_renderTexture, 0);

    glGenRenderbuffers(1, &m_depthRbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_width, m_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthRbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[Camera] Final framebuffer not complete!" << std::endl;
    }

    // ---- Cubemap FBO + texture ----
    glGenTextures(1, &m_cubemapTexture);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_cubemapTexture);
    for (int face = 0; face < 6; face++) {
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_RGB,
                     m_cubemapRes, m_cubemapRes, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    glGenFramebuffers(1, &m_cubemapFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_cubemapFbo);

    glGenRenderbuffers(1, &m_cubemapDepthRbo);
    glBindRenderbuffer(GL_RENDERBUFFER, m_cubemapDepthRbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_cubemapRes, m_cubemapRes);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_cubemapDepthRbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[Camera] Cubemap framebuffer not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Camera::bakeLut()
{
    m_lut.assign((size_t)m_width * m_height * 4, 0.0f);

    float fovRad  = glm::radians(std::clamp(m_fov, 10.0f, 179.0f));
    float tanHalf = std::tan(fovRad * 0.5f);
    // Scale the vertical ray component by the aspect ratio so pixels are
    // square (same angular size horizontally and vertically). This keeps the
    // mirror silhouette circular and the horizontal FOV equal to /camera/fov.
    float aspect = (float)m_height / (float)m_width;

    glm::vec3 origin(0.0f, m_cameraHeight, 0.0f);
    float baseY = m_mirror.baseHeight();

    for (int v = 0; v < m_height; v++) {
        float ny = 2.0f * (v + 0.5f) / (float)m_height - 1.0f;
        for (int u = 0; u < m_width; u++) {
            float nx = 2.0f * (u + 0.5f) / (float)m_width - 1.0f;
            glm::vec3 dir = glm::normalize(glm::vec3(nx * tanHalf, 1.0f, ny * tanHalf * aspect));

            size_t idx = ((size_t)v * m_width + u) * 4;
            float t = 0.0f;
            if (m_mirror.intersectRay(origin, dir, t)) {
                glm::vec3 hit = origin + dir * t;
                float h = baseY - hit.y; // height from mirror base
                float radial = std::sqrt(hit.x * hit.x + hit.z * hit.z);

                glm::vec3 n0 = m_mirror.normalAt(h, radial); // normal in +X plane
                glm::vec3 uhat(1.0f, 0.0f, 0.0f);
                if (radial > 1e-6f) uhat = glm::vec3(hit.x / radial, 0.0f, hit.z / radial);
                glm::vec3 n(uhat.x * n0.x, n0.y, uhat.z * n0.x);

                glm::vec3 refl = glm::normalize(dir - 2.0f * glm::dot(dir, n) * n);

                m_lut[idx + 0] = refl.x;
                m_lut[idx + 1] = refl.y;
                m_lut[idx + 2] = refl.z;
                m_lut[idx + 3] = 1.0f;
            } else {
                m_lut[idx + 3] = 0.0f;
            }
        }
    }

    glGenTextures(1, &m_lutTexture);
    glBindTexture(GL_TEXTURE_2D, m_lutTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_width, m_height, 0, GL_RGBA, GL_FLOAT, m_lut.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Camera::initComposite()
{
    m_compositeProgram = createProgram(QUAD_VERT, COMPOSITE_FRAG);

    // Fullscreen quad; UV t=0 at the bottom (matches GL texture convention).
    float verts[] = {
        // pos           uv
        -1.0f, -1.0f,   0.0f, 0.0f,
         1.0f, -1.0f,   1.0f, 0.0f,
         1.0f,  1.0f,   1.0f, 1.0f,
        -1.0f,  1.0f,   0.0f, 1.0f,
    };
    unsigned int idx[] = { 0, 1, 2, 0, 2, 3 };

    glGenVertexArrays(1, &m_quadVao);
    glGenBuffers(1, &m_quadVbo);
    glGenBuffers(1, &m_quadEbo);

    glBindVertexArray(m_quadVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_quadVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_quadEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    glBindVertexArray(0);
}

void Camera::initPreview()
{
    m_previewProgram = createProgram(QUAD_VERT, PREVIEW_FRAG);

    // Quad geometry filled per-frame in drawPreview(); UV mapping uses t=1 at
    // the top (image top), which is why we rebuild vertices each call.
    float verts[] = {
        -1.0f, -1.0f,   0.0f, 0.0f,
         1.0f, -1.0f,   1.0f, 0.0f,
         1.0f,  1.0f,   1.0f, 1.0f,
        -1.0f,  1.0f,   0.0f, 1.0f,
    };
    unsigned int idx[] = { 0, 1, 2, 0, 2, 3 };

    glGenVertexArrays(1, &m_previewVao);
    glGenBuffers(1, &m_previewVbo);
    glGenBuffers(1, &m_previewEbo);

    glBindVertexArray(m_previewVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_previewVbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_previewEbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

    glBindVertexArray(0);
}

void Camera::renderView(const glm::vec3& robotPos, float robotYaw,
                        const std::function<void(Renderer&)>& drawScene)
{
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - m_lastRender).count();
    if (elapsed < 1.0f / m_streamFps) {
        m_frameChanged = false; // reuse last image
        return;
    }
    m_lastRender = now;
    m_frameChanged = true;

    // Effective viewpoint in world space (local viewpoint rotated by yaw).
    glm::vec3 vpLocal = m_mirror.effectiveViewpointLocal(m_cameraHeight);
    float c = std::cos(robotYaw), s = std::sin(robotYaw);
    glm::vec3 vpWorld(
        robotPos.x + c * vpLocal.x + s * vpLocal.z,
        robotPos.y + vpLocal.y,
        robotPos.z - s * vpLocal.x + c * vpLocal.z
    );

    captureCubemap(vpWorld, drawScene);
    composite(robotYaw);
    readback();
    applyNoise();
}

void Camera::captureCubemap(const glm::vec3& vp,
                            const std::function<void(Renderer&)>& drawScene)
{
    if (!m_renderer) return;

    glm::mat4 proj = glm::perspective(glm::radians(90.0f), 1.0f, 0.05f, 100.0f);
    glm::mat4 views[6] = {
        glm::lookAt(vp, vp + glm::vec3( 1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
        glm::lookAt(vp, vp + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
        glm::lookAt(vp, vp + glm::vec3( 0.0f, 1.0f, 0.0f), glm::vec3(0.0f,  0.0f, 1.0f)),
        glm::lookAt(vp, vp + glm::vec3( 0.0f,-1.0f, 0.0f), glm::vec3(0.0f,  0.0f,-1.0f)),
        glm::lookAt(vp, vp + glm::vec3( 0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
        glm::lookAt(vp, vp + glm::vec3( 0.0f, 0.0f,-1.0f), glm::vec3(0.0f, -1.0f, 0.0f)),
    };

    glViewport(0, 0, m_cubemapRes, m_cubemapRes);
    glBindFramebuffer(GL_FRAMEBUFFER, m_cubemapFbo);
    for (int face = 0; face < 6; face++) {
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, m_cubemapTexture, 0);
        glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        m_renderer->beginFrame(proj, views[face]);
        drawScene(*m_renderer);
        m_renderer->endFrame();
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Camera::composite(float robotYaw)
{
    glViewport(0, 0, m_width, m_height);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glClearColor(m_background.r, m_background.g, m_background.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    glUseProgram(m_compositeProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_lutTexture);
    glUniform1i(glGetUniformLocation(m_compositeProgram, "uLut"), 0);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_CUBE_MAP, m_cubemapTexture);
    glUniform1i(glGetUniformLocation(m_compositeProgram, "uCubemap"), 1);
    glUniform1f(glGetUniformLocation(m_compositeProgram, "uYaw"), robotYaw);
    glUniform3f(glGetUniformLocation(m_compositeProgram, "uBackground"),
                m_background.r, m_background.g, m_background.b);

    glBindVertexArray(m_quadVao);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Camera::readback()
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, m_width, m_height, GL_RGB, GL_UNSIGNED_BYTE, m_imageData.data());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // GL rows are bottom-up; flip to top-down.
    size_t rowBytes = (size_t)m_width * 3;
    std::vector<uint8_t> row(rowBytes);
    for (int y = 0; y < m_height / 2; y++) {
        uint8_t* top = m_imageData.data() + (size_t)y * rowBytes;
        uint8_t* bot = m_imageData.data() + (size_t)(m_height - 1 - y) * rowBytes;
        std::memcpy(row.data(), top, rowBytes);
        std::memcpy(top, bot, rowBytes);
        std::memcpy(bot, row.data(), rowBytes);
    }
}

void Camera::applyNoise()
{
    std::normal_distribution<float> dist(0.0f, 1.0f);
    for (size_t i = 0; i < m_imageData.size(); i++) {
        float n = dist(m_rng) * m_noiseStd + dist(m_rng) * m_pixelNoise;
        float v = (float)m_imageData[i] / 255.0f + n;
        m_imageData[i] = (uint8_t)std::clamp((int)std::lround(v * 255.0f), 0, 255);
    }
}

void Camera::drawPreview(int windowWidth, int windowHeight)
{
    if (windowWidth <= 0 || windowHeight <= 0) return;

    float pw = 240.0f;
    float ph = pw * (float)m_height / (float)m_width;

    float x0 = 10.0f, y0 = 10.0f; // top-left margin (window coords, y down)
    float x1 = x0 + pw, y1 = y0 + ph;

    auto toNdc = [&](float x, float y) -> std::pair<float, float> {
        float nx = x / (float)windowWidth * 2.0f - 1.0f;
        float ny = 1.0f - y / (float)windowHeight * 2.0f;
        return { nx, ny };
    };

    auto tl = toNdc(x0, y0);
    auto tr = toNdc(x1, y0);
    auto br = toNdc(x1, y1);
    auto bl = toNdc(x0, y1);

    // Image top (texture t=1) must land at the box top.
    float verts[] = {
        tl.first,  tl.second,  0.0f, 1.0f, // top-left
        tr.first,  tr.second,  1.0f, 1.0f, // top-right
        br.first,  br.second,  1.0f, 0.0f, // bottom-right
        bl.first,  bl.second,  0.0f, 0.0f, // bottom-left
    };

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    glUseProgram(m_previewProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_renderTexture);
    glUniform1i(glGetUniformLocation(m_previewProgram, "uTex"), 0);

    glBindVertexArray(m_previewVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_previewVbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, 0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
}
