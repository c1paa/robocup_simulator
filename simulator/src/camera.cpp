#include "camera.h"
#include "config.h"
#include <iostream>
#include <random>
#include <cstring>

static const float MM = 1000.0f;

void Camera::init(Config& cfg)
{
    m_width  = cfg.getInt("/camera/width",  640);
    m_height = cfg.getInt("/camera/height",  480);
    m_fov    = cfg.getFloat("/camera/fov",   180.0f);
    m_noiseStd  = cfg.getFloat("/camera/noise_std",  0.02f);
    m_pixelNoise= cfg.getFloat("/camera/pixel_noise", 0.01f);
    m_mirror.a  = cfg.getFloat("/camera/mirror_a", 35.0f) / MM;
    m_mirror.b  = cfg.getFloat("/camera/mirror_b", 40.0f) / MM;
    m_mirror.radius = cfg.getFloat("/camera/mirror_radius", 25.0f) / MM;

    m_imageData.resize(m_width * m_height * 3);

    createFramebuffer();
    std::cout << "[Camera] " << m_width << "x" << m_height << " FOV=" << m_fov << " deg" << std::endl;
}

void Camera::createFramebuffer()
{
    glGenFramebuffers(1, &m_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    glGenTextures(1, &m_renderTexture);
    glBindTexture(GL_TEXTURE_2D, m_renderTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_width, m_height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_renderTexture, 0);

    // Depth buffer
    unsigned int rbo;
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_width, m_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "[Camera] Framebuffer not complete!" << std::endl;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Camera::update()
{
    // Render robot view into framebuffer, then read back pixels
    // Placeholder: generate synthetic test pattern with noise
    
    static std::mt19937 rng(42);
    static std::normal_distribution<float> noise(0.0f, 1.0f);

    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            int idx = (y * m_width + x) * 3;

            // Simple test pattern: green field with white lines
            float base = (x < m_width / 2) ? 0.05f : 0.3f;
            float n = noise(rng) * m_noiseStd;

            m_imageData[idx + 0] = (uint8_t)std::clamp((base + n) * 255.0f, 0.0f, 255.0f);
            m_imageData[idx + 1] = (uint8_t)std::clamp((0.3f + n) * 255.0f, 0.0f, 255.0f);
            m_imageData[idx + 2] = (uint8_t)std::clamp((0.1f + n) * 255.0f, 0.0f, 255.0f);
        }
    }
}

void Camera::renderView(const glm::vec3& robotPos, float robotYaw)
{
    (void)robotPos;
    (void)robotYaw;
    // Will render scene from robot's mirror camera perspective
    update();
}
