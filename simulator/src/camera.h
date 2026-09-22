#pragma once

#include "common.h"
#include <vector>

class Config;

class Camera
{
public:
    void init(Config& cfg);
    void update();

    // Render robot camera view to a buffer
    void renderView(const glm::vec3& robotPos, float robotYaw);

    // Get the rendered image data
    const std::vector<uint8_t>& imageData() const { return m_imageData; }
    int imageWidth()  const { return m_width; }
    int imageHeight() const { return m_height; }

    // Mirror parameters
    struct Mirror {
        float a = 0.035f;   // hyperbola parameter a
        float b = 0.04f;    // hyperbola parameter b
        float radius = 0.025f;
    };
    const Mirror& mirror() const { return m_mirror; }

private:
    int m_width  = 640;
    int m_height = 480;
    float m_fov  = 180.0f;
    float m_noiseStd = 0.02f;
    float m_pixelNoise = 0.01f;

    Mirror m_mirror;
    std::vector<uint8_t> m_imageData; // RGB

    // Framebuffer for offscreen rendering
    unsigned int m_fbo = 0;
    unsigned int m_renderTexture = 0;

    void createFramebuffer();
};
