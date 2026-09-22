#pragma once

#include "common.h"
#include "mirror_profile.h"
#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include <random>

class Config;
class Renderer;

class Camera
{
public:
    void init(Config& cfg, const MirrorProfile& mirror, float cameraHeight, Renderer* renderer);
    void shutdown();

    // Render robot camera view into m_imageData. drawScene renders the world
    // (field + robot body) with the given Renderer, using whatever
    // projection/view the Renderer currently has set.
    void renderView(const glm::vec3& robotPos, float robotYaw,
                    const std::function<void(Renderer&)>& drawScene);

    // Get the rendered image data (top-down RGB)
    const std::vector<uint8_t>& imageData() const { return m_imageData; }
    int imageWidth()  const { return m_width; }
    int imageHeight() const { return m_height; }

    // True if the most recent renderView() call actually re-rendered the
    // image (vs. reusing the previous one because /camera/stream_fps hadn't
    // elapsed yet). Callers that publish frames to subscribers (GrpcServer)
    // should gate on this so they don't broadcast duplicate frames faster
    // than the camera actually produces new ones.
    bool frameChanged() const { return m_frameChanged; }

    // Draw a small preview of the camera image into a corner of the window.
    void drawPreview(int windowWidth, int windowHeight);

private:
    int m_width  = 640;
    int m_height = 480;
    float m_fov  = 120.0f;
    float m_noiseStd = 0.02f;
    float m_pixelNoise = 0.01f;
    int m_cubemapRes = 256;
    float m_streamFps = 30.0f;
    glm::vec3 m_background = glm::vec3(0.0f);

    MirrorProfile m_mirror;
    float m_cameraHeight = 0.11f;
    Renderer* m_renderer = nullptr;

    std::vector<uint8_t> m_imageData; // RGB, top-down
    std::vector<float> m_lut;         // width*height*4, RGBA32F (dir + valid)
    bool m_frameChanged = false;

    // Final composited image FBO
    unsigned int m_fbo = 0;
    unsigned int m_renderTexture = 0;
    unsigned int m_depthRbo = 0;

    // Cubemap capture FBO + texture
    unsigned int m_cubemapFbo = 0;
    unsigned int m_cubemapTexture = 0;
    unsigned int m_cubemapDepthRbo = 0;

    // LUT texture
    unsigned int m_lutTexture = 0;

    // Compositing shader + fullscreen quad
    unsigned int m_compositeProgram = 0;
    unsigned int m_quadVao = 0, m_quadVbo = 0, m_quadEbo = 0;

    // Preview overlay shader + quad
    unsigned int m_previewProgram = 0;
    unsigned int m_previewVao = 0, m_previewVbo = 0, m_previewEbo = 0;

    // stream_fps gating
    std::chrono::steady_clock::time_point m_lastRender;

    std::mt19937 m_rng{42};

    void createFramebuffers();
    void bakeLut();
    void initComposite();
    void initPreview();
    void captureCubemap(const glm::vec3& worldViewpoint,
                        const std::function<void(Renderer&)>& drawScene);
    void composite(float robotYaw);
    void readback();
    void applyNoise();
};
