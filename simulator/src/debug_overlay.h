#pragma once

#include <string>
#include <vector>

// On-screen text HUD (toggled by the L key, same pattern as C for the camera
// preview) showing live robot/dribbler/kicker telemetry -- motor speeds,
// commanded vs. actual state, etc. Renders with stb_easy_font (vendored in
// third_party/), which builds ASCII text out of flat-shaded quads -- no font
// texture or external text-rendering dependency needed.
class DebugOverlay
{
public:
    ~DebugOverlay();

    void init();
    void shutdown();

    // Draws `lines` (one string per line, top-down) in a translucent box in
    // the given corner of the window. windowWidth/windowHeight are the
    // current framebuffer size, same as Camera::drawPreview's parameters.
    void draw(int windowWidth, int windowHeight, const std::vector<std::string>& lines);

private:
    unsigned int m_textProgram = 0;
    unsigned int m_boxProgram = 0;
    unsigned int m_textVao = 0, m_textVbo = 0, m_textEbo = 0;
    unsigned int m_boxVao = 0, m_boxVbo = 0;
    int m_maxQuads = 0;
};
