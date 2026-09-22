#pragma once

// OpenGL on macOS
#if defined(__APPLE__)
    #define GL_SILENCE_DEPRECATION
    #include <OpenGL/gl3.h>
    #include <OpenGL/gl3ext.h>
#else
    #include <GL/gl3.h>
    #include <GL/glext.h>
#endif

// GLM with experimental extensions
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// lerp helper (C++17)
namespace util {
    inline float lerp(float a, float b, float t) {
        return a + (b - a) * t;
    }
}
