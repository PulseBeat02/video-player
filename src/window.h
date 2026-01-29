#pragma once

#include <functional>
#include <GLFW/glfw3.h>

constexpr int DEFAULT_WIDTH = 1920;
constexpr int DEFAULT_HEIGHT = 1080;
constexpr auto DEFAULT_WINDOW_TITLE = "Pulse Media Player";

GLFWwindow* getWindow();

void init_gl_window();
void destroy_gl_window();

inline bool should_stop() {
    return glfwWindowShouldClose(getWindow());
}
