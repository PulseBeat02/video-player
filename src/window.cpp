#include <stdexcept>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "window.h"
#include "gl_renderer.h"

GLFWwindow *display_window = nullptr;

GLFWwindow *getWindow() {
    return display_window;
}

static void framebuffer_size_callback(GLFWwindow *window, const int fbW, const int fbH) {
    glViewport(0, 0, fbW, fbH);
}

void init_gl_window() {
    if (display_window != nullptr) {
        return;
    }

    glfwInit();
    display_window = glfwCreateWindow(DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_WINDOW_TITLE, nullptr, nullptr);
    glfwMakeContextCurrent(display_window);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
        throw std::runtime_error("GL init failed");
    }

    int fbW, fbH;
    glfwGetFramebufferSize(display_window, &fbW, &fbH);
    glViewport(0, 0, fbW, fbH);
    glfwSetFramebufferSizeCallback(display_window, framebuffer_size_callback);
    glDisable(GL_SCISSOR_TEST);
    init_opengl_renderer(1920, 1080);
}

void destroy_gl_window() {
    if (display_window == nullptr) {
        return;
    }
    destroy_opengl_renderer();
    glfwDestroyWindow(display_window);
    glfwTerminate();
}
