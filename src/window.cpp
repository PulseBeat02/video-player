// This file is part of Pulse Media Player, a media player written in C++
// Copyright (C) Brandon Li <https://brandonli.me/>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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

void center_gl_window() {
    GLFWmonitor *monitor = glfwGetWindowMonitor(display_window);
    if (!monitor)
        monitor = glfwGetPrimaryMonitor();
    int mx, my;
    glfwGetMonitorPos(monitor, &mx, &my);

    const GLFWvidmode *mode = glfwGetVideoMode(monitor);
    int winW, winH;
    glfwGetWindowSize(display_window, &winW, &winH);
    glfwSetWindowPos(
        display_window,
        mx + (mode->width - winW) / 2,
        my + (mode->height - winH) / 2
    );
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
    init_opengl_renderer(960, 540);
    center_gl_window();
}

void destroy_gl_window() {
    if (display_window == nullptr) {
        return;
    }
    destroy_opengl_renderer();
    glfwDestroyWindow(display_window);
    glfwTerminate();
}
