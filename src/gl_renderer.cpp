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

#include "gl_renderer.h"

#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

static GLuint texY, texUV;
static GLuint prog;
static GLuint quadVAO, quadVBO;
static int gW, gH;
static GLint locY, locUV;
static std::vector<uint8_t> uvPacked;

static auto YUV_FRAG = R"(#version 330 core
in vec2 vUV;
out vec4 FragColor;

uniform sampler2D Y;
uniform sampler2D UV;

void main() {
    float y = texture(Y, vUV).r;
    vec2 uv = texture(UV, vUV).rg;
    float u = uv.r - 0.5;
    float v = uv.g - 0.5;
    y = (y - 0.0625) * 1.1643;
    vec3 rgb = vec3(
        y + 1.5958 * v,
        y - 0.3917 * u - 0.8129 * v,
        y + 2.0170 * u
    );
    FragColor = vec4(rgb, 1.0);
}
)";

static auto YUV_VERT = R"(#version 330 core
layout(location=0) in vec2 pos;
layout(location=1) in vec2 uv;

out vec2 vUV;

void main() {
    vUV = uv;
    gl_Position = vec4(pos,0,1);
}
)";

static GLuint compile(const GLenum type, const char *src) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    return shader;
}

static GLuint make_program() {
    const GLuint vShader = compile(GL_VERTEX_SHADER, YUV_VERT);
    const GLuint fShader = compile(GL_FRAGMENT_SHADER, YUV_FRAG);
    const GLuint program = glCreateProgram();
    glAttachShader(program, vShader);
    glAttachShader(program, fShader);
    glLinkProgram(program);
    glDeleteShader(vShader);
    glDeleteShader(fShader);
    return program;
}

static void make_tex_r8(GLuint &texture, const int width, const int height) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
}

static void make_tex_rg8(GLuint &texture, const int width, const int height) {
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, width, height, 0, GL_RG, GL_UNSIGNED_BYTE, nullptr);
}

constexpr float quad[] = {
    -1, -1, 0, 1,
    1, -1, 1, 1,
    1, 1, 1, 0,
    -1, -1, 0, 1,
    1, 1, 1, 0,
    -1, 1, 0, 0
};

static void upload_r8(const GLuint texture, const uint8_t *data, const int stridePixels, const int width,
                      const int height) {
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, stridePixels);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_BYTE, data);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

static void upload_rg8(const GLuint texture, const uint8_t *data, const int stridePixels, const int width,
                       const int height) {
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, stridePixels);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RG, GL_UNSIGNED_BYTE, data);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
}

static void pack_uv420(const uint8_t *u, const int uPitchBytes, const uint8_t *v, const int vPitchBytes, const int w2,
                       const int h2) {
    const int dstStrideBytes = w2 * 2;
    uvPacked.resize(static_cast<size_t>(dstStrideBytes) * static_cast<size_t>(h2));
    uint8_t *dst = uvPacked.data();
    for (int y = 0; y < h2; ++y) {
        const uint8_t *ur = u + y * uPitchBytes;
        const uint8_t *vr = v + y * vPitchBytes;
        uint8_t *dr = dst + y * dstStrideBytes;
        int x = 0;
        for (; x + 8 <= w2; x += 8) {
            dr[0] = ur[0];
            dr[1] = vr[0];
            dr[2] = ur[1];
            dr[3] = vr[1];
            dr[4] = ur[2];
            dr[5] = vr[2];
            dr[6] = ur[3];
            dr[7] = vr[3];
            dr[8] = ur[4];
            dr[9] = vr[4];
            dr[10] = ur[5];
            dr[11] = vr[5];
            dr[12] = ur[6];
            dr[13] = vr[6];
            dr[14] = ur[7];
            dr[15] = vr[7];
            ur += 8;
            vr += 8;
            dr += 16;
        }
        for (; x < w2; ++x) {
            dr[0] = *ur++;
            dr[1] = *vr++;
            dr += 2;
        }
    }
}


void init_opengl_renderer(const int width, const int height) {
    gW = width;
    gH = height;
    make_tex_r8(texY, gW, gH);
    make_tex_rg8(texUV, gW / 2, gH / 2);
    prog = make_program();
    locY = glGetUniformLocation(prog, "Y");
    locUV = glGetUniformLocation(prog, "UV");
    glUseProgram(prog);
    glUniform1i(locY, 0);
    glUniform1i(locUV, 1);
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<void *>(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    if (auto *win = glfwGetCurrentContext()) {
        glfwSetWindowAspectRatio(win, gW, gH);
        glfwSetWindowSize(win, gW, gH);
    }
}

void display_opengl(const VideoPixelBuffer &frame_buffer) {
    auto *win = glfwGetCurrentContext();
    if (!win || glfwWindowShouldClose(win)) {
        return;
    }
    int fbW, fbH;

    glfwGetFramebufferSize(win, &fbW, &fbH);
    glfwSetWindowAspectRatio(win, gW, gH);

    const int w = frame_buffer.getWidth();
    const int h = frame_buffer.getHeight();
    if (w != gW || h != gH) {
        gW = w;
        gH = h;
        glfwSetWindowAspectRatio(win, gW, gH);
        glDeleteTextures(1, &texY);
        glDeleteTextures(1, &texUV);
        make_tex_r8(texY, gW, gH);
        make_tex_rg8(texUV, gW / 2, gH / 2);
    }

    const int w2 = w / 2;
    const int h2 = h / 2;

    upload_r8(texY, frame_buffer.getYPlaneBuffer().data(), frame_buffer.getYPitch(), w, h);

    pack_uv420(
        frame_buffer.getUPlaneBuffer().data(), frame_buffer.getUPitch(),
        frame_buffer.getVPlaneBuffer().data(), frame_buffer.getVPitch(),
        w2, h2
    );
    upload_rg8(texUV, uvPacked.data(), w2, w2, h2);

    glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(prog);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texY);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texUV);
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    glfwSwapBuffers(glfwGetCurrentContext());
    glfwPollEvents();
}

void destroy_opengl_renderer() {
    glDeleteTextures(1, &texY);
    glDeleteTextures(1, &texUV);
    glDeleteVertexArrays(1, &quadVAO);
    glDeleteBuffers(1, &quadVBO);
    glDeleteProgram(prog);
    uvPacked.clear();
    uvPacked.shrink_to_fit();
}
