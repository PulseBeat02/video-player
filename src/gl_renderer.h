#pragma once
#include "utils.h"

void init_opengl_renderer(int width, int height);

void display_opengl(const YUVFrameBuffer &frame_buffer);

void destroy_opengl_renderer();
