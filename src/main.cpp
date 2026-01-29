#include "gl_renderer.h"
#include "player_cpu.h"
#include "player_gpu.h"
#include "window.h"

int main(const int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <video_file>\n", argv[0]);
        return 1;
    }
    PlayerGPU player(argv[1]);
    player.play(init_gl_window, display_opengl, should_stop, destroy_gl_window);
    return 0;
}
