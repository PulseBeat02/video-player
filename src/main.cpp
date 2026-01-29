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

#include "al_renderer.h"
#include "gl_renderer.h"
#include "player_gpu.h"
#include "window.h"

void setup() {
    init_gl_window();
    init_openal_renderer();
}

void teardown() {
    destroy_openal_renderer();
    destroy_gl_window();
}

int main(const int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <video_file>\n", argv[0]);
        return 1;
    }
    PlayerGPU player(argv[1]);
    PlayerEventAdapter adapter;
    adapter.onStart = setup;
    adapter.onStop = teardown;
    adapter.onFrame = display_opengl;
    adapter.onAudio = play_openal;
    adapter.shouldStopPlayback = should_stop;
    player.play(adapter);
    return 0;
}
