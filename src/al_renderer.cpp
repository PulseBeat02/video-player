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

#include <queue>
#include <vector>
#include <AL/al.h>
#include <AL/alc.h>

static ALCdevice *device = nullptr;
static ALCcontext *context = nullptr;
static ALuint source = 0;
static std::vector<ALuint> buffers;
static int gSampleRate = 0;
static int gChannels = 0;
static std::queue<ALuint> freeBuffers;

constexpr int NUM_BUFFERS = 4;
constexpr int NUM_CHANNELS = 2;
constexpr int SAMPLE_RATE = 44100;

void init_openal_renderer() {
    gSampleRate = SAMPLE_RATE;
    gChannels = NUM_CHANNELS;
    device = alcOpenDevice(nullptr);
    if (!device) {
        return;
    }

    context = alcCreateContext(device, nullptr);
    if (!context) {
        alcCloseDevice(device);
        device = nullptr;
        return;
    }
    alcMakeContextCurrent(context);
    alGenSources(1, &source);

    buffers.resize(NUM_BUFFERS);
    alGenBuffers(NUM_BUFFERS, buffers.data());

    while (!freeBuffers.empty()) freeBuffers.pop();
    for (ALuint b: buffers) freeBuffers.push(b);
}

void play_openal(const AudioSampleBuffer &audio_buffer) {
    if (!device || !context) return;

    // Reclaim processed buffers -> freeBuffers
    ALint processed = 0;
    alGetSourcei(source, AL_BUFFERS_PROCESSED, &processed);

    while (processed-- > 0) {
        ALuint b = 0;
        alSourceUnqueueBuffers(source, 1, &b);
        freeBuffers.push(b);
    }

    // Need a free buffer to write into
    if (freeBuffers.empty()) {
        // optional: drop or block; dropping avoids stutter in your decode thread
        return;
    }

    ALuint b = freeBuffers.front();
    freeBuffers.pop();

    // Use actual audio_buffer properties (don’t hardcode globals)
    const int channels = audio_buffer.getChannels();
    const int sampleRate = audio_buffer.getSampleRate();
    const ALenum format = (channels == 1) ? AL_FORMAT_MONO16 : AL_FORMAT_STEREO16;

    alBufferData(b,
                 format,
                 audio_buffer.getData().data(),
                 static_cast<ALsizei>(audio_buffer.getData().size()),
                 sampleRate);

    alSourceQueueBuffers(source, 1, &b);

    // Ensure playback is running
    ALint state = 0;
    alGetSourcei(source, AL_SOURCE_STATE, &state);
    if (state != AL_PLAYING) alSourcePlay(source);
}

void destroy_openal_renderer() {
    if (!device) {
        return;
    }
    if (source) {
        alSourceStop(source);
        alDeleteSources(1, &source);
        source = 0;
    }
    if (!buffers.empty()) {
        alDeleteBuffers(static_cast<ALsizei>(buffers.size()), buffers.data());
        buffers.clear();
    }
    if (context) {
        alcMakeContextCurrent(nullptr);
        alcDestroyContext(context);
        context = nullptr;
    }
    if (device) {
        alcCloseDevice(device);
        device = nullptr;
    }
}
