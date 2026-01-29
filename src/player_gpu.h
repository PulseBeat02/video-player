/*
 * This file is part of Pulse Media Player, a media player written in C++
 * Copyright (C) Brandon Li <https://brandonli.me/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <thread>
#include "utils.h"

extern "C" {
#include <libavutil/pixfmt.h>
}

struct AVFormatContext;
struct AVCodecContext;
struct AVCodec;
struct AVFrame;
struct AVPacket;
struct SwsContext;

class PlayerGPU {
public:
    explicit PlayerGPU(const char *url);

    ~PlayerGPU();

    PlayerGPU(PlayerGPU &&) = delete;

    PlayerGPU(const PlayerGPU &) = delete;

    PlayerGPU &operator=(const PlayerGPU &) = delete;

    PlayerGPU &operator=(PlayerGPU &&) = delete;

    void play(PlayerEventAdapter &listener);

    bool shouldStopPlayback() const;

    bool isLateEnoughToDrop(double targetPresentationSec) const;

    static void presentFrameCopy(const std::function<void(const YUVFrameBuffer &)> &onFrame,
                                 std::optional<FrameCopy> &lastPresented, FrameCopy frameCopy);

    static void presentFrameCopyWithPts(const std::function<void(const YUVFrameBuffer &)> &onFrame,
                                        std::optional<FrameCopy> &lastPresented, FrameCopy frameCopy,
                                        double ptsSec);

    void decodeAndPresentAvailableFrames(const std::function<void(const YUVFrameBuffer &)> &onFrame,
                                         std::optional<FrameCopy> &lastPresented);

    void drainDecoderAndPresent(const std::function<void(const YUVFrameBuffer &)> &onFrame,
                                std::optional<FrameCopy> &lastPresented);

    void presentWhileWaiting(const std::function<void(const YUVFrameBuffer &)> &onFrame,
                             const std::optional<FrameCopy> &lastPresented, double targetPresentationSec) const;

    void pumpPause(const std::function<void(const YUVFrameBuffer &)> &onFrame,
                   const std::optional<FrameCopy> &lastPresented) const;

    void pause();

    void resume();

    void stop();

    void seek(double time_seconds);

    [[nodiscard]] bool isPaused() const { return paused.load(); }

    [[nodiscard]] bool isStopped() const { return stopped.load(); }

    [[nodiscard]] double getDuration() const;

    [[nodiscard]] double getCurrentTime() const;

private:
    void openInputFile(const char *url);

    void findVideoStream();

    void initializeDecoder();

    void allocateFrameResources();

    void configureYUV420Frame() const;

    void cleanupSwsContext();

    void cleanupFrames();

    void cleanupPacket();

    void cleanupDecoder();

    void cleanupFormatContext();

    void recreateSwsContextIfNeeded(AVPixelFormat fmt);

    static void makeFrameWritable(AVFrame *dst);

    void scaleFrame(const AVFrame *src, const AVFrame *dst) const;

    static void copyFrameTimestamps(const AVFrame *src, AVFrame *dst);

    void convertToYUV420(AVFrame *src, AVFrame *dst);

    static int64_t getFrameTimestamp(const AVFrame *frame_ptr);

    static bool isValidTimestamp(int64_t ts);

    void resetTiming();

    void initializeTimingIfNeeded(int64_t ts);

    [[nodiscard]] double ptsToSeconds(int64_t ts) const;

    [[nodiscard]] double elapsedSeconds() const;

    bool readNextPacket() const;

    [[nodiscard]] bool isVideoPacket() const;

    bool trySendPacket() const;

    bool tryReceiveFrame(AVFrame *frame_ptr) const;

    static bool needsYUV420Conversion(const AVFrame *frame_ptr);

    AVFrame *getOutputFrame(AVFrame *frame_ptr, AVFrame *yuv420_ptr);

    void handleSeekRequest();

    void performSeek(double time_seconds);

    void flushDecoder() const;

    void waitWhilePaused() const;

    [[nodiscard]] bool checkExternalStopRequest() const;

    FrameCopy copyFrame(const AVFrame *yuv420_frame) const;

    AVFormatContext *format_context = nullptr;
    AVCodecContext *decoder = nullptr;
    const AVCodec *codec = nullptr;
    AVPacket *packet = nullptr;
    AVFrame *frame = nullptr;
    AVFrame *yuv420 = nullptr;
    SwsContext *sws = nullptr;
    AVPixelFormat last_fmt = AV_PIX_FMT_NONE;

    int video_stream_index = -1;
    double timebase = 0.0;

    std::chrono::steady_clock::time_point start_time{};
    std::atomic<bool> timing_started{false};
    int64_t base_ts = 0;
    double pause_time = 0.0;

    std::atomic<bool> paused{false};
    std::atomic<bool> stopped{false};
    std::atomic<bool> seek_requested{false};
    std::atomic<double> seek_target{0.0};

    std::thread decode_thread;
    std::thread render_thread;

    FrameQueue queue{10};
    PlayerEventAdapter *adapter = nullptr;
};
