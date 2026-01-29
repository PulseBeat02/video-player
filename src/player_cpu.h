#pragma once
#pragma message("Header `foo.h` is deprecated!")

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>

#include "utils.h"

extern "C" {
#include <libavutil/pixfmt.h>
}

constexpr long long NANOSECONDS_PER_SECOND = 1000000000LL;

struct AVFormatContext;
struct AVCodecContext;
struct AVCodec;
struct AVStream;
struct AVFrame;
struct AVPacket;
struct SwsContext;


class Player {
public:
    explicit Player(const char *url);

    ~Player();

    Player(Player &&) = delete;

    Player(const Player &) = delete;

    Player &operator=(const Player &) = delete;

    Player &operator=(Player &&) = delete;

    void play(
        const std::function<void()> &start,
        const std::function<void(const YUVFrameBuffer &)> &callback,
        const std::function<bool()> &should_stop_callback, const std::function<void()> &stop);

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

    [[nodiscard]] int getYPlaneSize() const;

    [[nodiscard]] int getUVPlaneSize() const;

    [[nodiscard]] YUVFrameBuffer constructYUVFrameBuffer(const AVFrame *frame_ptr) const;

    [[nodiscard]] static int64_t getFrameTimestamp(const AVFrame *frame_ptr);

    static bool isValidTimestamp(int64_t ts);

    void initializeTimingIfNeeded(int64_t ts);

    [[nodiscard]] double calculateTargetTime(int64_t ts) const;

    void sleepUntilTargetTime(double target_time) const;

    void sleepForFrame(const AVFrame *frame_ptr);

    bool tryReceiveFrame(AVFrame *frame_ptr) const;

    static bool needsYUV420Conversion(const AVFrame *frame_ptr);

    AVFrame *getOutputFrame(AVFrame *frame_ptr, AVFrame *yuv420_ptr);

    void waitWhilePaused() const;

    [[nodiscard]] bool shouldStopProcessing() const;

    void processAndDisplayFrame(AVFrame *frame_ptr, AVFrame *yuv420_ptr,
                                const std::function<void(const YUVFrameBuffer &)> &callback);

    void drainDecoder(AVFrame *frame_ptr, AVFrame *yuv420_ptr,
                      const std::function<void(const YUVFrameBuffer &)> &callback);

    bool checkExternalStopRequest() const;

    void unreferencePacket() const;

    [[nodiscard]] bool isVideoPacket() const;

    void handleSeekRequest();

    void performSeek(double time_seconds);

    bool trySendPacket() const;

    void processPacket(const std::function<void(const YUVFrameBuffer &)> &callback);

    bool readNextPacket() const;

    bool shouldContinuePlayback() const;

    void processVideoPacket(const std::function<void(const YUVFrameBuffer &)> &callback);

    void handlePacket(const std::function<void(const YUVFrameBuffer &)> &callback);

    void flushDecoder() const;

    void drainRemainingFrames(const std::function<void(const YUVFrameBuffer &)> &callback);

    void savePauseTime();

    void restoreTimeAfterPause();

    [[nodiscard]] double getElapsedTime() const;

    void resetTiming();

    AVFormatContext *format_context = nullptr;
    AVCodecContext *decoder = nullptr;
    const AVCodec *codec = nullptr;
    AVPacket *packet = nullptr;
    AVFrame *frame = nullptr;
    AVFrame *yuv420 = nullptr;
    SwsContext *sws = nullptr;
    AVPixelFormat last_fmt;

    int video_stream_index = -1;
    double timebase = 0.0;

    std::chrono::steady_clock::time_point start_time;
    std::atomic<bool> timing_started{false};
    int64_t base_ts = 0;
    double pause_time = 0.0;

    std::atomic<bool> paused{false};
    std::atomic<bool> stopped{false};
    std::atomic<bool> seek_requested{false};
    std::atomic<double> seek_target{0.0};
    std::function<bool()> stop_callback;
};
