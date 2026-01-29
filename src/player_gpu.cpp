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

#include "player_gpu.h"
#include "logging.h"

#include <chrono>
#include <stdexcept>

#include "utils.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
#include <libavutil/pixfmt.h>
}

#include <GLFW/glfw3.h>

constexpr double DROP_THRESHOLD_SEC = 0.050;
constexpr double EARLY_EPSILON_SEC = 0.0015;

PlayerGPU::PlayerGPU(const char *url) {
    openInputFile(url);
    findVideoStream();
    findAudioStream();
    initializeDecoder();
    initializeAudioDecoder();
    allocateFrameResources();
}

PlayerGPU::~PlayerGPU() {
    cleanupSwsContext();
    cleanupSwrContext();
    cleanupFrames();
    cleanupAudioFrame();
    cleanupPacket();
    cleanupDecoder();
    cleanupAudioDecoder();
    cleanupFormatContext();
}

void PlayerGPU::openInputFile(const char *url) {
    if (const auto open_result = avformat_open_input(&format_context, url, nullptr, nullptr); open_result != 0) {
        throw std::runtime_error("Could not open input file");
    }
    if (const auto stream_info_result = avformat_find_stream_info(format_context, nullptr); stream_info_result < 0) {
        avformat_close_input(&format_context);
        throw std::runtime_error("Could not find stream information");
    }
}

void PlayerGPU::findVideoStream() {
    const AVCodec *codec_ptr = nullptr;
    video_stream_index = av_find_best_stream(format_context, AVMEDIA_TYPE_VIDEO, -1, -1, &codec_ptr, 0);
    if (video_stream_index < 0) {
        avformat_close_input(&format_context);
        throw std::runtime_error("Could not find video stream");
    }
    const auto *video_stream = format_context->streams[video_stream_index];
    timebase = av_q2d(video_stream->time_base);
    codec = codec_ptr;
}

void PlayerGPU::findAudioStream() {
    const AVCodec *codec_ptr = nullptr;
    audio_stream_index = av_find_best_stream(format_context, AVMEDIA_TYPE_AUDIO, -1, -1, &codec_ptr, 0);
    if (audio_stream_index < 0) {
        return;
    }
    const auto *audio_stream = format_context->streams[audio_stream_index];
    audio_timebase = av_q2d(audio_stream->time_base);
    audio_codec = codec_ptr;
}

void PlayerGPU::initializeDecoder() {
    decoder = avcodec_alloc_context3(codec);
    if (!decoder) {
        avformat_close_input(&format_context);
        throw std::runtime_error("Could not allocate decoder context");
    }
    decoder->thread_count = 0;

    const auto *video_stream = format_context->streams[video_stream_index];
    if (const auto params_result = avcodec_parameters_to_context(decoder, video_stream->codecpar); params_result < 0) {
        avcodec_free_context(&decoder);
        avformat_close_input(&format_context);
        throw std::runtime_error("Could not copy codec parameters");
    }

    if (const auto open_result = avcodec_open2(decoder, codec, nullptr); open_result != 0) {
        avcodec_free_context(&decoder);
        avformat_close_input(&format_context);
        throw std::runtime_error("Could not open codec");
    }
}

void PlayerGPU::initializeAudioDecoder() {
    if (audio_stream_index < 0) {
        return;
    }
    audio_decoder = avcodec_alloc_context3(audio_codec);
    if (!audio_decoder) {
        return;
    }
    const auto *audio_stream = format_context->streams[audio_stream_index];
    if (const auto params_result = avcodec_parameters_to_context(audio_decoder, audio_stream->codecpar);
        params_result < 0) {
        avcodec_free_context(&audio_decoder);
        audio_decoder = nullptr;
        return;
    }
    if (const auto open_result = avcodec_open2(audio_decoder, audio_codec, nullptr); open_result != 0) {
        avcodec_free_context(&audio_decoder);
        audio_decoder = nullptr;
        return;
    }
}

void PlayerGPU::allocateFrameResources() {
    packet = av_packet_alloc();
    frame = av_frame_alloc();
    yuv420 = av_frame_alloc();
    if (!packet || !frame || !yuv420) {
        throw std::runtime_error("Could not allocate AVPacket/AVFrame");
    }
    configureYUV420Frame();
    if (audio_decoder) {
        audio_frame = av_frame_alloc();
        audio_resampled = av_frame_alloc();
        if (!audio_frame || !audio_resampled) {
            throw std::runtime_error("Could not allocate audio AVFrame");
        }
    }
}

void PlayerGPU::configureYUV420Frame() const {
    yuv420->format = AV_PIX_FMT_YUV420P;
    yuv420->width = decoder->width;
    yuv420->height = decoder->height;
    if (const auto buffer_result = av_frame_get_buffer(yuv420, 32); buffer_result < 0) {
        throw std::runtime_error("Could not allocate YUV420 buffer");
    }
}

void PlayerGPU::cleanupSwsContext() {
    if (!sws) return;
    sws_freeContext(sws);
    sws = nullptr;
}

void PlayerGPU::cleanupSwrContext() {
    if (!swr) return;
    swr_free(&swr);
}

void PlayerGPU::cleanupFrames() {
    av_frame_free(&yuv420);
    av_frame_free(&frame);
}

void PlayerGPU::cleanupAudioFrame() {
    av_frame_free(&audio_resampled);
    av_frame_free(&audio_frame);
}

void PlayerGPU::cleanupPacket() {
    av_packet_free(&packet);
}

void PlayerGPU::cleanupDecoder() {
    avcodec_free_context(&decoder);
}

void PlayerGPU::cleanupAudioDecoder() {
    avcodec_free_context(&audio_decoder);
}

void PlayerGPU::cleanupFormatContext() {
    avformat_close_input(&format_context);
}

void PlayerGPU::recreateSwsContextIfNeeded(const AVPixelFormat fmt) {
    if (sws && fmt == last_fmt) return;
    cleanupSwsContext();
    sws = sws_getContext(
        decoder->width, decoder->height, fmt,
        decoder->width, decoder->height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws) {
        throw std::runtime_error("Could not create sws context");
    }
    last_fmt = fmt;
}

void PlayerGPU::makeFrameWritable(AVFrame *dst) {
    if (const auto result = av_frame_make_writable(dst); result < 0) {
        throw std::runtime_error("Could not make frame writable");
    }
}

void PlayerGPU::scaleFrame(const AVFrame *src, const AVFrame *dst) const {
    sws_scale(sws, src->data, src->linesize, 0, src->height, dst->data, dst->linesize);
}

void PlayerGPU::copyFrameTimestamps(const AVFrame *src, AVFrame *dst) {
    dst->pts = src->pts;
    dst->best_effort_timestamp = src->best_effort_timestamp;
}

void PlayerGPU::convertToYUV420(AVFrame *src, AVFrame *dst) {
    const auto fmt = static_cast<AVPixelFormat>(src->format);
    recreateSwsContextIfNeeded(fmt);
    makeFrameWritable(dst);
    scaleFrame(src, dst);
    copyFrameTimestamps(src, dst);
}

int64_t PlayerGPU::getFrameTimestamp(const AVFrame *frame_ptr) {
    if (frame_ptr->pts != AV_NOPTS_VALUE) return frame_ptr->pts;
    return frame_ptr->best_effort_timestamp;
}

bool PlayerGPU::isValidTimestamp(const int64_t ts) {
    return ts != AV_NOPTS_VALUE;
}

void PlayerGPU::resetTiming() {
    timing_started = false;
    pause_time = 0.0;
}

void PlayerGPU::initializeTimingIfNeeded(const int64_t ts) {
    if (timing_started.load()) return;
    start_time = std::chrono::steady_clock::now();
    timing_started = true;
    base_ts = ts;
}

double PlayerGPU::ptsToSeconds(const int64_t ts) const {
    return static_cast<double>(ts - base_ts) * timebase;
}

double PlayerGPU::elapsedSeconds() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - start_time).count();
}

bool PlayerGPU::readNextPacket() const {
    return av_read_frame(format_context, packet) >= 0;
}

bool PlayerGPU::isVideoPacket() const {
    return packet->stream_index == video_stream_index;
}

bool PlayerGPU::isAudioPacket() const {
    return audio_stream_index >= 0 && packet->stream_index == audio_stream_index;
}

bool PlayerGPU::trySendPacket() const {
    if (const auto ret = avcodec_send_packet(decoder, packet); ret < 0 && ret != AVERROR(EAGAIN)) {
        log_error("Error sending packet to decoder");
        return false;
    }
    return true;
}

bool PlayerGPU::tryReceiveFrame(AVFrame *frame_ptr) const {
    const auto ret = avcodec_receive_frame(decoder, frame_ptr);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return false;
    if (ret < 0) {
        log_error("Error receiving frame from decoder");
        return false;
    }
    return true;
}

bool PlayerGPU::needsYUV420Conversion(const AVFrame *frame_ptr) {
    return frame_ptr->format != AV_PIX_FMT_YUV420P;
}

AVFrame *PlayerGPU::getOutputFrame(AVFrame *frame_ptr, AVFrame *yuv420_ptr) {
    if (!needsYUV420Conversion(frame_ptr)) return frame_ptr;
    convertToYUV420(frame_ptr, yuv420_ptr);
    return yuv420_ptr;
}

void PlayerGPU::waitWhilePaused() const {
    while (paused.load() && !stopped.load()) {
        std::this_thread::yield();
    }
}

bool PlayerGPU::checkExternalStopRequest() const {
    return adapter->shouldStopPlayback();
}

void PlayerGPU::handleSeekRequest() {
    if (!seek_requested.load()) return;
    const auto target = seek_target.load();
    seek_requested = false;
    performSeek(target);
}

void PlayerGPU::performSeek(const double time_seconds) {
    adapter->onSeek(time_seconds);
    const auto timestamp = static_cast<int64_t>(time_seconds / timebase);
    if (const auto seek_result = av_seek_frame(format_context, video_stream_index, timestamp, AVSEEK_FLAG_BACKWARD);
        seek_result < 0) {
        return;
    }
    avcodec_flush_buffers(decoder);
    if (audio_decoder) {
        avcodec_flush_buffers(audio_decoder);
    }
    resetTiming();
    base_ts = timestamp;
}

void PlayerGPU::flushDecoder() const {
    avcodec_send_packet(decoder, nullptr);
}

double PlayerGPU::getDuration() const {
    if (!format_context) return 0.0;
    return static_cast<double>(format_context->duration) / AV_TIME_BASE;
}

double PlayerGPU::getCurrentTime() const {
    if (!timing_started.load()) return 0.0;
    return elapsedSeconds();
}

void PlayerGPU::pause() {
    adapter->onPause();
    if (paused.load()) return;
    paused = true;
    const auto now = std::chrono::steady_clock::now();
    pause_time = std::chrono::duration<double>(now - start_time).count();
}

void PlayerGPU::resume() {
    adapter->onResume();
    if (!paused.load()) return;
    paused = false;
    using Clock = std::chrono::steady_clock;
    const auto pause_dur = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(pause_time));
    start_time = Clock::now() - pause_dur;
}

void PlayerGPU::stop() {
    adapter->onStop();
    stopped = true;
}

void PlayerGPU::seek(const double time_seconds) {
    adapter->onSeek(time_seconds);
    seek_target = time_seconds;
    seek_requested = true;
}

FrameCopy PlayerGPU::copyFrame(const AVFrame *yuv420_frame) const {
    FrameCopy buffer_frame;
    buffer_frame.width = yuv420_frame->width;
    buffer_frame.height = yuv420_frame->height;
    buffer_frame.yPitch = yuv420_frame->linesize[0];
    buffer_frame.uPitch = yuv420_frame->linesize[1];
    buffer_frame.vPitch = yuv420_frame->linesize[2];
    const int h = buffer_frame.height;
    const int h2 = h / 2;
    const size_t yBytes = static_cast<size_t>(buffer_frame.yPitch) * static_cast<size_t>(h);
    const size_t uBytes = static_cast<size_t>(buffer_frame.uPitch) * static_cast<size_t>(h2);
    const size_t vBytes = static_cast<size_t>(buffer_frame.vPitch) * static_cast<size_t>(h2);
    buffer_frame.y.resize(yBytes);
    buffer_frame.u.resize(uBytes);
    buffer_frame.v.resize(vBytes);
    std::memcpy(buffer_frame.y.data(), yuv420_frame->data[0], yBytes);
    std::memcpy(buffer_frame.u.data(), yuv420_frame->data[1], uBytes);
    std::memcpy(buffer_frame.v.data(), yuv420_frame->data[2], vBytes);
    if (const auto ts = getFrameTimestamp(yuv420_frame); isValidTimestamp(ts) && timing_started.load()) {
        buffer_frame.pts_sec = ptsToSeconds(ts);
    }
    return buffer_frame;
}

void PlayerGPU::play(PlayerEventAdapter &listener) {
    this->adapter = &listener;
    listener.onStart();
    listener.onPlay();
    glfwSwapInterval(1);

    stopped = false;
    resetTiming();

    const std::function callback =
            [&](const VideoPixelBuffer &f) {
        listener.onFrame(f);
    };
    std::optional<FrameCopy> lastPresented;
    while (readNextPacket()) {
        if (shouldStopPlayback()) {
            av_packet_unref(packet);
            break;
        }
        handleSeekRequest();
        if (isAudioPacket()) {
            processAudioPacket();
            av_packet_unref(packet);
            continue;
        }
        if (!isVideoPacket()) {
            av_packet_unref(packet);
            continue;
        }
        if (!trySendPacket()) {
            av_packet_unref(packet);
            continue;
        }
        decodeAndPresentAvailableFrames(callback, lastPresented);
        av_packet_unref(packet);
    }
    flushDecoder();
    drainDecoderAndPresent(callback, lastPresented);

    if (audio_decoder) {
        avcodec_send_packet(audio_decoder, nullptr);
        while (!shouldStopPlayback()) {
            if (!tryReceiveAudioFrame(audio_frame)) {
                break;
            }
            resampleAudio(audio_frame, audio_resampled);
            const auto buffer = constructAudioBuffer(audio_resampled);
            adapter->onAudio(buffer);
        }
    }

    stop();
    listener.onStop();
}

bool PlayerGPU::shouldStopPlayback() const {
    return stopped.load() || checkExternalStopRequest();
}

void PlayerGPU::pumpPause(
    const std::function<void(const VideoPixelBuffer &)> &onFrame,
    const std::optional<FrameCopy> &lastPresented) const {
    while (paused.load() && !shouldStopPlayback()) {
        if (lastPresented) {
            onFrame(lastPresented->asBuffer());
        } else {
            std::this_thread::yield();
        }
    }
}

void PlayerGPU::presentWhileWaiting(
    const std::function<void(const VideoPixelBuffer &)> &onFrame,
    const std::optional<FrameCopy> &lastPresented,
    const double targetPresentationSec) const {
    if (!lastPresented) return;
    while (!shouldStopPlayback()) {
        pumpPause(onFrame, lastPresented);
        if (shouldStopPlayback()) return;
        if (const double nowSec = elapsedSeconds(); nowSec + EARLY_EPSILON_SEC >= targetPresentationSec) break;
        onFrame(lastPresented->asBuffer());
    }
}

bool PlayerGPU::isLateEnoughToDrop(const double targetPresentationSec) const {
    const double nowSec = elapsedSeconds();
    return (nowSec - targetPresentationSec) > DROP_THRESHOLD_SEC;
}

void PlayerGPU::presentFrameCopy(
    const std::function<void(const VideoPixelBuffer &)> &onFrame,
    std::optional<FrameCopy> &lastPresented,
    FrameCopy frameCopy) {
    onFrame(frameCopy.asBuffer());
    lastPresented = std::move(frameCopy);
}

void PlayerGPU::presentFrameCopyWithPts(
    const std::function<void(const VideoPixelBuffer &)> &onFrame,
    std::optional<FrameCopy> &lastPresented,
    FrameCopy frameCopy,
    const double ptsSec) {
    frameCopy.pts_sec = ptsSec;
    presentFrameCopy(onFrame, lastPresented, std::move(frameCopy));
}

void PlayerGPU::decodeAndPresentAvailableFrames(
    const std::function<void(const VideoPixelBuffer &)> &onFrame,
    std::optional<FrameCopy> &lastPresented) {
    while (true) {
        if (!tryReceiveFrame(frame)) break;
        const AVFrame *outputFrame = getOutputFrame(frame, yuv420);
        const auto timestamp = getFrameTimestamp(outputFrame);
        if (!isValidTimestamp(timestamp)) {
            pumpPause(onFrame, lastPresented);
            if (shouldStopPlayback()) break;
            presentFrameCopy(onFrame, lastPresented, copyFrame(outputFrame));
            continue;
        }
        initializeTimingIfNeeded(timestamp);
        const double targetPresentationSec = ptsToSeconds(timestamp);
        if (isLateEnoughToDrop(targetPresentationSec)) {
            continue;
        }
        presentWhileWaiting(onFrame, lastPresented, targetPresentationSec);
        pumpPause(onFrame, lastPresented);
        if (shouldStopPlayback()) break;
        presentFrameCopyWithPts(onFrame, lastPresented, copyFrame(outputFrame), targetPresentationSec);
    }
}

void PlayerGPU::drainDecoderAndPresent(
    const std::function<void(const VideoPixelBuffer &)> &onFrame,
    std::optional<FrameCopy> &lastPresented) {
    while (!shouldStopPlayback()) {
        if (!tryReceiveFrame(frame)) break;
        const AVFrame *outputFrame = getOutputFrame(frame, yuv420);
        const auto timestamp = getFrameTimestamp(outputFrame);
        if (isValidTimestamp(timestamp)) {
            initializeTimingIfNeeded(timestamp);
            if (const double targetPresentationSec = ptsToSeconds(timestamp);
                !isLateEnoughToDrop(targetPresentationSec)) {
                presentWhileWaiting(onFrame, lastPresented, targetPresentationSec);
                pumpPause(onFrame, lastPresented);
            }
        }
        if (shouldStopPlayback()) break;
        auto frameCopy = copyFrame(outputFrame);
        if (isValidTimestamp(timestamp)) {
            frameCopy.pts_sec = ptsToSeconds(timestamp);
        }
        presentFrameCopy(onFrame, lastPresented, std::move(frameCopy));
    }
}

void PlayerGPU::recreateSwrContextIfNeeded() {
    if (!audio_decoder) {
        return;
    }
    const int sample_rate = audio_decoder->sample_rate;
    const int channels = audio_decoder->ch_layout.nb_channels;
    if (swr && sample_rate == last_sample_rate && channels == last_channels) {
        return;
    }
    cleanupSwrContext();
    int ret = swr_alloc_set_opts2(
        &swr,
        &audio_decoder->ch_layout,
        AV_SAMPLE_FMT_S16,
        sample_rate,
        &audio_decoder->ch_layout,
        audio_decoder->sample_fmt,
        sample_rate,
        0,
        nullptr
    );
    if (ret < 0 || !swr) {
        swr_free(&swr);
        return;
    }

    ret = swr_init(swr);
    if (ret < 0) {
        swr_free(&swr);
        return;
    }
    last_sample_rate = sample_rate;
    last_channels = channels;
}

AudioSampleBuffer PlayerGPU::constructAudioBuffer(const AVFrame *audio_frame_ptr) const {
    const auto data_span = std::span<const uint8_t>(audio_frame_ptr->data[0],
                                                     audio_frame_ptr->nb_samples * last_channels * 2);
    const int64_t ts = audio_frame_ptr->best_effort_timestamp;
    const double pts_sec = isValidTimestamp(ts) ? static_cast<double>(ts) * audio_timebase : 0.0;
    return {
        pts_sec,
        data_span,
        last_sample_rate,
        last_channels,
        audio_frame_ptr->nb_samples
    };
}

bool PlayerGPU::tryReceiveAudioFrame(AVFrame *audio_frame_ptr) const {
    const auto ret = avcodec_receive_frame(audio_decoder, audio_frame_ptr);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        return false;
    }
    if (ret < 0) {
        return false;
    }
    return true;
}

void PlayerGPU::resampleAudio(AVFrame *src, AVFrame *dst) {
    recreateSwrContextIfNeeded();
    if (!swr) {
        return;
    }
    dst->format = AV_SAMPLE_FMT_S16;
    dst->ch_layout = src->ch_layout;
    dst->ch_layout.nb_channels = src->ch_layout.nb_channels;
    dst->sample_rate = src->sample_rate;
    dst->nb_samples = src->nb_samples;
    if (const auto buffer_result = av_frame_get_buffer(dst, 0); buffer_result < 0) {
        return;
    }
    swr_convert(swr, dst->data, dst->nb_samples, const_cast<const uint8_t **>(src->data), src->nb_samples);
    dst->pts = src->pts;
    dst->best_effort_timestamp = src->best_effort_timestamp;
}

void PlayerGPU::processAudioPacket() {
    if (!audio_decoder) {
        return;
    }
    if (const auto ret = avcodec_send_packet(audio_decoder, packet); ret < 0 && ret != AVERROR(EAGAIN)) {
        return;
    }
    while (true) {
        if (!tryReceiveAudioFrame(audio_frame)) {
            break;
        }
        resampleAudio(audio_frame, audio_resampled);
        const auto buffer = constructAudioBuffer(audio_resampled);
        adapter->onAudio(buffer);
        if (shouldStopPlayback()) {
            break;
        }
    }
}
