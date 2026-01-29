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

#include "player_cpu.h"
#include "logging.h"

#include <chrono>
#include <stdexcept>
#include <thread>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
}

Player::Player(const char *url) : last_fmt(AV_PIX_FMT_NONE) {
    openInputFile(url);
    findVideoStream();
    initializeDecoder();
    allocateFrameResources();
}

Player::~Player() {
    cleanupSwsContext();
    cleanupFrames();
    cleanupPacket();
    cleanupDecoder();
    cleanupFormatContext();
}

void Player::openInputFile(const char *url) {
    if (const auto open_result = avformat_open_input(&format_context, url, nullptr, nullptr); open_result != 0) {
        throw std::runtime_error("Could not open input file");
    }
    if (const auto stream_info_result = avformat_find_stream_info(format_context, nullptr); stream_info_result < 0) {
        avformat_close_input(&format_context);
        throw std::runtime_error("Could not find stream information");
    }
}

void Player::findVideoStream() {
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

void Player::initializeDecoder() {
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

void Player::allocateFrameResources() {
    packet = av_packet_alloc();
    frame = av_frame_alloc();
    yuv420 = av_frame_alloc();
    if (!packet || !frame || !yuv420) {
        throw std::runtime_error("Could not allocate AVPacket/AVFrame");
    }
    configureYUV420Frame();
}

void Player::configureYUV420Frame() const {
    yuv420->format = AV_PIX_FMT_YUV420P;
    yuv420->width = decoder->width;
    yuv420->height = decoder->height;
    if (const auto buffer_result = av_frame_get_buffer(yuv420, 32); buffer_result < 0) {
        throw std::runtime_error("Could not allocate YUV420 buffer");
    }
}

void Player::cleanupSwsContext() {
    if (!sws) {
        return;
    }
    sws_freeContext(sws);
    sws = nullptr;
}

void Player::cleanupFrames() {
    av_frame_free(&yuv420);
    av_frame_free(&frame);
}

void Player::cleanupPacket() {
    av_packet_free(&packet);
}

void Player::cleanupDecoder() {
    avcodec_free_context(&decoder);
}

void Player::cleanupFormatContext() {
    avformat_close_input(&format_context);
}

void Player::recreateSwsContextIfNeeded(const AVPixelFormat fmt) {
    if (sws && fmt == last_fmt) {
        return;
    }
    cleanupSwsContext();
    sws = sws_getContext(
        decoder->width, decoder->height, fmt,
        decoder->width, decoder->height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr
    );
    if (!sws) {
        throw std::runtime_error("Could not create sws context");
    }
    last_fmt = fmt;
}

void Player::makeFrameWritable(AVFrame *dst) {
    if (const auto result = av_frame_make_writable(dst); result < 0) {
        throw std::runtime_error("Could not make frame writable");
    }
}

void Player::scaleFrame(const AVFrame *src, const AVFrame *dst) const {
    sws_scale(sws, src->data, src->linesize, 0, src->height, dst->data, dst->linesize);
}

void Player::copyFrameTimestamps(const AVFrame *src, AVFrame *dst) {
    dst->pts = src->pts;
    dst->best_effort_timestamp = src->best_effort_timestamp;
}

void Player::convertToYUV420(AVFrame *src, AVFrame *dst) {
    const auto fmt = static_cast<AVPixelFormat>(src->format);
    recreateSwsContextIfNeeded(fmt);
    makeFrameWritable(dst);
    scaleFrame(src, dst);
    copyFrameTimestamps(src, dst);
}

int Player::getYPlaneSize() const {
    const auto h = decoder->height;
    const auto linesize = frame->linesize[0];
    return linesize * h;
}

int Player::getUVPlaneSize() const {
    const auto h = decoder->height;
    const auto linesize = frame->linesize[1];
    return linesize * (h / 2);
}

YUVFrameBuffer Player::constructYUVFrameBuffer(const AVFrame *frame_ptr) const {
    const auto w = frame_ptr->width;
    const auto h = frame_ptr->height;
    const auto y_span = std::span<const uint8_t>(frame_ptr->data[0], frame_ptr->linesize[0] * h);
    const auto u_span = std::span<const uint8_t>(frame_ptr->data[1], frame_ptr->linesize[1] * (h / 2));
    const auto v_span = std::span<const uint8_t>(frame_ptr->data[2], frame_ptr->linesize[2] * (h / 2));
    const int64_t ts = frame_ptr->best_effort_timestamp;
    const double pts_sec =
            isValidTimestamp(ts) ? static_cast<double>(ts) * timebase : 0.0;
    return {
        pts_sec,
        y_span, u_span, v_span,
        frame_ptr->linesize[0], frame_ptr->linesize[1], frame_ptr->linesize[2],
        w, h
    };
}

int64_t Player::getFrameTimestamp(const AVFrame *frame_ptr) {
    if (frame_ptr->pts != AV_NOPTS_VALUE) {
        return frame_ptr->pts;
    }
    return frame_ptr->best_effort_timestamp;
}

bool Player::isValidTimestamp(const int64_t ts) {
    return ts != AV_NOPTS_VALUE;
}

void Player::initializeTimingIfNeeded(const int64_t ts) {
    if (timing_started.load()) {
        return;
    }
    start_time = std::chrono::steady_clock::now();
    timing_started = true;
    base_ts = ts;
}

double Player::calculateTargetTime(const int64_t ts) const {
    return static_cast<double>(ts - base_ts) * timebase;
}

void Player::sleepUntilTargetTime(const double target_time) const {
    const auto target_duration = std::chrono::duration<double>(target_time);
    const auto wake_time = start_time + target_duration;
    std::this_thread::sleep_until(wake_time);
}

void Player::sleepForFrame(const AVFrame *frame_ptr) {
    const auto ts = getFrameTimestamp(frame_ptr);
    if (!isValidTimestamp(ts)) {
        return;
    }
    initializeTimingIfNeeded(ts);
    const auto target_time = calculateTargetTime(ts);
    sleepUntilTargetTime(target_time);
}

bool Player::tryReceiveFrame(AVFrame *frame_ptr) const {
    const auto ret = avcodec_receive_frame(decoder, frame_ptr);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        return false;
    }
    if (ret < 0) {
        log_error("Error receiving frame from decoder");
        return false;
    }
    return true;
}

bool Player::needsYUV420Conversion(const AVFrame *frame_ptr) {
    return frame_ptr->format != AV_PIX_FMT_YUV420P;
}

AVFrame *Player::getOutputFrame(AVFrame *frame_ptr, AVFrame *yuv420_ptr) {
    if (!needsYUV420Conversion(frame_ptr)) {
        return frame_ptr;
    }
    convertToYUV420(frame_ptr, yuv420_ptr);
    return yuv420_ptr;
}

void Player::waitWhilePaused() const {
    while (paused.load() && !stopped.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool Player::shouldStopProcessing() const {
    return stopped.load();
}

void Player::processAndDisplayFrame(AVFrame *frame_ptr, AVFrame *yuv420_ptr,
                                    const std::function<void(const YUVFrameBuffer &)> &callback) {
    const auto *output = getOutputFrame(frame_ptr, yuv420_ptr);
    waitWhilePaused();
    if (shouldStopProcessing()) {
        return;
    }
    sleepForFrame(output);
    const auto buffer = constructYUVFrameBuffer(output);
    callback(buffer);
}

void Player::drainDecoder(AVFrame *frame_ptr, AVFrame *yuv420_ptr,
                          const std::function<void(const YUVFrameBuffer &)> &callback) {
    while (true) {
        if (!tryReceiveFrame(frame_ptr)) {
            break;
        }
        processAndDisplayFrame(frame_ptr, yuv420_ptr, callback);
        if (shouldStopProcessing()) {
            break;
        }
    }
}

bool Player::checkExternalStopRequest() const {
    if (!stop_callback) {
        return false;
    }
    return stop_callback();
}

void Player::unreferencePacket() const {
    av_packet_unref(packet);
}

bool Player::isVideoPacket() const {
    return packet->stream_index == video_stream_index;
}

void Player::handleSeekRequest() {
    if (!seek_requested.load()) {
        return;
    }
    const auto target = seek_target.load();
    seek_requested = false;
    performSeek(target);
}

void Player::performSeek(const double time_seconds) {
    const auto timestamp = static_cast<int64_t>(time_seconds / timebase);
    if (const auto seek_result = av_seek_frame(format_context, video_stream_index, timestamp, AVSEEK_FLAG_BACKWARD);
        seek_result < 0) {
        return;
    }
    avcodec_flush_buffers(decoder);
    resetTiming();
    base_ts = timestamp;
}

bool Player::trySendPacket() const {
    if (const auto ret = avcodec_send_packet(decoder, packet); ret < 0 && ret != AVERROR(EAGAIN)) {
        log_error("Error sending packet to decoder");
        return false;
    }
    return true;
}

void Player::processPacket(const std::function<void(const YUVFrameBuffer &)> &callback) {
    if (!trySendPacket()) {
        return;
    }
    drainDecoder(frame, yuv420, callback);
}

bool Player::readNextPacket() const {
    return av_read_frame(format_context, packet) >= 0;
}

bool Player::shouldContinuePlayback() const {
    if (checkExternalStopRequest()) {
        return false;
    }
    if (shouldStopProcessing()) {
        return false;
    }
    return true;
}

void Player::processVideoPacket(const std::function<void(const YUVFrameBuffer &)> &callback) {
    if (!isVideoPacket()) {
        return;
    }
    processPacket(callback);
}

void Player::handlePacket(const std::function<void(const YUVFrameBuffer &)> &callback) {
    handleSeekRequest();
    processVideoPacket(callback);
}

void Player::flushDecoder() const {
    avcodec_send_packet(decoder, nullptr);
}

void Player::drainRemainingFrames(const std::function<void(const YUVFrameBuffer &)> &callback) {
    flushDecoder();
    drainDecoder(frame, yuv420, callback);
}

void Player::play(
    const std::function<void()> &start,
    const std::function<void(const YUVFrameBuffer &)> &callback,
    const std::function<bool()> &should_stop_callback, const std::function<void()> &stop) {
    start();
    stop_callback = should_stop_callback;
    stopped = false;
    resetTiming();
    while (readNextPacket()) {
        if (!shouldContinuePlayback()) {
            unreferencePacket();
            break;
        }
        handlePacket(callback);
        unreferencePacket();
    }
    drainRemainingFrames(callback);
    stop();
}

void Player::savePauseTime() {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - start_time);
    pause_time = elapsed.count();
}

void Player::pause() {
    if (paused.load()) {
        return;
    }
    paused = true;
    savePauseTime();
}

void Player::restoreTimeAfterPause() {
    using Clock = std::chrono::steady_clock;
    const auto pause_duration =
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(pause_time));
    start_time = Clock::now() - pause_duration;
}

void Player::resume() {
    if (!paused.load()) {
        return;
    }
    paused = false;
    restoreTimeAfterPause();
}

void Player::stop() {
    stopped = true;
}

void Player::seek(const double time_seconds) {
    seek_target = time_seconds;
    seek_requested = true;
}

double Player::getDuration() const {
    if (!format_context) {
        return 0.0;
    }
    return static_cast<double>(format_context->duration) / AV_TIME_BASE;
}

double Player::getElapsedTime() const {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - start_time);
    return elapsed.count();
}

double Player::getCurrentTime() const {
    if (!timing_started.load()) {
        return 0.0;
    }
    return getElapsedTime();
}

void Player::resetTiming() {
    timing_started = false;
    pause_time = 0.0;
}
