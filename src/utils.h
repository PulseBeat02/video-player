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

#include <span>
#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>
#include <functional>

class YUVFrameBuffer {
    double pts = 0.0;
    std::span<const uint8_t> y;
    std::span<const uint8_t> u;
    std::span<const uint8_t> v;
    int yPitch = 0;
    int uPitch = 0;
    int vPitch = 0;
    int width = 0;
    int height = 0;

public:
    YUVFrameBuffer() = default;

    YUVFrameBuffer(const double pts,
                   const std::span<const uint8_t> y,
                   const std::span<const uint8_t> u,
                   const std::span<const uint8_t> v,
                   const int yPitch, const int uPitch, const int vPitch,
                   const int width, const int height)
        : pts(pts), y(y), u(u), v(v),
          yPitch(yPitch), uPitch(uPitch), vPitch(vPitch),
          width(width), height(height) {
    }

    [[nodiscard]] double getPts() const { return pts; }
    [[nodiscard]] std::span<const uint8_t> getYPlaneBuffer() const { return y; }
    [[nodiscard]] std::span<const uint8_t> getUPlaneBuffer() const { return u; }
    [[nodiscard]] std::span<const uint8_t> getVPlaneBuffer() const { return v; }

    [[nodiscard]] int getYPitch() const { return yPitch; }
    [[nodiscard]] int getUPitch() const { return uPitch; }
    [[nodiscard]] int getVPitch() const { return vPitch; }
    [[nodiscard]] int getWidth() const { return width; }
    [[nodiscard]] int getHeight() const { return height; }

    [[nodiscard]] const uint8_t *getYRow(const int r) const { return y.data() + r * yPitch; }
    [[nodiscard]] const uint8_t *getURow(const int r) const { return u.data() + r * uPitch; }
    [[nodiscard]] const uint8_t *getVRow(const int r) const { return v.data() + r * vPitch; }
};

struct FrameCopy {
    double pts_sec = 0.0;
    int width = 0;
    int height = 0;
    int yPitch = 0;
    int uPitch = 0;
    int vPitch = 0;
    std::vector<uint8_t> y;
    std::vector<uint8_t> u;
    std::vector<uint8_t> v;

    [[nodiscard]] YUVFrameBuffer asBuffer() const {
        return {
            pts_sec,
            std::span(y.data(), y.size()),
            std::span(u.data(), u.size()),
            std::span(v.data(), v.size()),
            yPitch, uPitch, vPitch,
            width, height
        };
    }
};

class FrameQueue {
public:
    explicit FrameQueue(const size_t max_frames) : max_frames(max_frames) {
    }

    void stop() { {
            std::lock_guard lk(lock);
            stopped = true;
            buffer.clear();
        }
        cv_not_empty.notify_all();
        cv_not_full.notify_all();
    }

    void clear() { {
            std::lock_guard lk(lock);
            buffer.clear();
        }
        cv_not_full.notify_all();
    }

    void push(FrameCopy &&f) {
        std::unique_lock lk(lock);
        cv_not_full.wait(lk, [&] { return stopped || buffer.size() < max_frames; });
        if (stopped) return;
        buffer.push_back(std::move(f));
        cv_not_empty.notify_one();
    }

    bool peek(FrameCopy &out) const {
        std::lock_guard lk(lock);
        if (buffer.empty()) return false;
        out = buffer.front();
        return true;
    }

    bool popBlocking(FrameCopy &out) {
        std::unique_lock lk(lock);
        cv_not_empty.wait(lk, [&] { return stopped || !buffer.empty(); });
        if (stopped) return false;
        out = std::move(buffer.front());
        buffer.pop_front();
        cv_not_full.notify_one();
        return true;
    }

    bool tryPop(FrameCopy &out) {
        std::lock_guard lk(lock);
        if (buffer.empty()) return false;
        out = std::move(buffer.front());
        buffer.pop_front();
        cv_not_full.notify_one();
        return true;
    }

    template<class Pred>
    void dropWhile(Pred pred) {
        std::lock_guard lk(lock);
        while (!buffer.empty() && pred(buffer.front())) {
            buffer.pop_front();
        }
        cv_not_full.notify_all();
    }

    size_t size() const {
        std::lock_guard lk(lock);
        return buffer.size();
    }

private:
    mutable std::mutex lock;
    std::condition_variable cv_not_empty;
    std::condition_variable cv_not_full;
    std::deque<FrameCopy> buffer;
    size_t max_frames = 0;
    bool stopped = false;
};

struct PlayerEventAdapter {
    std::function<void()> onStart = [] {
    };
    std::function<void()> onPlay = [] {
    };
    std::function<void()> onPause = [] {
    };
    std::function<void()> onResume = [] {
    };
    std::function<void()> onStop = [] {
    };
    std::function<void(double)> onSeek = [](double) {
    };
    std::function<bool()> shouldStopPlayback = [] { return false; };
    std::function<void(const YUVFrameBuffer &)> onFrame = [](const YUVFrameBuffer &) {
    };
};
