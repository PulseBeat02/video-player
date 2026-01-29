#pragma once
#include <span>
#include <cstdint>

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
