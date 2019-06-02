/*
 * author : Shuichi TAKANO
 * since  : Mon Jun 03 2019 1:51:44
 */
#ifndef _79DEEE5D_A134_1589_171D_2521C04E08E3
#define _79DEEE5D_A134_1589_171D_2521C04E08E3

#include <stdint.h>
#include <vector>

namespace graphics
{

struct BMP;

inline uint16_t
makeColor(int r, int g, int b)
{
    return ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3);
}

////

class Texture
{
    const uint8_t* bits_{};
    int w_{};
    int h_{};
    int pitch_{};
    int transparentIndex_{-1};
    uint16_t palette_[256];

public:
    Texture() = default;
    Texture(const uint8_t* bits, int w, int h, int pitch)
        : bits_(bits)
        , w_(w)
        , h_(h)
        , pitch_(pitch)
    {
    }

    void initialize(const uint8_t* bits, int w, int h, int pitch)
    {
        bits_  = bits;
        w_     = w;
        h_     = h;
        pitch_ = pitch;
    }

    bool initialize(const BMP* bmp);

    const uint8_t* getBits() const { return bits_; }
    int getWidth() const { return w_; }
    int getHeight() const { return h_; }
    int getPitch() const { return pitch_; }
    int getTransparentIndex() const { return transparentIndex_; }
    const uint16_t* getPalette() const { return palette_; }
};

////

class Framebuffer
{
    std::vector<uint16_t> buffer_;
    int w_{};
    int h_{};

public:
    Framebuffer() = default;
    Framebuffer(int w, int h) { resize(w, h); }

    void resize(int w, int h)
    {
        buffer_.resize(w * h);
        w_ = w;
        h_ = h;
    }

    void put(const Texture& tex, int dx, int dy, int sx, int sy, int w, int h);
    void
    putTrans(const Texture& tex, int dx, int dy, int sx, int sy, int w, int h);

    const uint16_t* getBits() const { return buffer_.data(); }
    int getWidth() const { return w_; }
    int getHeight() const { return h_; }
};

} // namespace graphics

#endif /* _79DEEE5D_A134_1589_171D_2521C04E08E3 */
