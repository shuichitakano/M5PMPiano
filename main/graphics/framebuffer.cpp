/*
 * author : Shuichi TAKANO
 * since  : Mon Jun 03 2019 2:4:36
 */

#include "framebuffer.h"
#include "bmp.h"

namespace graphics
{

bool
Texture::initialize(const BMP* bmp)
{
    if (!bmp->isBMP() || bmp->getBitCount() != 8)
    {
        return false;
    }

    auto w     = bmp->getWidth();
    auto h     = bmp->getHeight();
    auto pitch = ((w + 3) & ~3);
    auto bits  = static_cast<const uint8_t*>(bmp->getBits()) + pitch * (h - 1);
    initialize(bits, w, h, -pitch);

    int nPal  = bmp->getPaletteCount();
    auto* pal = bmp->getPalette();
    for (int i = 0; i < nPal; ++i)
    {
        int r       = pal->getR();
        int g       = pal->getG();
        int b       = pal->getB();
        palette_[i] = makeColor(r, g, b);
        ++pal;

        if (r == 255 && g == 0 && b == 255)
        {
            transparentIndex_ = i;
        }
    }

    return true;
}

/////////////////////////////////

void
Framebuffer::put(
    const Texture& tex, int dx, int dy, int sx, int sy, int w, int h)
{
    auto dstPitch = getWidth();
    auto* dst     = buffer_.data() + dx + dstPitch * dy;

    auto srcPitch   = tex.getPitch();
    const auto* src = tex.getBits() + sx + srcPitch * sy;
    const auto* pal = tex.getPalette();

    while (h)
    {
        auto td = dst;
        auto ts = src;
        auto tw = w;
        while (tw)
        {
            *td = pal[*ts];
            ++td;
            ++ts;
            --tw;
        }

        dst += dstPitch;
        src += srcPitch;
        --h;
    }
}

void
Framebuffer::putTrans(
    const Texture& tex, int dx, int dy, int sx, int sy, int w, int h)
{
    auto dstPitch = getWidth();
    auto* dst     = buffer_.data() + dx + dstPitch * dy;

    auto srcPitch   = tex.getPitch();
    const auto* src = tex.getBits() + sx + srcPitch * sy;
    const auto* pal = tex.getPalette();
    int trans       = tex.getTransparentIndex();

    while (h)
    {
        auto td = dst;
        auto ts = src;
        auto tw = w;
        while (tw)
        {
            int i = *ts;
            if (i != trans)
            {
                *td = pal[i];
            }
            ++td;
            ++ts;
            --tw;
        }

        dst += dstPitch;
        src += srcPitch;
        --h;
    }
}

} // namespace graphics
