/*
 * author : Shuichi TAKANO
 * since  : Sun Mar 24 2019 1:15:10
 */
#ifndef FBEF24A5_7134_145C_1195_8E9120056B14
#define FBEF24A5_7134_145C_1195_8E9120056B14

#include <stdint.h>
#include <stdlib.h>

namespace graphics
{

struct WORD
{
    uint8_t data[2];
    operator uint16_t() const { return data[0] | (data[1] << 8); }
};

struct DWORD
{
    uint8_t data[4];
    operator uint32_t() const
    {
        return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    }
};

using BYTE = uint8_t;
using LONG = DWORD;

struct BITMAPINFOHEADER
{
    DWORD biSize;
    LONG biWidth;
    LONG biHeight;
    WORD biPlanes;
    WORD biBitCount;
    DWORD biCompression;
    DWORD biSizeImage;
    LONG biXPelsPerMeter;
    LONG biYPelsPerMeter;
    DWORD biClrUsed;
    DWORD biClrImportant;

public:
    size_t getPaletteCount() const
    {
        return biClrUsed ? biClrUsed
                         : ((biBitCount <= 8) ? 1 << biBitCount : 0);
    }
};

struct RGBQUAD
{
    BYTE rgbBlue;
    BYTE rgbGreen;
    BYTE rgbRed;
    BYTE rgbReserved;

public:
    uint8_t getR() const { return rgbRed; }
    uint8_t getG() const { return rgbGreen; }
    uint8_t getB() const { return rgbBlue; }
    uint8_t getA() const { return rgbReserved; }
};

struct BITMAPINFO
{
    BITMAPINFOHEADER bmiHeader;
    RGBQUAD bmiColors[1];
};

struct BITMAPFILEHEADER
{
    WORD bfType;
    DWORD bfSize;
    WORD bfReserved1;
    WORD bfReserved2;
    DWORD bfOffBits;

public:
    bool checkType() const { return bfType == 0x4d42; }
};

struct BMP
{
    BITMAPFILEHEADER fileHeader;
    BITMAPINFO info;

public:
    bool isBMP() const { return fileHeader.checkType(); }
    const BITMAPINFOHEADER& getInfoHeader() const { return info.bmiHeader; }

    size_t getSize() const { return fileHeader.bfSize; }
    size_t getWidth() const { return getInfoHeader().biWidth; }
    size_t getHeight() const { return getInfoHeader().biHeight; }
    size_t getBitCount() const { return getInfoHeader().biBitCount; }
    size_t getPaletteCount() const { return getInfoHeader().getPaletteCount(); }

    const void* getBits() const { return (char*)this + fileHeader.bfOffBits; }
    const RGBQUAD* getPalette() const { return info.bmiColors; }
};

} // namespace graphics

#endif /* FBEF24A5_7134_145C_1195_8E9120056B14 */
