/*
 * author : Shuichi TAKANO
 * since  : Thu May 09 2019 2:10:43
 */

#include "soundboard.h"
#include <assert.h>

namespace physical_modeling_piano
{

namespace
{
constexpr size_t
getDelayLength(int i)
{
    constexpr size_t delayLengths[] = {37, 87, 181, 271, 359, 592, 687, 721};
    return convertSampleSize(delayLengths[i]);
}

} // namespace

void
Soundboard::initialize(const SystemParameters& sysParams)
{
    a_ = sysParams.soundboardFeedback;

    size_t delaySize = 0;

    for (int i = 0; i < 8; ++i)
    {
        auto delay = getDelayLength(i);
        decay_[i].constant.initialize(sysParams.sampleRate / delay,
                                      sysParams.sampleRate,
                                      sysParams.soundboardLossC1,
                                      sysParams.soundboardLossC3);

        delaySize += computeDelayBufferSize(delay);
    }

    printf("delay size = %zd\n", delaySize);
    delayBuffer_.resize(delaySize);

    size_t ofs = 0;
    for (int i = 0; i < 8; ++i)
    {
        auto delay = getDelayLength(i);
        auto size  = computeDelayBufferSize(delay);
        delays_[i].attachBuffer(&delayBuffer_[ofs], size);
        ofs += size;
    }
    assert(ofs == delayBuffer_.size());
}

void
Soundboard::setScale(float s)
{
    scale_ = s / 8.0f;
}

void
Soundboard::update(ResultT* dst, const ValueT* src, size_t nSamples)
{
    while (nSamples)
    {
        ValueT t;
        mul(t, ot_, a_);
        add(t, t, *src);

        ValueT i[8];
        add(i[0], t, o_[1]);
        add(i[1], t, o_[2]);
        add(i[2], t, o_[3]);
        add(i[3], t, o_[4]);
        add(i[4], t, o_[5]);
        add(i[5], t, o_[6]);
        add(i[6], t, o_[7]);
        add(i[7], t, o_[0]);

        o_[0] = decay_[0].filter(delays_[0].update(i[0], getDelayLength(0)));
        o_[1] = decay_[1].filter(delays_[1].update(i[1], getDelayLength(1)));
        o_[2] = decay_[2].filter(delays_[2].update(i[2], getDelayLength(2)));
        o_[3] = decay_[3].filter(delays_[3].update(i[3], getDelayLength(3)));
        o_[4] = decay_[4].filter(delays_[4].update(i[4], getDelayLength(4)));
        o_[5] = decay_[5].filter(delays_[5].update(i[5], getDelayLength(5)));
        o_[6] = decay_[6].filter(delays_[6].update(i[6], getDelayLength(6)));
        o_[7] = decay_[7].filter(delays_[7].update(i[7], getDelayLength(7)));

        ValueT oo, oe;
        add(oe, o_[0], o_[2]);
        add(oe, oe, o_[4]);
        add(oe, oe, o_[6]);
        add(oo, o_[1], o_[3]);
        add(oo, oo, o_[5]);
        add(oo, oo, o_[7]);

        ValueT r;
        sub(r, oe, oo);
        add(ot_, oe, oo);

        ResultT rs;
        mul(*dst, r, scale_);

        ++dst;
        ++src;
        --nSamples;
    }
}

} // namespace physical_modeling_piano
