/*
 * author : Shuichi TAKANO
 * since  : Wed May 08 2019 4:31:2
 */
#ifndef _4C189BD8_C134_1528_201F_B37AFEDA5067
#define _4C189BD8_C134_1528_201F_B37AFEDA5067

#include "delay.h"
#include "filter.h"
#include "fixed.h"
#include "sys_params.h"
#include <vector>

namespace physical_modeling_piano
{

class Soundboard
{
public:
#if USE_FIXED_POINT
    using ValueT         = FixedPoint<int32_t, 25>;
    using FilterHistoryT = FixedPoint<int32_t, 33>;
    using CoefT          = FixedPoint<int32_t, 8>;
    using ResultT        = FixedPoint<int32_t, 15>;
    using ScaleT         = FixedPoint<int32_t, 3>;
#else
    using ValueT         = float;
    using FilterHistoryT = float;
    using CoefT          = float;
    using ResultT        = float;
    using ScaleT         = float;
#endif

public:
    Soundboard() { setScale(10.0f); }

    void initialize(const SystemParameters& sysParams);
    void setScale(float s);

    void update(ResultT* dst, const ValueT* src, size_t nSamples);

private:
    DelayState<ValueT> delays_[8];
    std::vector<ValueT> delayBuffer_;

    ValueT o_[8]{};
    ValueT ot_{};
    CoefT a_{};
    ScaleT scale_{}; // 1/8含む

    Filter<LossFilter<CoefT, FilterHistoryT>> decay_[8];
};

} // namespace physical_modeling_piano

#endif /* _4C189BD8_C134_1528_201F_B37AFEDA5067 */
