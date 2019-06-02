/*
 * author : Shuichi TAKANO
 * since  : Sun Mar 24 2019 2:15:25
 */
#ifndef _69B00176_C134_145C_20D9_3E539E19E5A2
#define _69B00176_C134_145C_20D9_3E539E19E5A2

#include <stdint.h>

#define DEF_LINKED_BINARY(x)                                                   \
    extern const uint8_t x##_start[] asm("_binary_" #x "_start");              \
    extern const uint8_t x##_end[] asm("_binary_" #x "_end");

#define GET_LINKED_BINARY(x) x##_start
#define GET_LINKED_BINARY_SIZE(x) (x##_end - x##start)

#define GET_LINKED_BINARY_T(t, x) reinterpret_cast<const t*>(x##_start)

#endif /* _69B00176_C134_145C_20D9_3E539E19E5A2 */
