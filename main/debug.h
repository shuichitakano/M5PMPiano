/*
 * author : Shuichi TAKANO
 * since  : Tue Oct 30 2018 6:35:28
 */
#ifndef _4F7C9512_7133_F00A_60F0_F1CD3C763BFD
#define _4F7C9512_7133_F00A_60F0_F1CD3C763BFD

#include <stdio.h>

#ifdef NDEBUG
#define DBOUT(x) \
    do           \
    {            \
    } while (0)
#else
#define DBOUT(x) printf x
#endif

#endif /* _4F7C9512_7133_F00A_60F0_F1CD3C763BFD */
