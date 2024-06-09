#ifndef __FONT_H
#define __FONT_H

#include "stdint.h"

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;

//Font lib.
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

//16-bit(RGB565) Image lib.
/*******************************************
 *             CAUTION:
 *   If the MCU onchip flash cannot
 *  store such huge image data,please
 *           do not use it.
 * These pics are for test purpose only.
 *******************************************/

/* 128x128 pixel RGB565 image */
extern const uint16_t saber[][128];
extern const uint16_t cat_frame0[][14400];
extern const uint16_t cat_frame1[][14400];
//extern const uint16_t frame0[][14400];
//extern const uint16_t frame1[][14400];
//extern const uint16_t frame2[][14400];
//extern const uint16_t frame3[][14400];
//extern const uint16_t frame4[][14400];
//extern const uint16_t frame5[][14400];
//extern const uint16_t frame6[][14400];
//extern const uint16_t frame7[][14400];
//extern const uint16_t frame8[][14400];
//extern const uint16_t frame9[][14400];

/* 240x240 pixel RGB565 image 
extern const uint16_t knky[][240];
extern const uint16_t tek[][240];
extern const uint16_t adi1[][240];
*/
#endif
