#ifndef __IMLIB_H
#define __IMLIB_H

#include <stdint.h>
#include "mp_type.h"

#define BITMAP_COMPUTE_ROW_INDEX(image, y) (((image)->w)*(y))
#define BITMAP_COMPUTE_INDEX(row_index, x) ((row_index)+(x))

extern const int8_t lab_table[196608];

#define COLOR_RGB565_TO_L(pixel) lab_table[(pixel) * 3]
#define COLOR_RGB565_TO_A(pixel) lab_table[((pixel) * 3) + 1]
#define COLOR_RGB565_TO_B(pixel) lab_table[((pixel) * 3) + 2]

#define IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(image, y) \
({ \
    __typeof__ (image) _image = (image); \
    __typeof__ (y) _y = (y); \
    ((uint16_t *) _image->data) + (_image->w * _y); \
})

#define COLOR_THRESHOLD_RGB565(pixel, threshold, invert) \
({ \
    __typeof__ (pixel) _pixel = (pixel); \
    __typeof__ (threshold) _threshold = (threshold); \
    __typeof__ (invert) _invert = (invert); \
    uint8_t _l = COLOR_RGB565_TO_L(_pixel); \
    int8_t _a = COLOR_RGB565_TO_A(_pixel); \
    int8_t _b = COLOR_RGB565_TO_B(_pixel); \
    ((_threshold->LMin <= _l) && (_l <= _threshold->LMax) && \
    (_threshold->AMin <= _a) && (_a <= _threshold->AMax) && \
    (_threshold->BMin <= _b) && (_b <= _threshold->BMax)) ^ _invert; \
})

#define IMAGE_GET_RGB565_PIXEL_FAST(row_ptr, x) \
({ \
    __typeof__ (row_ptr) _row_ptr = (row_ptr); \
    __typeof__ (x) _x = (x); \
    _row_ptr[_x]; \
})


#define IM_MAX(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define IM_MIN(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

/////////////////////
// Rectangle Stuff //
/////////////////////

bool rectangle_overlap(rectangle_t *ptr0, rectangle_t *ptr1);
void rectangle_united(rectangle_t *dst, rectangle_t *src);

#endif //__IMLIB_H
