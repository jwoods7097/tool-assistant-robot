
#include "list_obj.h"
#include "bitmap.h"
#include "fb_alloc.h"
#include "imlib.h"

#define COLOR_L_MIN 0
#define COLOR_L_MAX 100
#define COLOR_A_MIN -128
#define COLOR_A_MAX 127
#define COLOR_B_MIN -128
#define COLOR_B_MAX 127

#define COLOR_GRAYSCALE_MIN 0
#define COLOR_GRAYSCALE_MAX 255

/////////////////
// Color Stuff //
/////////////////
typedef struct color_thresholds_list_lnk_data
{
    uint8_t LMin, LMax; // or grayscale
    int8_t AMin, AMax;
    int8_t BMin, BMax;
}__attribute__((aligned(8)))
color_thresholds_list_lnk_data_t;
// Color Stuff end //

void imlib_find_blobs(list_t *out, image_t *ptr, rectangle_t *roi, unsigned int x_stride, unsigned int y_stride,
                     list_t *thresholds, bool invert, unsigned int area_threshold, unsigned int pixels_threshold,
                     bool merge, int margin,
                     bool (*threshold_cb)(void*,find_blobs_list_lnk_data_t*), void *threshold_cb_arg,
                     bool (*merge_cb)(void*,find_blobs_list_lnk_data_t*,find_blobs_list_lnk_data_t*), void *merge_cb_arg);





