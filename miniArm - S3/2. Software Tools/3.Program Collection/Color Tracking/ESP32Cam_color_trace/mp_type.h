#ifndef __MP_TYPE_H
#define __MP_TYPE_H

#include <Arduino.h>

typedef unsigned int mp_size_t;

//返回值元素
typedef struct py_blob_obj {
    int x, y, w, h, pixels, cx, cy, code, count;
    float rotation;
} py_blob_obj_t;

//返回值队列
typedef struct _mp_obj_list_t{
  mp_size_t alloc;
  mp_size_t len;
  py_blob_obj_t *items;
}mp_obj_list_t;

// 图像
typedef struct image {
    int w;
    int h;
    int bpp;
    union {
        uint8_t *pixels;
        uint8_t *data;
    };
	uint8_t *pix_ai;	//for MAIX AI speed up
} __attribute__((aligned(8)))image_t;

typedef struct point {
    int16_t x;
    int16_t y;
} __attribute__((aligned(8))) point_t;

typedef struct rectangle {
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
} __attribute__((aligned(8))) rectangle_t;

typedef struct find_blobs_list_lnk_data {
    rectangle_t rect;
    uint32_t pixels;
    point_t centroid;
    float rotation;
    uint16_t code, count;
} __attribute__((aligned(8))) find_blobs_list_lnk_data_t;






#endif //__MP_TYPE_H


