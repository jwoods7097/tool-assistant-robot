#ifndef __BITMAP_H
#define __BITMAP_H

////////////
// bitmap //
////////////

#include "mp_type.h"
#include "list_obj.h"

typedef struct bitmap
{
    mp_size_t size;
    char *data;
}
__attribute__((aligned(8)))bitmap_t;

void bitmap_alloc(bitmap_t *ptr, mp_size_t size);
void bitmap_free(bitmap_t *ptr);
void bitmap_clear(bitmap_t *ptr);
void bitmap_bit_set(bitmap_t *ptr, mp_size_t index);
bool bitmap_bit_get(bitmap_t *ptr, mp_size_t index);
#define BITMAP_COMPUTE_ROW_INDEX(image, y) (((image)->w)*(y))
#define BITMAP_COMPUTE_INDEX(row_index, x) ((row_index)+(x))

//////////
// lifo //
//////////

typedef struct lifo
{
    mp_size_t len, size, data_len;
    char *data;
}
__attribute__((aligned(8)))lifo_t;

void lifo_alloc(lifo_t *ptr, mp_size_t size, mp_size_t data_len);
void lifo_alloc_all(lifo_t *ptr, mp_size_t *size, mp_size_t data_len);
void lifo_free(lifo_t *ptr);
void lifo_clear(lifo_t *ptr);
mp_size_t lifo_size(lifo_t *ptr);
bool lifo_is_not_empty(lifo_t *ptr);
bool lifo_is_not_full(lifo_t *ptr);
void lifo_enqueue(lifo_t *ptr, void *data);
void lifo_dequeue(lifo_t *ptr, void *data);
void lifo_poke(lifo_t *ptr, void *data);
void lifo_peek(lifo_t *ptr, void *data);

//////////////
// iterator //
//////////////

list_lnk_t *iterator_start_from_head(list_t *ptr);
list_lnk_t *iterator_start_from_tail(list_t *ptr);
list_lnk_t *iterator_next(list_lnk_t *lnk);
list_lnk_t *iterator_prev(list_lnk_t *lnk);
void iterator_get(list_t *ptr, list_lnk_t *lnk, void *data);
void iterator_set(list_t *ptr, list_lnk_t *lnk, void *data);

#endif //__BITMAP_H