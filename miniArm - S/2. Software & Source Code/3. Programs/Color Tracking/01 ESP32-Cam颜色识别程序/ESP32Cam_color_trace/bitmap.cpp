
#include "bitmap.h"
#include "fb_alloc.h"
////////////
// bitmap //
////////////

#define IM_LOG2_2(x)    (((x) &                0x2ULL) ? ( 2                        ) :             1) // NO ({ ... }) !
#define IM_LOG2_4(x)    (((x) &                0xCULL) ? ( 2 +  IM_LOG2_2((x) >>  2)) :  IM_LOG2_2(x)) // NO ({ ... }) !
#define IM_LOG2_8(x)    (((x) &               0xF0ULL) ? ( 4 +  IM_LOG2_4((x) >>  4)) :  IM_LOG2_4(x)) // NO ({ ... }) !
#define IM_LOG2_16(x)   (((x) &             0xFF00ULL) ? ( 8 +  IM_LOG2_8((x) >>  8)) :  IM_LOG2_8(x)) // NO ({ ... }) !
#define IM_LOG2_32(x)   (((x) &         0xFFFF0000ULL) ? (16 + IM_LOG2_16((x) >> 16)) : IM_LOG2_16(x)) // NO ({ ... }) !
#define IM_LOG2(x)      (((x) & 0xFFFFFFFF00000000ULL) ? (32 + IM_LOG2_32((x) >> 32)) : IM_LOG2_32(x)) // NO ({ ... }) !

#define CHAR_BITS (sizeof(char) * 8)
#define CHAR_MASK (CHAR_BITS - 1)
#define CHAR_SHIFT IM_LOG2(CHAR_MASK)

void bitmap_alloc(bitmap_t *ptr, mp_size_t size)
{
    ptr->size = size;
    ptr->data = (char *) fb_alloc0(((size + CHAR_MASK) >> CHAR_SHIFT) * sizeof(char));
}

void bitmap_free(bitmap_t *ptr)
{
    if (ptr->data) {
        fb_free();
    }
}

void bitmap_clear(bitmap_t *ptr)
{
    memset(ptr->data, 0, ((ptr->size + CHAR_MASK) >> CHAR_SHIFT) * sizeof(char));
}

void bitmap_bit_set(bitmap_t *ptr, mp_size_t index)
{
    ptr->data[index >> CHAR_SHIFT] |= 1 << (index & CHAR_MASK);
}

bool bitmap_bit_get(bitmap_t *ptr, mp_size_t index)
{
    return (ptr->data[index >> CHAR_SHIFT] >> (index & CHAR_MASK)) & 1;
}

//////////
// lifo //
//////////

void lifo_alloc(lifo_t *ptr, mp_size_t size, mp_size_t data_len)
{
    ptr->len = 0;
    ptr->size = size;
    ptr->data_len = data_len;
    ptr->data = (char *) fb_alloc(size * data_len);
}

void lifo_alloc_all(lifo_t *ptr, mp_size_t *size, mp_size_t data_len)
{
    uint64_t tmp_size;
    ptr->data = (char *) fb_alloc_all(&tmp_size);
    ptr->data_len = data_len;
    ptr->size = tmp_size / data_len;
    ptr->len = 0;
    *size = ptr->size;
}

void lifo_free(lifo_t *ptr)
{
    if (ptr->data) {
        fb_free();
    }
}

void lifo_clear(lifo_t *ptr)
{
    ptr->len = 0;
}

mp_size_t lifo_size(lifo_t *ptr)
{
    return ptr->len;
}

bool lifo_is_not_empty(lifo_t *ptr)
{
    return ptr->len;
}

bool lifo_is_not_full(lifo_t *ptr)
{
    return ptr->len != ptr->size;
}

void lifo_enqueue(lifo_t *ptr, void *data)
{
    memcpy(ptr->data + (ptr->len * ptr->data_len), data, ptr->data_len);

    ptr->len += 1;
}

void lifo_dequeue(lifo_t *ptr, void *data)
{
    if (data) {
        memcpy(data, ptr->data + ((ptr->len - 1) * ptr->data_len), ptr->data_len);
    }

    ptr->len -= 1;
}

void lifo_poke(lifo_t *ptr, void *data)
{
    memcpy(ptr->data + (ptr->len * ptr->data_len), data, ptr->data_len);
}

void lifo_peek(lifo_t *ptr, void *data)
{
    memcpy(data, ptr->data + ((ptr->len - 1) * ptr->data_len), ptr->data_len);
}

//////////////
// iterator //
//////////////

list_lnk_t *iterator_start_from_head(list_t *ptr)
{
    return ptr->head_ptr;
}

list_lnk_t *iterator_start_from_tail(list_t *ptr)
{
    return ptr->tail_ptr;
}

list_lnk_t *iterator_next(list_lnk_t *lnk)
{
    return lnk->next_ptr;
}

list_lnk_t *iterator_prev(list_lnk_t *lnk)
{
    return lnk->prev_ptr;
}

void iterator_get(list_t *ptr, list_lnk_t *lnk, void *data)
{
    memcpy(data, lnk->data, ptr->data_len);
}

void iterator_set(list_t *ptr, list_lnk_t *lnk, void *data)
{
    memcpy(lnk->data, data, ptr->data_len);
}
