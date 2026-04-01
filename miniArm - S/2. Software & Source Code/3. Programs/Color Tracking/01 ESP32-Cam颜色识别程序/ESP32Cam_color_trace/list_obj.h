#ifndef __LIST_OBJ_H
#define __LIST_OBJ_H

#include "mp_type.h"

//////////
// list //
//////////

typedef struct list_lnk
{
    struct list_lnk *next_ptr, *prev_ptr;
    char data[];
}
__attribute__((aligned(8)))list_lnk_t;

typedef struct list
{
    list_lnk_t *head_ptr, *tail_ptr;
    size_t size, data_len;
}
__attribute__((aligned(8)))list_t;

void list_init(list_t *ptr, size_t data_len);
void list_copy(list_t *dst, list_t *src);
void list_free(list_t *ptr);
void list_clear(list_t *ptr);
size_t list_size(list_t *ptr);
void list_push_front(list_t *ptr, void *data);
void list_push_back(list_t *ptr, void *data);
void list_pop_front(list_t *ptr, void *data);
void list_pop_back(list_t *ptr, void *data);
void list_get_front(list_t *ptr, void *data);
void list_get_back(list_t *ptr, void *data);
void list_set_front(list_t *ptr, void *data);
void list_set_back(list_t *ptr, void *data);
void list_insert(list_t *ptr, void *data, size_t index);
void list_remove(list_t *ptr, void *data, size_t index);
void list_get(list_t *ptr, void *data, size_t index);
void list_set(list_t *ptr, void *data, size_t index);
// list end //

#endif //__LIST_OBJ_H

