
#include "list_obj.h"

//////////
// list //
//////////

void list_init(list_t *ptr, mp_size_t data_len)
{
    ptr->head_ptr = NULL;
    ptr->tail_ptr = NULL;
    ptr->size = 0;
    ptr->data_len = data_len;
}

void list_copy(list_t *dst, list_t *src)
{
    memcpy(dst, src, sizeof(list_t));
}

void list_free(list_t *ptr)
{
    for (list_lnk_t *i = ptr->head_ptr; i!=NULL; ) {
        list_lnk_t *j = i->next_ptr;
        free(i);
        i = j;
    }
}

void list_clear(list_t *ptr)
{
    list_free(ptr);

    ptr->head_ptr = NULL;
    ptr->tail_ptr = NULL;
    ptr->size = 0;
}

mp_size_t list_size(list_t *ptr)
{
    return ptr->size;
}

void list_push_front(list_t *ptr, void *data)
{
    list_lnk_t *tmp = (list_lnk_t *) malloc(sizeof(list_lnk_t) + ptr->data_len);
    memcpy(tmp->data, data, ptr->data_len);

    if (ptr->size++) {
        tmp->next_ptr = ptr->head_ptr;
        tmp->prev_ptr = NULL;
        ptr->head_ptr->prev_ptr = tmp;
        ptr->head_ptr = tmp;
    } else {
        tmp->next_ptr = NULL;
        tmp->prev_ptr = NULL;
        ptr->head_ptr = tmp;
        ptr->tail_ptr = tmp;
    }
}

void list_push_back(list_t *ptr, void *data)
{
    list_lnk_t *tmp = (list_lnk_t *) malloc(sizeof(list_lnk_t) + ptr->data_len);
    memcpy(tmp->data, data, ptr->data_len);

    if (ptr->size++) {
        tmp->next_ptr = NULL;
        tmp->prev_ptr = ptr->tail_ptr;
        ptr->tail_ptr->next_ptr = tmp;
        ptr->tail_ptr = tmp;
    } else {
        tmp->next_ptr = NULL;
        tmp->prev_ptr = NULL;
        ptr->head_ptr = tmp;
        ptr->tail_ptr = tmp;
    }
}

void list_pop_front(list_t *ptr, void *data)
{
    list_lnk_t *tmp = ptr->head_ptr;

    if (data) {
        memcpy(data, tmp->data, ptr->data_len);
    }

    if (tmp->next_ptr) {
        tmp->next_ptr->prev_ptr = NULL;
    }
    ptr->head_ptr = tmp->next_ptr;
    ptr->size -= 1;
    free(tmp);
}

void list_pop_back(list_t *ptr, void *data)
{
    list_lnk_t *tmp = ptr->tail_ptr;

    if (data) {
        memcpy(data, tmp->data, ptr->data_len);
    }

    tmp->prev_ptr->next_ptr = NULL;
    ptr->tail_ptr = tmp->prev_ptr;
    ptr->size -= 1;
    free(tmp);
}

void list_get_front(list_t *ptr, void *data)
{
    memcpy(data, ptr->head_ptr->data, ptr->data_len);
}

void list_get_back(list_t *ptr, void *data)
{
    memcpy(data, ptr->tail_ptr->data, ptr->data_len);
}

void list_set_front(list_t *ptr, void *data)
{
    memcpy(ptr->head_ptr->data, data, ptr->data_len);
}

void list_set_back(list_t *ptr, void *data)
{
    memcpy(ptr->tail_ptr->data, data, ptr->data_len);
}

void list_insert(list_t *ptr, void *data, mp_size_t index)
{
    if (index == 0) {
        list_push_front(ptr, data);
    } else if (index >= ptr->size) {
        list_push_back(ptr, data);
    } else if (index < (ptr->size >> 1)) {

        list_lnk_t *i = ptr->head_ptr;

        while (index) {
            i = i->next_ptr;
            index -= 1;
        }

        list_lnk_t *tmp = (list_lnk_t *) malloc(sizeof(list_lnk_t) + ptr->data_len);
        memcpy(tmp->data, data, ptr->data_len);

        tmp->next_ptr = i;
        tmp->prev_ptr = i->prev_ptr;
        i->prev_ptr->next_ptr = tmp;
        i->prev_ptr = tmp;
        ptr->size += 1;

    } else {

        list_lnk_t *i = ptr->tail_ptr;
        index = ptr->size - index - 1;

        while (index) {
            i = i->prev_ptr;
            index -= 1;
        }

        list_lnk_t *tmp = (list_lnk_t *) malloc(sizeof(list_lnk_t) + ptr->data_len);
        memcpy(tmp->data, data, ptr->data_len);

        tmp->next_ptr = i;
        tmp->prev_ptr = i->prev_ptr;
        i->prev_ptr->next_ptr = tmp;
        i->prev_ptr = tmp;
        ptr->size += 1;
    }
}

void list_remove(list_t *ptr, void *data, mp_size_t index)
{
    if (index == 0) {
        list_pop_front(ptr, data);
    } else if (index >= (ptr->size - 1)) {
        list_pop_back(ptr, data);
    } else if (index < (ptr->size >> 1)) {

        list_lnk_t *i = ptr->head_ptr;

        while (index) {
            i = i->next_ptr;
            index -= 1;
        }

        if (data) {
            memcpy(data, i->data, ptr->data_len);
        }

        i->prev_ptr->next_ptr = i->next_ptr;
        i->next_ptr->prev_ptr = i->prev_ptr;
        ptr->size -= 1;
        free(i);

    } else {

        list_lnk_t *i = ptr->tail_ptr;
        index = ptr->size - index - 1;

        while (index) {
            i = i->prev_ptr;
            index -= 1;
        }

        if (data) {
            memcpy(data, i->data, ptr->data_len);
        }

        i->prev_ptr->next_ptr = i->next_ptr;
        i->next_ptr->prev_ptr = i->prev_ptr;
        ptr->size -= 1;
        free(i);
    }
}

void list_get(list_t *ptr, void *data, mp_size_t index)
{
    if (index == 0) {
        list_get_front(ptr, data);
    } else if (index >= (ptr->size - 1)) {
        list_get_back(ptr, data);
    } else if (index < (ptr->size >> 1)) {

        list_lnk_t *i = ptr->head_ptr;

        while (index) {
            i = i->next_ptr;
            index -= 1;
        }

        memcpy(data, i->data, ptr->data_len);

    } else {

        list_lnk_t *i = ptr->tail_ptr;
        index = ptr->size - index - 1;

        while (index) {
            i = i->prev_ptr;
            index -= 1;
        }

        memcpy(data, i->data, ptr->data_len);
    }
}

void list_set(list_t *ptr, void *data, mp_size_t index)
{
    if (index == 0) {
        list_set_front(ptr, data);
    } else if (index >= (ptr->size - 1)) {
        list_set_back(ptr, data);
    } else if (index < (ptr->size >> 1)) {

        list_lnk_t *i = ptr->head_ptr;

        while (index) {
            i = i->next_ptr;
            index -= 1;
        }

        memcpy(i->data, data, ptr->data_len);

    } else {

        list_lnk_t *i = ptr->tail_ptr;
        index = ptr->size - index - 1;

        while (index) {
            i = i->prev_ptr;
            index -= 1;
        }

        memcpy(i->data, data, ptr->data_len);
    }
}

