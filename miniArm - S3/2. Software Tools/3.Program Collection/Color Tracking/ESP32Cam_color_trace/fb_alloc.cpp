/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013-2016 Kwabena W. Agyeman <kwagyeman@openmv.io>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Interface for using extra frame buffer RAM as a stack.
 *
 */
#include "fb_alloc.h"
#include "mp_type.h"

#define OMV_FB_ALLOC_SIZE 700 * 1024 // minimum fb alloc size  默认 700 * 1024
#define FB_MAX_ALLOC_TIMES    100

typedef struct 
{
    bool valid;
    uint8_t count;
    uint8_t mark;
    void* p;
}fb_alloc_addr_info_t __attribute__((aligned(8)));

static fb_alloc_addr_info_t m_fb_alloc_addr[FB_MAX_ALLOC_TIMES]; //must <255
static uint8_t m_count_max_now = 0;
static uint8_t m_mark_max_now = 0;


void fb_alloc_fail()
{
  // Serial.println("Out of Memory! Please reduce the resolution of the image you are running this algorithm on to bypass this issue!")
}

void fb_alloc_fail_2()
{
  // Serial.println("Too many fb_alloc! no space save! try again or reduce img size!");
}

void fb_alloc_init_once()
{
    
}

void fb_alloc_init0()
{
    memset(m_fb_alloc_addr, 0, sizeof(m_fb_alloc_addr));
}

// uint64_t fb_avail()
// {
//     mp_size_t size = get_free_heap_size2();
//     return  size > OMV_FB_ALLOC_SIZE ? OMV_FB_ALLOC_SIZE : size;
// }

void fb_alloc_mark()
{
    ++m_mark_max_now;
}

void fb_alloc_free_till_mark()
{
    uint8_t i;

    for(i=0; i<FB_MAX_ALLOC_TIMES; ++i)
    {
        if( m_fb_alloc_addr[i].valid && m_fb_alloc_addr[i].mark==m_mark_max_now)
        {
            free(m_fb_alloc_addr[i].p);
            m_fb_alloc_addr[i].p = NULL;
            m_fb_alloc_addr[i].valid = false;
            --m_count_max_now;
        }
    }
    --m_mark_max_now;
}

void *fb_alloc(uint64_t size) //++
{
    uint8_t i;

    if (!size) {
        return NULL;
    }
    // size=((size+sizeof(uint64_t)-1)/sizeof(uint64_t))*sizeof(uint64_t);// Round Up
    size=((size+32-1)/32)*32;//TODO:
    void* p = malloc(size);
    if(!p)
    {
        // Serial.print("fb alloc fail,errno ");
        // Serial.println(size);
        fb_alloc_fail();
        return NULL;
    }

    for(i=0; i<FB_MAX_ALLOC_TIMES; ++i)
    {
        if( !m_fb_alloc_addr[i].valid )
        {
            m_fb_alloc_addr[i].valid = true;
            m_fb_alloc_addr[i].p = p;
            m_fb_alloc_addr[i].mark = m_mark_max_now;
            ++m_count_max_now;
            m_fb_alloc_addr[i].count = m_count_max_now;
            break;
        }
    }
    if(i == FB_MAX_ALLOC_TIMES)
    {
        free(p);
        fb_alloc_fail_2();
        return NULL;
    }
    return m_fb_alloc_addr[i].p;
}

// returns null pointer without error if passed size==0
void *fb_alloc0(uint64_t size)  //++
{
    void *mem = fb_alloc(size);
    if(mem == NULL)
    {
      return NULL;
    }
    memset(mem, 0, size); // does nothing if size is zero.
    return mem;
}

void *fb_alloc_all(uint64_t *size) //++
{
    uint8_t i =0;

    void* p = malloc(OMV_FB_ALLOC_SIZE);
    if( !p )
    {
      fb_alloc_fail();
      return NULL;
    }
        
    for(i=0; i<FB_MAX_ALLOC_TIMES; ++i)
    {
        if( !m_fb_alloc_addr[i].valid )
        {
            m_fb_alloc_addr[i].valid = true;
            m_fb_alloc_addr[i].p = p;
            m_fb_alloc_addr[i].mark = m_mark_max_now;
            ++m_count_max_now;
            m_fb_alloc_addr[i].count = m_count_max_now;
            break;
        }
    }
    if(i == FB_MAX_ALLOC_TIMES)
    {
        free(p);
        fb_alloc_fail_2();
        return NULL;
    }
    *size = OMV_FB_ALLOC_SIZE;
    return m_fb_alloc_addr[i].p;
}

void fb_free() //++
{
    uint8_t i;
    
    for(i=0; i<FB_MAX_ALLOC_TIMES; ++i)
    {
        if( m_fb_alloc_addr[i].valid && m_fb_alloc_addr[i].count==m_count_max_now)
        {
          if(m_fb_alloc_addr[i].p != NULL)
          {
            // Serial.println("--Free--");
            free(m_fb_alloc_addr[i].p);
            // Serial.println("--Free OK--");
          }
            // Serial.println("++++");
            m_fb_alloc_addr[i].p = NULL;
            m_fb_alloc_addr[i].valid = false;
            --m_count_max_now;
            break;
        }
    }
}

// returns null pointer without error if returned size==0
void *fb_alloc0_all(uint64_t *size)
{
    void *mem = fb_alloc_all(size);
    memset(mem, 0, *size); // does nothing if size is zero.
    return mem;
}



void fb_free_all()
{
    uint8_t i;

    for(i=0; i<FB_MAX_ALLOC_TIMES; ++i)
    {
        if( m_fb_alloc_addr[i].valid )
        {
            free(m_fb_alloc_addr[i].p);
            m_fb_alloc_addr[i].p = NULL;
            m_fb_alloc_addr[i].valid = false;
        }
    }
    m_count_max_now = 0;
    m_mark_max_now  = 0;
}
