#ifndef FLASHC_H
#define FLASHC_H

static const char flashc_user_page[512];

#define AVR32_FLASHC_USER_PAGE_ADDRESS &(flashc_user_page)

static void inline flashc_memcpy(void *src, void* dst, int32_t size ,  bool write) {
}

#endif