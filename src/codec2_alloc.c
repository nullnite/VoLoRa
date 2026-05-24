/* codec2_alloc.c
 *
 * Implements codec2_malloc/calloc/free declared in debug_alloc.h when
 * __EMBEDDED__ is defined. Routes to standard malloc/free backed by
 * CONFIG_HEAP_MEM_POOL_SIZE.
 */

#include <stdlib.h>

void *codec2_malloc(size_t size)  { return malloc(size); }
void *codec2_calloc(size_t n, size_t size) { return calloc(n, size); }
void  codec2_free(void *ptr)      { free(ptr); }
