#ifndef LINEAR_LOOKUP_H
#define LINEAR_LOOKUP_H

#include <stddef.h>

typedef struct {
    const float *keys;
    const float *values;
    size_t size;
} Linear_Lookup_t;

void linear_lookup_init(Linear_Lookup_t *lookup, const float *keys_arr, const float *values_arr, size_t size);
float linear_lookup(const Linear_Lookup_t *lookup, float key);


#endif // LINEAR_LOOKUP_H