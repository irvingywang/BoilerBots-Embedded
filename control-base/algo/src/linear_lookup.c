#include "linear_lookup.h"

void linear_lookup_init(Linear_Lookup_t *lookup, const float *keys, const float *values, size_t size) {
    lookup->keys = keys;
    lookup->values = values;
    lookup->size = size;
}

float linear_lookup(const Linear_Lookup_t *lookup, float key) {
    if (lookup->size == 0) {
        return 0; // No data available
    }

    if (key <= lookup->keys[0]) { // bounds check for the first key
        return lookup->values[0];
    }

    if (key >= lookup->keys[lookup->size - 1]) { // bounds check for the last key
        return lookup->values[lookup->size - 1];
    }


    // TODO linear lookup

    return 0;
}

