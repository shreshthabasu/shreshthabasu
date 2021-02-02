#include <stdlib.h>

typedef struct vect_i32 {
    int *x;
    int *y;
    int curr_num;
    int capacity;
} vect_i32_t;
void initVector_i32(vect_i32_t *v);
void add_i32(vect_i32_t *v, int x0, int y0);
