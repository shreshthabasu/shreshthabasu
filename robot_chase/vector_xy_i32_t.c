#include "vector_xy_i32_t.h"

void initVector_i32(vect_i32_t *v) {
    v->capacity = 1;
    v->curr_num = 0;
    v->x = malloc(sizeof(int) * v->capacity);
    v->y = malloc(sizeof(int) * v->capacity);
    //return v;
}

void add_i32(vect_i32_t *v, int x0, int y0) {
    if (v->capacity == v->curr_num) {
        v->capacity *= 2;
        v->x = realloc(v->x, sizeof(int) * v->capacity);
        v->y = realloc(v->y, sizeof(int) * v->capacity);
    }
    v->x[v->curr_num] = x0;
    v->y[v->curr_num] = y0;
    v->curr_num += 1;
}
