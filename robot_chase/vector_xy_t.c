#include "vector_xy_t.h"

void initVector(vect_t *v) {
    v->curr_num = 0;
    v->capacity = 1;
    v->x = malloc(sizeof(double) * v->capacity);
    v->y = malloc(sizeof(double) * v->capacity);
}

void add(vect_t *v, double x0, double y0) {
    if (v->curr_num == v->capacity) {
        v->capacity *= 2;
        v->x = realloc(v->x, sizeof(double) * v->capacity);
        v->y = realloc(v->y, sizeof(double) * v->capacity);
    }
    v->x[v->curr_num] = x0;
    v->y[v->curr_num] = y0;
    v->curr_num += 1;
}
