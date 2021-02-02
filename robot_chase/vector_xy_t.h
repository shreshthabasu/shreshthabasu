#include <stdlib.h>

typedef struct vect {
    double *x;
    double *y;
    int curr_num;
    int capacity;
} vect_t;
void initVector(vect_t *v);
void add(vect_t *v, double x0, double y0);
