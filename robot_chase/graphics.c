#include "image_server.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "graphics.h"

#define M_PI 3.14159265358979323846

vect_i32_t *raster_line(int x0, int y0, int x1, int y1, vect_i32_t *v) {
    initVector_i32(v);
    add_i32(v, x0, y0);
    float dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    float dy = -1 * abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    float err = dx + dy;
    while (1) {
        if (x0 == x1 && y0 == y1) {
            break;
        }
        float e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
        add_i32(v, x0, y0);
    }
    return v;
}

void plot_line(bitmap_t *bmp, vect_i32_t *v, color_bgr_t color) {
    int last = v->curr_num - 1;
    for (int j = 0; j < last; j++) {
        if (v->x[j] >= 0 && v->y[j] >= 0) {
            int index = v->y[j] * bmp->width + v->x[j];
            bmp->data[index] = color;
        }
    }
}

void round_poly(vect_t *poly) {
    double x_min = 640;
    double y_min = 480;
    double epsilon = 1e-6;
    int last = poly->curr_num;
    for (int i = 0; i < last; i++) {
        x_min = poly->x[i] < x_min ? poly->x[i] : x_min;
        y_min = poly->y[i] < y_min ? poly->y[i] : y_min;
    }
    for (int j = 0; j < last; j++) {
        if (poly->x[j] == x_min) {
            poly->x[j] = ceil(poly->x[j]);
        } else {
            poly->x[j] = floor(poly->x[j] - epsilon);
        }
        if (poly->y[j] == y_min) {
            poly->y[j] = ceil(poly->y[j]);
        } else {
            poly->y[j] = floor(poly->y[j] - epsilon);
        }
    }
}

void draw_polygon(bitmap_t *bmp, vect_t *poly, color_bgr_t color) {
    int next = 0;
    int last = poly->curr_num;
    for (int curr = 0; curr < last; curr++) {
        next = curr + 1;
        if (next == last) {
            next = 0;
        }
        vect_i32_t v;
        vect_i32_t *rast_vect_line = raster_line(poly->x[curr], poly->y[curr],
                                                 poly->x[next], poly->y[next], &v);
        plot_line(bmp, rast_vect_line, color);
        free(rast_vect_line->x);
        free(rast_vect_line->y);
    }
}

void make_rectangle(vect_t *rect, double h, double w) {
    initVector(rect);
    add(rect, w, h);
    add(rect, w, 0);
    add(rect, 0, 0);
    add(rect, 0, h);
}

void translate(vect_t *poly, double x, double y) {
    int len = poly->curr_num;
    for (int i = 0; i < len; i++) {
        poly->x[i] += x;
        poly->y[i] += y;
    }
}

void fill_poly(bitmap_t *bmp, vect_t *poly, color_bgr_t color) {
    int x0[bmp->height];
    int x1[bmp->height];
    for (int m = 0; m < bmp->height; m++) {
        x0[m] = bmp->width;
        x1[m] = 0;
    }
    int next = 0;
    int last = poly->curr_num;
    int y_min = bmp->height;
    int y_max = 0;
    for (int curr = 0; curr < last; curr++) {
        next = curr + 1;
        if (next == last) {
            next = 0;
        }
        vect_i32_t v;
        vect_i32_t *rast_vect_line = raster_line(poly->x[curr], poly->y[curr],
                                                 poly->x[next], poly->y[next], &v);
        //plot_line(bmp, rast_vect_line, color);
        for (int y = 0; y < bmp->height; y++) {
            for (int i = 0; i < rast_vect_line->curr_num - 1; i++) {
                if (y == rast_vect_line->y[i]) {
                    y_min = rast_vect_line->y[i] < y_min ? rast_vect_line->y[i] : y_min;
                    y_max = rast_vect_line->y[i] > y_max ? rast_vect_line->y[i] : y_max;
                    x0[y] = rast_vect_line->x[i] < x0[y] ? rast_vect_line->x[i] : x0[y];
                    x1[y] = rast_vect_line->x[i] > x1[y] ? rast_vect_line->x[i] : x1[y];
                }
            }
        }
        free(rast_vect_line->x);
        free(rast_vect_line->y);
    }

    for (int j = y_min; j <= y_max; j++) { //y vals
        for (int k = x0[j]; k <= x1[j]; k++) { //x vals
            int index = j * bmp->width + k;
            bmp->data[index] = color;
        }
    }
}

void rotate(vect_t *poly, double rad_angle) {
    double rad = -rad_angle; //(-1 * degree * M_PI) / 180; was in degrees for rasterize
    int last = poly->curr_num;
    double x_coords[last];
    double y_coords[last];
    for (int j = 0; j < last; j++) {
        x_coords[j] = poly->x[j];
        y_coords[j] = poly->y[j];
    }
    for (int i = 0; i < last; i++) {
        poly->x[i] = x_coords[i] * cos(rad) - y_coords[i] * sin(rad); // change y to pos
        poly->y[i] = x_coords[i] * sin(rad) + y_coords[i] * cos(rad); // changed x to neg
    }
}

void make_triangle(vect_t *poly, double base) {
    double height = (4 * base) / 3;
    initVector(poly);
    add(poly, height / 2, 0);
    add(poly, -height / 2, base / 2);
    add(poly, -height / 2, -base / 2);
}
