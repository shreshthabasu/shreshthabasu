#include "vector_xy_i32_t.h"
#include "vector_xy_t.h"
#include "bmp.h"

vect_i32_t *raster_line(int x0, int y0, int x1, int y1, vect_i32_t *v);
void plot_line(bitmap_t *bmp, vect_i32_t *v, color_bgr_t color);
void round_poly(vect_t *poly);
void draw_polygon(bitmap_t *bmp, vect_t *poly, color_bgr_t color);
void make_rectangle(vect_t *rect, double h, double w);
void translate(vect_t *poly, double x, double y);
void fill_poly(bitmap_t *bmp, vect_t *poly, color_bgr_t color);
void rotate(vect_t *poly, double rad_angle);
void create_robot(vect_t *poly, bitmap_t *bmp, double base, color_bgr_t color);
void create_rot_robot(vect_t *poly, bitmap_t *bmp, double base, double angle, color_bgr_t color);
void make_triangle(vect_t *poly, double base);
