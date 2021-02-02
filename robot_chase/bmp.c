#include "bmp.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
// calculate the number of bytes of memory needed to serialize the bitmap
// that is, to write a valid bmp file to memory
size_t bmp_calculate_size(bitmap_t *bmp) {
    size_t size = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) +
                  (bmp->width * bmp->height * sizeof(color_bgr_t));
    return size;
}

// write the bmp file to memory at data, which must be at least
// bmp_calculate_size large.
void bmp_serialize(bitmap_t *bmp, uint8_t *data) {
    BITMAPFILEHEADER file_header = {0}; // start out as all zero values
    file_header.bfType = 0x4D42;
    file_header.bfSize = bmp_calculate_size(bmp);
    file_header.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

    BITMAPINFOHEADER info_header = {0};
    info_header.biSize = 40;
    info_header.biWidth = bmp->width;
    info_header.biHeight = bmp->height;
    info_header.biBitCount = 24;
    info_header.biPlanes = 1;
    info_header.biXPelsPerMeter = 2835;
    info_header.biYPelsPerMeter = 2835;

    uint8_t *data_out = data;
    memcpy(data_out, &file_header, sizeof(file_header));
    data_out += sizeof(file_header);
    memcpy(data_out, &info_header, sizeof(info_header));
    data_out += sizeof(info_header);
    for (int i = 0; i < bmp->height; i++) {
        int index = (bmp->height - i - 1) * bmp->width;
        memcpy(data_out, &bmp->data[index], sizeof(color_bgr_t) * bmp->width);
        data_out += sizeof(color_bgr_t) * bmp->width;
    }
}
