CFLAGS = -ggdb3 -std=c11 -Wall -Wunused-parameter -Wstrict-prototypes -Werror -Wextra -Wshadow
CFLAGS += -fsanitize=signed-integer-overflow
CFLAGS += -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable
CFLAGS += -fsanitize=address -fsanitize=undefined

chase: chase.c bmp.c image_server.c vector_xy_i32_t.c vector_xy_t.c graphics.c
	gcc -o $@ $^ $(CFLAGS) -g -pthread -lm -O3
