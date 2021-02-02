#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include "graphics.h"
#include "image_server.h"
#include <unistd.h>
#include <float.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#define WIDTH 640
#define HEIGHT 480

#define BLOCK_SIZE 40
#define MAP "XXXXXXXXXXXXXXXX" \
            "X              X" \
            "X  XXXX   XXX  X" \
            "X   XX      X  X" \
            "X       XXX    X" \
            "XXXXXX         X" \
            "X         XXXXXX" \
            "X    XXX       X" \
            "X  XX     XX   X" \
            "X   X    XX    X" \
            "X      XXX     X" \
            "XXXXXXXXXXXXXXXX"
#define MAP_W (WIDTH / BLOCK_SIZE)
#define MAP_H (HEIGHT / BLOCK_SIZE)
#define M_PI 3.14159265358979323846

typedef struct agent {
    double x;
    double y;
    double rot;
    double vel;
    double ang_vel;
} agent_t;

typedef struct search_node {
    int depth;
    agent_t runner_st;
    agent_t chaser_st;
} search_node_t;

void display(size_t bmp_size, bitmap_t *bmp) {
    uint8_t *serialized_bmp = malloc(bmp_size);
    bmp_serialize(bmp, serialized_bmp);
    image_server_set_data(bmp_size, serialized_bmp);
    image_server_start("8000");
    //sleep(1);
    int seconds = 0;
    long nanoseconds = 40 * 1000 * 1000;
    struct timespec interval = {seconds, nanoseconds};
    nanosleep(&interval, NULL);
    free(serialized_bmp);
}

void draw_map(bitmap_t *bmp) {
    color_bgr_t white = {255, 255, 255};
    int map_len = strlen(MAP);
    for (int i = 0; i < map_len; i++) {
        if (MAP[i] == ' ') {
            vect_t sq;
            make_rectangle(&sq, BLOCK_SIZE, BLOCK_SIZE);
            int trans_y = (i / MAP_W) * BLOCK_SIZE;
            int trans_x = (i % MAP_W) * BLOCK_SIZE;
            translate(&sq, (double)trans_x, (double)trans_y);
            round_poly(&sq);
            fill_poly(bmp, &sq, white);
            free(sq.x);
            free(sq.y);
        }
    }
}

void update_graphics(agent_t *chaser_st, agent_t *runner_st) {
    double base = 20;
    color_bgr_t red = {0, 0, 255};
    color_bgr_t green = {0, 255, 0};
    bitmap_t bmp = {0};
    bmp.height = 480;
    bmp.width = 640;
    bmp.data = calloc(bmp.width * bmp.height, sizeof(color_bgr_t));
    size_t bmp_size = bmp_calculate_size(&bmp);
    draw_map(&bmp);

    //chaser
    vect_t chaser;
    make_triangle(&chaser, base);
    rotate(&chaser, chaser_st->rot);
    translate(&chaser, chaser_st->x, chaser_st->y);
    round_poly(&chaser);
    fill_poly(&bmp, &chaser, red);

    // runner
    vect_t runner;
    make_triangle(&runner, base);
    rotate(&runner, runner_st->rot);
    translate(&runner, runner_st->x, runner_st->y);
    round_poly(&runner);
    fill_poly(&bmp, &runner, green);

    //display
    display(bmp_size, &bmp);
    free(bmp.data);
}

int gen_num(void) {
    int num = rand() % 20;
    if (num == 1) {
        return num;
    }
    if (num == 2) {
        return num;
    }
    return 0;
}

void get_obstacles(vect_t *obstacle, agent_t *robot_st) {
    int r_map_x = (int)robot_st->x / BLOCK_SIZE;
    int r_map_y = (int)robot_st->y / BLOCK_SIZE;
    for (int y = -1; y <= 1; y++) {
        if (r_map_y + y < 0 || r_map_y + y > MAP_H - 1) {
            continue;
        }
        for (int x = -1; x <= 1; x++) {
            if (r_map_x + x < 0 || r_map_x + x > MAP_W - 1) {
                continue;
            }
            int index = (r_map_y + y) * MAP_W + (r_map_x + x);
            if (MAP[index] == 'X') {
                int trans_y = (index / MAP_W) * BLOCK_SIZE;
                int trans_x = (index % MAP_W) * BLOCK_SIZE;
                add(obstacle, trans_x, trans_y);
            }
        }
    }
}

bool check_collision(vect_t poly1, vect_t poly2) {
    bool collision = 0;
    int next_p1 = 0;
    for (int curr_p1 = 0; curr_p1 < poly1.curr_num; curr_p1++) {
        next_p1 = curr_p1 + 1;
        if (next_p1 == poly1.curr_num) {
            next_p1 = 0;
        }
        int next_p2 = 0;
        for (int curr_p2 = 0; curr_p2 < poly2.curr_num; curr_p2++) {
            next_p2 = curr_p2 + 1;
            if (next_p2 == poly2.curr_num) {
                next_p2 = 0;
            }
            double x1 = poly1.x[curr_p1];
            double y1 = poly1.y[curr_p1];
            double x2 = poly1.x[next_p1];
            double y2 = poly1.y[next_p1];
            double x3 = poly2.x[curr_p2];
            double y3 = poly2.y[curr_p2];
            double x4 = poly2.x[next_p2];
            double y4 = poly2.y[next_p2];
            double cp1 = ((x1 - x3) * (y4 - y3) - (y1 - y3) * (x4 - x3));
            double cp2 = ((x2 - x3) * (y4 - y3) - (y2 - y3) * (x4 - x3));
            double cp3 = ((x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1));
            double cp4 = ((x4 - x1) * (y2 - y1) - (y4 - y1) * (x2 - x1));
            if (cp1 * cp2 < 0 && cp3 * cp4 < 0) {
                collision = 1;
                return collision;
            }
            if (cp1 * cp2 == 0 || cp3 * cp4 == 0) {
                double max_x;
                double min_x;
                double max_y;
                double min_y;
                if (x2 >= x1) {
                    max_x = x2;
                    min_x = x1;
                } else {
                    max_x = x1;
                    min_x = x2;
                }
                if (y2 >= y1) {
                    max_y = y2;
                    min_y = y1;
                } else {
                    max_y = y1;
                    min_y = y2;
                }
                if ((x3 >= min_x && x3 <= max_x) || (x4 >= min_x && x4 <= max_x)) {
                    if ((y3 >= min_y && y3 <= max_y) || (y4 >= min_y && y4 <= max_y)) {
                        collision = 1;
                        return collision;
                    }
                }
            }
        }
    }
    return collision;
}

void resolve_wall_collisions(agent_t *robot_st) {
    double base = 20;
    vect_t obstacle;
    initVector(&obstacle);
    vect_t rob;
    make_triangle(&rob, base);
    rotate(&rob, robot_st->rot);
    translate(&rob, robot_st->x, robot_st->y);
    get_obstacles(&obstacle, robot_st);
    int collision_count = 100;
    bool dec_vel = false;
    while (collision_count > 0) {
        collision_count = 0;
        for (int i = 0; i < obstacle.curr_num; i++) {
            vect_t wall;
            make_rectangle(&wall, BLOCK_SIZE, BLOCK_SIZE);
            translate(&wall, obstacle.x[i], obstacle.y[i]);
            double wall_r = (pow(wall.x[0] - wall.x[2], 2) + pow(wall.y[0] - wall.y[2], 2)) / 2;
            double robot_r = (pow(rob.x[0] - rob.x[1], 2) + pow(rob.y[0] - rob.y[1], 2)) / 2;
            double collision_dist_sq = (wall_r + robot_r) * (wall_r + robot_r);
            double dx = obstacle.x[i] + (BLOCK_SIZE / 2) - robot_st->x;
            double dy = obstacle.y[i] + (BLOCK_SIZE / 2) - robot_st->y;
            double dist_sq = pow(dx, 2) + pow(dy, 2);
            if (dist_sq <= collision_dist_sq) {
                bool is_collision = check_collision(wall, rob);
                if (is_collision) {
                    collision_count += 1;
                    double c_dist = sqrt(dist_sq);
                    double del_x = -0.5 * dx / c_dist;
                    double del_y = -0.5 * dy / c_dist;
                    robot_st->x += del_x;
                    robot_st->y += del_y;
                    if (!dec_vel) {
                        robot_st->vel *= 0.25;
                        dec_vel = true;
                    }
                    translate(&rob, del_x, del_y);
                }
            }
            free(wall.x);
            free(wall.y);
        }
    }
    free(obstacle.x);
    free(obstacle.y);
    free(rob.x);
    free(rob.y);
}

void do_runner_action(agent_t *robot_st, int action) {
    if (action == 1) {
        robot_st->vel = robot_st->vel + 2 > 12.0 ? 12.0 : robot_st->vel + 2;
    } else if (action == 2) {
        robot_st->ang_vel += M_PI / 16.0;
    }
}

void do_chaser_action(agent_t *robot_st, int action) {
    if (action == 1) {
        robot_st->vel = robot_st->vel + 2 > 12.0 ? 12.0 : robot_st->vel + 2;
    } else if (action == 2) {
        robot_st->ang_vel += M_PI / 16.0;
    } else if (action == 3) {
        robot_st->ang_vel -= M_PI / 16.0;
    }
}

void update_movement(agent_t *robot_st) {
    robot_st->rot += robot_st->ang_vel;
    robot_st->ang_vel *= 0.8;
    robot_st->x += robot_st->vel * cos(robot_st->rot);
    robot_st->y += robot_st->vel * -sin(robot_st->rot);
    resolve_wall_collisions(robot_st);
}

double search_actions(search_node_t node, int *chosen_action, vect_t runner) {
    double base = 20;
    int max_depth = 4;
    vect_t chaser;
    make_triangle(&chaser, base);
    rotate(&chaser, node.chaser_st.rot);
    translate(&chaser, node.chaser_st.x, node.chaser_st.y);
    bool is_collision = check_collision(runner, chaser);
    free(chaser.x);
    free(chaser.y);
    if (is_collision) {
        return 0.0;
    }
    if (node.depth >= max_depth) {
        double min_x = node.runner_st.x - node.chaser_st.x;
        double min_y = node.runner_st.y - node.chaser_st.y;
        double min_dist = sqrt(pow(min_x, 2) + pow(min_y, 2));
        return min_dist;
    }
    double score[4] = {0, 0, 0, 0};
    for (int action = 0; action < 4; action++) {
        search_node_t new_node = {0};
        new_node.depth = node.depth;
        new_node.runner_st = node.runner_st;
        new_node.chaser_st = node.chaser_st;
        do_chaser_action(&new_node.chaser_st, action);
        update_movement(&new_node.chaser_st);
        for (int i = 0; i < 3; i++) {
            vect_t new_chaser;
            make_triangle(&new_chaser, base);
            rotate(&new_chaser, node.chaser_st.rot);
            translate(&new_chaser, node.chaser_st.x, node.chaser_st.y);
            bool is_collision_new = check_collision(runner, new_chaser);
            if (!is_collision_new) {
                update_movement(&new_node.chaser_st);
            }
            free(new_chaser.x);
            free(new_chaser.y);
        }
        new_node.depth += 1;
        score[action] = search_actions(new_node, chosen_action, runner);
        score[action] += 300 / fmin(2, new_node.chaser_st.vel);
    }
    double min_score = DBL_MAX;
    int action_index = 5;
    for (int k = 0; k < 4; k++) {
        if (score[k] <= min_score) {
            min_score = score[k];
            action_index = k;
        }
    }
    *chosen_action = action_index;
    return min_score;
}

int get_chaser_action(agent_t chaser_st, agent_t runner_st) {
    double base = 20;
    vect_t runner;
    make_triangle(&runner, base);
    rotate(&runner, runner_st.rot);
    translate(&runner, runner_st.x, runner_st.y);
    search_node_t node = {0};
    node.depth = 0;
    node.chaser_st = chaser_st;
    node.runner_st = runner_st;
    int chosen_action = 0;
    double score = search_actions(node, &chosen_action, runner);
    free(runner.x);
    free(runner.y);
    return chosen_action;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "usage: %s <time steps> <fast=0|1|2> <initial runner index>\n", argv[0]);
        return 1;
    }
    int init_runner_id = atoi(argv[3]);
    int fast = atoi(argv[2]);
    int time_stamp = atoi(argv[1]);
    agent_t chaser_st = {0};
    chaser_st.x = WIDTH / 2;
    chaser_st.y = HEIGHT / 2;
    agent_t runner_st = {0};
    int map_idx_x = init_runner_id % MAP_W;
    int map_idx_y = init_runner_id / MAP_W;
    runner_st.x = (map_idx_x + 0.5) * BLOCK_SIZE;
    runner_st.y = (map_idx_y + 0.5) * BLOCK_SIZE;
    for (int t = 0; t < time_stamp; t++) {
        if (fast == 0) {
            update_graphics(&chaser_st, &runner_st);
        }
        int runner_action = gen_num();
        int chaser_action = get_chaser_action(chaser_st, runner_st);
        printf("%d %d\n", runner_action, chaser_action);
        do_runner_action(&runner_st, runner_action);
        do_chaser_action(&chaser_st, chaser_action);
        update_movement(&runner_st);
        update_movement(&chaser_st);
        double base = 20;
        vect_t chaser;
        vect_t runner;
        make_triangle(&chaser, base);
        rotate(&chaser, chaser_st.rot);
        translate(&chaser, chaser_st.x, chaser_st.y);
        make_triangle(&runner, base);
        rotate(&runner, runner_st.rot);
        translate(&runner, runner_st.x, runner_st.y);
        bool is_collision = check_collision(chaser, runner);
        free(chaser.x);
        free(chaser.y);
        free(runner.x);
        free(runner.y);
        if (is_collision) {
            break;
        }
    }
    if (fast != 2) {
        update_graphics(&chaser_st, &runner_st);
    }
}
