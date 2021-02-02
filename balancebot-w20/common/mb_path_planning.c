#include <math.h>
#include "mb_path_planning.h"

void init_target_points(mb_odometry_t *target_pts) {
    target_pts[0].x = 0.0;
    target_pts[0].y = 0.0;
    target_pts[0].psi = 0.0;
    target_pts[1].x = 2.5;
    target_pts[1].y = 0.0;
    target_pts[1].psi = 0.0;
    target_pts[2].x = 5.5;
    target_pts[2].y = 0.0;
    target_pts[2].psi = 0.0;
    target_pts[3].x = 8.0;
    target_pts[3].y = 0.00;
    target_pts[3].psi = 0.0;
    target_pts[4].x = 11.20;
    target_pts[4].y = 0.0;
    target_pts[4].psi = 0.0;
}

void getTarget(mb_odometry_t curr_odo, mb_state_t *state) {
    mb_odometry_t target_pts[5];
    init_target_points(target_pts);
    float dist_thresh = 0.15;
    float dist = sqrt(pow(curr_odo.x - state->target_x, 2) + pow(curr_odo.y - state->target_y, 2));
    state->dist_to_target = dist;
    if (dist < dist_thresh) {
        if (state->cur_target_idx != 4) {
            state->cur_target_idx = (state->cur_target_idx + 1 )% 5;
        }
    }
    state->target_x = target_pts[state->cur_target_idx].x;
    state->target_y = target_pts[state->cur_target_idx].y;
}

void getReferences(mb_odometry_t cur, mb_state_t *state) {
    //double tar_x, tar_y;
    //getTarget(cur.x cur.y, &tar_x, &tar_y);
    getTarget(cur, state);
    state->distance_ref += 0.2 * DT;
    state->psi_ref = atan2(state->target_y - cur.y, state->target_x - cur.x);
}
