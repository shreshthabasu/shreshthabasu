// script by niraj to stop the motors
#include <stdio.h>
#include "common/mb_motor.h"
#include "common/mb_defs.h"

int main() {
	if(rc_kill_existing_process(2.0)<-2) {
		fprintf(stderr, "Couldn't kill existing processes\n")
		return -1;
	}
	rc_make_pid_file();
	rc_set_state(RUNNING);
}