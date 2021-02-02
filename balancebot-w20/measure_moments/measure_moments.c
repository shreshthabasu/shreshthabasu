/*******************************************************************************
* measure_moments.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the moments of inertia of your Balancebot
* 
* TODO: capture the gyro data and timestamps to a file to determine the period.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/mpu.h>
#define READ_HZ 100
#define TB_PITCH_X 0
#define TB_ROLL_Y 1
#define TB_YAW_Z 2
FILE* f1;
rc_mpu_data_t mpu_data;


void print_data(void) {
    fprintf(f1, "%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n", mpu_data.accel[0], mpu_data.accel[1], mpu_data.accel[2], mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2], mpu_data.dmp_TaitBryan[TB_PITCH_X], mpu_data.dmp_TaitBryan[TB_ROLL_Y], mpu_data.dmp_TaitBryan[TB_YAW_Z]);
    printf("%6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\n", mpu_data.accel[0], mpu_data.accel[1], mpu_data.accel[2], mpu_data.gyro[0], mpu_data.gyro[1], mpu_data.gyro[2], mpu_data.dmp_TaitBryan[TB_PITCH_X], mpu_data.dmp_TaitBryan[TB_ROLL_Y], mpu_data.dmp_TaitBryan[TB_YAW_Z]);
    fflush(stdout);
}

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char *argv[]){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
        
    f1 = fopen(argv[1], "w");
    if (f1 == NULL) {
        return 1;
    }
    rc_set_state(RUNNING);
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.dmp_sample_rate = READ_HZ;
    mpu_config.dmp_fetch_accel_gyro = 1;
    rc_mpu_initialize_dmp(&mpu_data, mpu_config);
    rc_mpu_set_dmp_callback(&print_data);
    while(rc_get_state()!=EXITING){
    	rc_nanosleep(1E9 / READ_HZ);
    }

	// exit cleanly
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}
