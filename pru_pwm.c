#include <stdio.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <time.h>
#include <sys/mman.h>
#include <fcntl.h>

#define PRU_NUM 1

#define PRU_ADDR 0x4A300000

#define SHAREDRAM_OFFSET 0x00010000

// Counts per revolution, rising and falling edge.
#define CPR 576

struct pid_data {
    /* PID tunings */
    float kp, ki, kd;

    /* PID controls */
    float setpoint;
    float e;
    float input, output, last_output;
    float min_output, max_output;
};

struct pwm {
    FILE *pwm,
         *duty,
         *period,
         *run;
};

void update_pid(volatile struct pid_data*);

void set_pwm(struct pwm*);
void close_pwm(struct pwm*);
void update_pwm_period(struct pwm*, int);
void update_pwm_duty(struct pwm*, int);

int main(void) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    ulong* imp = (ulong*) mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR+SHAREDRAM_OFFSET);

    unsigned int impulses = 0;
    unsigned int status;

    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    prussdrv_init();
    printf("prussdrv initialized\n");

    status = prussdrv_open(PRU_EVTOUT_0);
    if (status) {
        printf("prussdrv_open open failed!\n");
        return (status);
    }

    prussdrv_pruintc_init(&pruss_intc_initdata);
    printf("Interrupt initialized\n");

    status = prussdrv_exec_program(PRU_NUM, "./pru_pwm.bin");
    if (status) {
        printf("prussdrv_exec_program failed!\n");
        return (status);
    }

    // Init PID.
    struct pid_data motor1PID;
    motor1PID.kp = 1;
    motor1PID.ki = 1/8;
    motor1PID.kd = 0.125;

    motor1PID.min_output = -100;
    motor1PID.max_output = 100;

    motor1PID.setpoint = -1;

    // Init PWM.
    struct pwm pwmData;
    set_pwm(&pwmData);
    update_pwm_period(&pwmData, 50000);
    update_pwm_duty(&pwmData, 50000 - 10000);

    unsigned int now_ms, last_ms, delta_ms;
    float current_pos, last_pos, delta_pos;
    struct timespec now;

    FILE* inA1 = fopen("/sys/class/gpio/gpio47/direction", "w");
    FILE* inB1 = fopen("/sys/class/gpio/gpio27/direction", "w");
    int inA1v = 1;
    int writeStatus = 0;

    fseek(inA1, 0, SEEK_SET);
    fprintf(inA1, "%s", "low");
    writeStatus = fflush(inA1);
    printf("Set inA1: %i\n", writeStatus);

    fseek(inB1, 0, SEEK_SET);
    fprintf(inB1, "%s", "high");
    writeStatus = fflush(inB1);
    printf("Set inB1: %i\n", writeStatus);

    while (1) {
//        prussdrv_pru_wait_event(PRU_EVTOUT_0);
//        prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
//        printf("Imp: %i\n", (int) imp[0]);
        clock_gettime(CLOCK_REALTIME, &now);
        impulses = imp[0];
//        now_ms = (now.tv_sec * 1000 + (now.tv_nsec + 500000) / 100000); //convert to milliseconds
//        delta_ms = now_ms - last_ms;

        current_pos = (float) impulses / 576;

	motor1PID.input = current_pos;
        update_pid(&motor1PID);
        update_pwm_duty(&pwmData, 50000 - (motor1PID.output*500));

        if (motor1PID.output < 0 && inA1v != 1) {
            inA1v = 1;

            fseek(inA1, 0, SEEK_SET);
            fprintf(inA1, "%s", "high");
            writeStatus = fflush(inA1);

            fseek(inB1, 0, SEEK_SET);
            fprintf(inB1, "%s", "low");
            writeStatus = fflush(inB1);
        } else if (motor1PID.output > 0 && inA1v != 0) {
            inA1v = 0;
            fseek(inA1, 0, SEEK_SET);
            fprintf(inA1, "%s", "low");
            writeStatus = fflush(inA1);

            fseek(inB1, 0, SEEK_SET);
            fprintf(inB1, "%s", "high");
            //writeStatus = fflush(inB1);
        }

        last_pos = current_pos;
        last_ms = now_ms;
        printf("%i, %f, %i\n", impulses, motor1PID.output, inA1);
        //if (impulses > CPR) {
          //  break;
        //}

    }

    update_pwm_duty(&pwmData, 50000);
    close_pwm(&pwmData);
    printf("Finished!\n");
    prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

    return 0;
}

/**
 * Updates pwm.
 *
 * @param pid
 */
void update_pid(volatile struct pid_data* pid) {
    float p_f, d_f;
    float output_f, output;

    /* Calculate error */
    float error = (pid->input - pid->setpoint);
//    printf("\nErr %f\n", error);
    /* Calculate P term */
    p_f = pid->kp * error;

    /* Integrate I term */
    pid->e += (pid->ki * error); // >> SHIFT;

    /* Calculate D term */
    d_f = pid->kd * (pid->output - pid->last_output);

    /* Sum PID output */
    output_f = p_f + pid->e + d_f;
    output = output_f; // >> SHIFT;
    printf("\nOut %f\n", output);
    /* Set output_f, check min/max output */
    if (output < pid->min_output) output = pid->min_output;
    if (output > pid->max_output) output = pid->max_output;

    pid->last_output = pid->output;
    pid->output = pid->max_output - output;
}

/**
 *
 * echo /sys/devices/ocp.3/pwm_test_P8_13.* /
 *
 * @param pwmData
 */
void set_pwm(struct pwm* pwmData)
{
    pwmData->pwm = fopen("/sys/devices/bone_capemgr.9/slots", "w");
    if (pwmData->pwm == NULL)
        printf("Problem with file for pwm.\n");

    pwmData->period = fopen("/sys/devices/ocp.3/pwm_test_P8_13.15/period", "w");
    if (pwmData->period == NULL)
        printf("Problem with file for pwm-period.\n");

    pwmData->duty = fopen("/sys/devices/ocp.3/pwm_test_P8_13.15/duty", "w");
    if (pwmData->duty == NULL)
        printf("Problem with file for pwm-duty.\n");

    pwmData->run = fopen("/sys/devices/ocp.3/pwm_test_P8_13.15/run", "w");
    if (pwmData->run == NULL)
        printf("Problem with file for pwm-run.\n");

    int writeStatus = 0;
    //
    fseek(pwmData->pwm, 0, SEEK_SET);
    fprintf(pwmData->pwm, "am33xx_pwm");
    writeStatus = fflush(pwmData->pwm);
    printf("Status am33xx_pwm: %i\n", writeStatus);

    fprintf(pwmData->pwm, "bone_pwm_P8_13");
    writeStatus = fflush(pwmData->pwm);
    printf("Status bone_pwm_P8_13: %i\n", writeStatus);

    //
    fseek(pwmData->run, 0, SEEK_SET);
    fprintf(pwmData->run, "%d", 0);
    writeStatus = fflush(pwmData->run);
    printf("Status run0: %i\n", writeStatus);

    fseek(pwmData->run, 0, SEEK_SET);
    fprintf(pwmData->run, "%d", 1);
    writeStatus = fflush(pwmData->run);
    printf("Status run1: %i\n", writeStatus);
}

/**
 *
 * @param pwm pwmData
 */
void close_pwm(struct pwm* pwmData)
{
    fclose(pwmData->pwm);
    fclose(pwmData->duty);
    fclose(pwmData->period);
    fclose(pwmData->run);
}

/**
 * Updates pwm.
 *
 * @param pwm pwmData
 * @param int period 200000000
 *
 * @return void
 */
void update_pwm_period(struct pwm* pwmData, int iPeriod)
{
    int writeStatus = 0;
    fseek(pwmData->period, 0, SEEK_SET);
    fprintf(pwmData->period, "%d", iPeriod);
    writeStatus = fflush(pwmData->period);
//    printf("Period status: %i\n", writeStatus);
}

/**
 * Updates pwm.
 *
 * @param pwm pwmData
 * @param int iDuty   100000000
 *
 * @return void
 */
void update_pwm_duty(struct pwm* pwmData, int iDuty)
{
    int writeStatus = 0;
    fseek(pwmData->duty, 0, SEEK_SET);
    fprintf(pwmData->duty, "%d", iDuty);
    writeStatus = fflush(pwmData->duty);
//    printf("Duty status %i\n", writeStatus);

}
