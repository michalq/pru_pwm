#include <stdio.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <sys/time.h>

#define PRU_NUM 0
#define SHIFT 0

// Counts per revolution, rising and falling edge.
#define CPR 4*144

struct pid_data {
    /* PID tunings */
    float kp, ki, kd;

    /* PID controls */
    int setpoint;
    int e;
    int input, output, last_output;
    int min_output, max_output;
};

/* Shared memory block struct */
struct shared_mem {
    volatile char init_flag;
    volatile unsigned int pwm_out;
    volatile int enc_rpm;
    volatile struct pid_data pid;
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
    motor1PID.Kp_f = 0.6;
    motor1PID.Ki_f = 1/8;
    motor1PID.Kd_f = .125;

    motor1PID.min_output = 20;
    motor1PID.max_output = 100;

    motor1PID.setpoint = 1;

    // Init PWM.
    struct pwm pwmData;
    set_pwm(&pwmData);
    update_pwm_period(&pwmData, 100);
    update_pwm_duty(&pwmData, 0);

    uint32_t now_ms, last_ms;
    float current_pos, last_pos, delta_pos;
    struct timespec now;

    while (1) {
        prussdrv_pru_wait_event(PRU_EVTOUT_0);
        prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
        impulses++;
        now_ms = (now.tv_sec * 1000 + (now.tv_nsec + 500000) / 1000000); //convert to milliseconds
        delta_ms = now_ms - last_ms;

        current_pos = impulses / CPR;
        delta_pos = last_pos - current_pos;
        // speed = (delta_pos*(1000*60/CPR))/(delta_ms); // @todo Do i need it?

        update_pid(motor1PID);
        update_pwm_duty(&pwmData, motor1PID.output);

        last_pos = current_pos;
        last_ms = now_ms;
        printf("Impulse no. %i\n", impulses);
        if (impulses == CPR) {
            break;
        }

    }

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
    unsigned float p_f, d_f;
    float output_f, output;

    /* Calculate error */
    float error = (pid->input - pid->setpoint);

    /* Calculate P term */
    p_f = pid->Kp_f * error;

    /* Integrate I term */
    pid->int_err += (pid->Ki_f * error) >> SHIFT;

    /* Calculate D term */
    d_f = pid->Kd_f * (pid->output - pid->last_output);

    /* Sum PID output */
    output_f = p_f + pid->int_err + d_f;
    output = output_f >> SHIFT;

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

    //
    fseek(pwmData->pwm, 0, SEEK_SET);
    fprintf(pwmData->pwm, "am33xx_pwm");
    fflush(pwmData->pwm);

    fprintf(pwmData->pwm, "bone_pwm_P8_13");
    fflush(pwmData->pwm);

    //
    fseek(pwmData->run, 0, SEEK_SET);
    fprintf(pwmData->run, "%d", 0);
    fflush(pwmData->run);

    fseek(pwmData->run, 0, SEEK_SET);
    fprintf(pwmData->run, "%d", 1);
    fflush(pwmData->run);
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
    fseek(pwmData->period, 0, SEEK_SET);
    fprintf(pwmData->period, "%d", iPeriod);
    fflush(pwmData->period);
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
    fseek(pwmData->duty, 0, SEEK_SET);
    fprintf(pwmData->duty, "%d", iDuty);
    fflush(pwmData->duty);
}
