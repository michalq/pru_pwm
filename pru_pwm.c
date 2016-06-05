#include <stdio.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM 0
#define SHIFT 0

struct pid_data {
    /* PID tunings */
    int Kp_f, Ki_f, Kd_f;

    /* PID controls */
    int setpoint;
    int int_err;
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

void update_pid(volatile struct pid_data* pid);

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

    while (1) {
        prussdrv_pru_wait_event(PRU_EVTOUT_0);
        prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
        impulses++;
        printf("Impulse no. %i\n", impulses);
    }

    printf("Finished!\n");
    prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

    return 0;
}


void update_pid(volatile struct pid_data* pid) {
    unsigned int p_f, d_f;
    int output_f, output;

    /* Calculate error */
    int error = (pid->input - pid->setpoint);

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
