.origin 0
.entrypoint START

#include "../include/pru_gpio.hp"

#define OUT_MODULE GPIO1
#define OUT_PIN    28

#define IN_MODULE  GPIO0
#define IN_ENC_1A     30
#define IN_ENC_1B     31


#define REG_SYSCFG         C4
#define PRU0_ARM_INTERRUPT 19       
      
START:
        lbco r0, REG_SYSCFG, 4, 4  // Load 4-8 config bit
	clr r0, r0, 4              // clear bit 4
	sbco r0, REG_SYSCFG, 4, 4  // and save again to enable ocp
	mov r2, 0
	mov r3, 0
CHECK_A:
	getgpio IN_MODULE, IN_ENC_1A
	qbeq CHECK_B, r0, r2
	qbeq SET_HA, r0, 1
	qbeq SET_LA, r0, 0
CHECK_B:
	getgpio IN_MODULE, IN_ENC_1B
	qbeq CHECK_A, r0, r3
	qbeq SET_HB, r0, 1
	qbeq SET_LB, r0, 0
SET_HA:
	mov r2, 1
	jmp IRQQ
SET_LA:
	mov r2, 0
	jmp IRQQ
SET_HB:
	mov r3, 1
	jmp IRQQ
SET_LB:
	mov r3, 0
	jmp IRQQ
IRQQ:
	mov r31.b0, PRU0_ARM_INTERRUPT+16
	jmp CHECK_A
