.origin 0
.entrypoint START

#include "../include/pru_gpio.hp"

#define OUT_MODULE GPIO1
#define OUT_PIN    28

#define IN_MODULE  GPIO0
#define IN_PIN     30

#define REG_SYSCFG         C4
#define PRU0_ARM_INTERRUPT 19       
      
START:
        lbco r0, REG_SYSCFG, 4, 4  // Load 4-8 config bit
	clr r0, r0, 4              // clear bit 4
	sbco r0, REG_SYSCFG, 4, 4  // and save again to enable ocp

STATE_1:
	getgpio IN_MODULE, IN_PIN
	qbeq STATE_1, r0, 0
STATE_2:
	mov r31.b0, PRU0_ARM_INTERRUPT+16
STATE_3:
	getgpio IN_MODULE, IN_PIN
	qbeq STATE_3, r0, 1
STATE_4:
	mov r31.b0, PRU0_ARM_INTERRUPT+16
	jmp STATE_1
