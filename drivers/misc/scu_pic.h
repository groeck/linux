/*
 * Thermal Processor (PIC) I2C Message Protocol Defines
 */

/* Receive Messages */
#define I2C_SET_SCU_PIC_RESET_HOST			0x01
#define I2C_SET_SCU_PIC_FAN_STATE			0x02
#define I2C_SET_SCU_PIC_WDT_STATE			0x03
#define I2C_SET_SCU_PIC_THERMAL_CONTROL_STATE		0x04
#define I2C_SET_SCU_PIC_FAULT_LED_STATE			0x05
#define I2C_SET_SCU_PIC_FORCED_OFF			0x06

#define I2C_GET_SCU_PIC_FIRMWARE_REV_MAJOR		0x10
#define I2C_GET_SCU_PIC_FIRMWARE_REV_MINOR		0x11
#define I2C_GET_SCU_PIC_FAN1_SPEED			0x12
#define I2C_GET_SCU_PIC_FAN2_SPEED			0x13
#define I2C_GET_SCU_PIC_LOCAL_TEMP			0x14
#define I2C_GET_SCU_PIC_REMOTE_TEMP			0x15
#define I2C_GET_SCU_PIC_WDT_STATE			0x16
#define I2C_GET_SCU_PIC_THERMAL_CONTROL_STATE		0x17
#define I2C_GET_SCU_PIC_FAULT_LED_STATE			0x18
#define I2C_GET_SCU_PIC_RESET_REASON			0x19

/*
 * State Definitions
 */

/* LED States */
#define LED_OFF			0
#define LED_BLINK_SLOW		1
#define LED_BLINK_FAST		2
#define LED_ON			3
