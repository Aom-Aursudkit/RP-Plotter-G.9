/*
 * ModBusRTU.h
 *
 *  Created on: Feb 28, 2023
 *      Author: AlphaP-Tuf
 */

#ifndef INC_MODBUSRTU_H_
#define INC_MODBUSRTU_H_
#include "stm32g4xx_hal.h"
#include "string.h"
#define MODBUS_MESSAGEBUFFER_SIZE 300
typedef union {
	uint16_t U16;
	uint8_t U8[2];
} u16u8_t;
typedef enum _ModbusState {
	Modbus_state_Init,
	Modbus_state_Idle,
	Modbus_state_Emission,
	Modbus_state_Reception,
	Modbus_state_ControlAndWaiting
} ModbusStateTypedef;

typedef enum _modbusFunctioncode {

	Modbus_function_Read_Coil = 1,
	Modbus_function_Read_DiscreteInput,
	Modbus_function_Read_Holding_Register,
	Modbus_function_Read_Input_Registor,
	Modbus_function_Write_SingleCoil,
	Modbus_function_Write_SingleRegister,
	Modbus_function_Diagnostics = 8,
	Modbus_function_GetCommEventCounter = 11,
	Modbus_function_Write_MultipleCoil = 15,
	Modbus_function_Write_MultipleRegistor

} ModbusFunctionCode;

typedef enum _modbusRecvFrameStatus {
	Modbus_RecvFrame_Null = -2,
	Modbus_RecvFrame_FrameError = -1,
	Modbus_RecvFrame_Normal = 0,
	Modbus_RecvFrame_IllegalFunction,
	Modbus_RecvFrame_IllegalDataAddress,
	Modbus_RecvFrame_IllegalDataValue,
	Modbus_RecvFrame_SlaveDeviceFailure,
	Modbus_RecvFrame_Acknowlage,
	Modbus_RecvFrame_SlaveDeviceBusy,
	Modbus_RecvFrame_NegativeAcknowage,
	Modbus_RecvFrame_MemoryParityError,
	Modbus_RecvFrame_GatewayTargetDeviceFailedToRespon
} modbusRecvFrameStatus;

//Modbus Handle structure
typedef struct _ModbusHandleTypedef {
	uint8_t slaveAddress;

	//Register
	u16u8_t *RegisterAddress;
	uint32_t RegisterSize;

	UART_HandleTypeDef *huart; // 19200 8E1 , Enable Interrupt ,Register callback Enable

	TIM_HandleTypeDef *htim; // timer period = 2t ,Enable ONE pulse mode , Enable Interrupt ,Register callback Enable

	//flag
	uint8_t Flag_T15TimeOut;
	uint8_t Flag_T35TimeOut;

	modbusRecvFrameStatus RecvStatus;

	ModbusStateTypedef Mstatus; //Modbus state (for state machine)

	//PDU frame
	uint8_t Rxframe[MODBUS_MESSAGEBUFFER_SIZE];
	uint8_t Txframe[MODBUS_MESSAGEBUFFER_SIZE];
	uint8_t TxCount;
	//Serial frame
	struct _modbusUartStructure {
		uint8_t MessageBufferRx[MODBUS_MESSAGEBUFFER_SIZE + 3];
		uint16_t RxTail;
		uint8_t MessageBufferTx[MODBUS_MESSAGEBUFFER_SIZE + 3];
		uint16_t TxTail;
	} modbusUartStructure;

} ModbusHandleTypedef;

void Modbus_init(ModbusHandleTypedef*, u16u8_t*);
void Modbus_Protocal_Worker();

// ── Data Structures ──────────────────────────────────────────────────────────

//typedef struct {
//	uint16_t distance[10];  // Formerly R
//	uint16_t angle[10];     // Formerly Theta
//} TargetSet;

// ── External Frame Buffer ────────────────────────────────────────────────────

extern u16u8_t registerFrame[200];  // Frame data container

// ── Target Register Definitions ──────────────────────────────────────────────

#define REG_TARGET_BASE_ADDR    0x20
#define SLOT_GAP_ENTRIES        6

// compute per-slot offset (0–4 ⇒ +0; 5–9 ⇒ +6)
#define SLOT_OFFSET(slot)       ( (slot) < 5 \
                                  ? (slot)*2 \
                                  : (slot)*2 + SLOT_GAP_ENTRIES )

#define TARGET_DISTANCE(slot)   ( registerFrame[ REG_TARGET_BASE_ADDR \
                                               + SLOT_OFFSET(slot)      ].U16 )
#define TARGET_ANGLE(slot)      ( registerFrame[ REG_TARGET_BASE_ADDR \
                                               + SLOT_OFFSET(slot) + 1  ].U16 )

// Set distance and angle for a specific slot
#define SET_TARGET(slot, dist, ang)   \
    do {                              \
        TARGET_DISTANCE(slot) = (dist); \
        TARGET_ANGLE(slot)    = (ang);  \
    } while(0)

void ResetAllTargets(void);

// ── Bulk Read Helper ─────────────────────────────────────────────────────────

//static inline TargetSet readAllTargets(void) {
//	TargetSet targets;
//	for (uint8_t i = 0; i < 10; ++i) {
//		targets.distance[i] = TARGET_DISTANCE(i);
//		targets.angle[i] = TARGET_ANGLE(i);
//	}
//	return targets;
//}

// ── Register Address Map ─────────────────────────────────────────────────────

enum {
	HEART_BEAT = 0x00,
	REG_BASE_STATUS = 0x01,  // R
	REG_SERVO_LIMIT_SWITCH = 0x03,  // W
	REG_SERVO_CMD_UP = 0x04,  // R
	REG_SERVO_CMD_DOWN = 0x05,  // R

	REG_MOTION_STATUS = 0x10,  // W
	REG_POSITION_R = 0x11,  // W (mm*10, signed)
	REG_POSITION_THETA = 0x12,  // W (deg*10, signed)
	REG_SPEED_R = 0x13,  // W (mm/s*10)
	REG_SPEED_THETA = 0x14,  // W (deg/s*10)
	REG_ACCELERATION_R = 0x15,  // W (mm/s²*10)
	REG_ACCELERATION_THETA = 0x16,  // W (deg/s²*10)

	REG_TARGET_GOAL_R = 0x40,  // R
	REG_TARGET_GOAL_THETA = 0x41   // R
};

// ── Convenience Access Macros ────────────────────────────────────────────────

#define REG16(reg)   (registerFrame[(reg)].U16)     // 16-bit unsigned
#define REG8_LO(reg) (registerFrame[(reg)].U8[0])   // Low byte
#define REG8_HI(reg) (registerFrame[(reg)].U81])   // High byte

void Get_QRIdata(float *prism_vel_mm,
                 float *prism_acc_mm,
                 float *prism_mm_pos,
                 float *rev_ang_vel_rad,
                 float *rev_ang_acc_rad,
                 float *rev_rad_pos);

// Helper macro to clamp and round a float to uint16_t
#define FLOAT_TO_intU16(value) ((int16_t)(value * 10.0f) + 0.5f)

// Helper macro to convert radians to degrees * 10, safely
#define RAD_TO_DEG10(value) ((int16_t)((value) * (1800.0f / M_PI) + 0.5f))
// External variables

extern uint8_t Pen_Status;
extern uint8_t Pen_BaseSystem;
extern float TargetR_BaseSystem;
extern float TargetP_BaseSystem;
extern uint8_t State_BaseSystem;

#endif /* INC_MODBUSRTU_H_ */
