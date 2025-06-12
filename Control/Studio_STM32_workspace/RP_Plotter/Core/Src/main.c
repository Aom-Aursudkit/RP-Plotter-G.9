/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "arm_math.h"
#include "stdio.h"
#include "string.h"
#include <math.h>

// BaseSystem//////////
#include "ModBusRTU.h"
//////////////////////

// Trapezoidal/////////
#include "Trapezoidal.h"
//////////////////////

// FIBO_G09///////////
#include "all_path.h"
#include "dora.h"
#include <stdbool.h>
//////////////////////

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_ERR_TOL_RAD 0.0068f   // 0.00174533f/* ±0.1 degree */
#define P_ERR_TOL_MM 0.40f	   // 0.10f      /* ±0.1 mm */
#define HOLD_TIME_US 1000000UL /* 1s  in microseconds */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint64_t _micros = 0;

// MotorParam//////////
float P_Motor_R = 3.43;
float P_Motor_L = 0.00124;
float P_Motor_B = 0.000222663543417634;
float P_Motor_J = 0.0000199333070343912;
float P_Motor_Ke = 0.027615623075429;
float P_Motor_Km = 0.0274929200534876;
float R_Motor_R = 10.86;
float R_Motor_L = 0.01116;
float R_Motor_B = 0.067455299899581;
float R_Motor_J = 0.002236277790154;
float R_Motor_Ke = 0.19423665004696;
float R_Motor_Km = 0.09600489295806550;
//////////////////////

// RevoluteFeedforward/
float TargetR_Deg = 0;
float R_Pos_Error_Deg = 0;
float g = 9.81;
float R_Inertia = 0.2;
float R_Accel = 0.4;
float R_Mass = 4.3;
float R_length = 0;
float R_torque_ff = 0;
float R_current_ff = 0;
float R_v_ff = 0;
float R_pwm_ff = 0;
float R_Kff = 0.5f;
//////////////////////

// Receiver////////////
float Receiver[5];
int32_t Receiver_Period[5];
// uint16_t ADC_RawRead[300]={0};
volatile uint32_t rise_time[3] = { 0 }; // For PC0, PC2, PC3
volatile uint32_t pulse_width_us[3] = { 0 };
//////////////////////

// Encoder/////////////
uint32_t revolute_raw;
uint32_t prismatic_raw;

typedef struct {
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float QEIPostion_1turn;
	float QEIVelocity;
	float QEIAcceleration;
	float Velocity;
	float Velocity_f;
	float Acceleration;
	int64_t AbsolutePosition;
	float RadPosition;
	float DegPosition;
} Revolute_QEI_StructureTypeDef; // rad

Revolute_QEI_StructureTypeDef Revolute_QEIdata = { 0 };

typedef struct {
	uint32_t Position[2];
	uint64_t TimeStamp[2];
	float QEIVelocity;
	float QEIAcceleration;
	float Velocity;
	float Velocity_f;
	float Acceleration;
	int64_t AbsolutePosition;
	float mmPosition;
} Prismatic_QEI_StructureTypeDef; // mm

Prismatic_QEI_StructureTypeDef Prismatic_QEIdata = { 0 };

enum {
	NEW, OLD
};

uint8_t Z_index_R;
uint8_t z_temp;
//////////////////////

// Control/////////////
float vx;
float vy;
float End_x;
float End_y;
float TargetX;
float TargetY;
float TargetR;
float TargetP;
float TargetRPos;
float TargetPPos;
float TargetRVel;
float TargetPVel;
float TargetRAcc;
float TargetPAcc;
float inv_L;
float R_Pos_Error;
float P_Pos_Error;
float R_Velo_Error;
float P_Velo_Error;

float R_PWM;
float P_PWM;
//////////////////////

// Mode & Status///////
uint8_t Mode;
uint8_t Last_Mode;
uint8_t EmergencyState;
uint8_t IsPress;
uint8_t Pen_Status;
uint8_t Pen_Status_in;
uint8_t P_Limit;
uint8_t R_Limit;
uint64_t lock_timer_us = 0;
//////////////////////

// PID/////////////////
arm_pid_instance_f32 PID = { 0 };

typedef struct {
	float integ;
	float prevError;
} PID_State;

PID_State pid_r = { 0 };	 // for Revolute
PID_State pid_p = { 0 };	 // for Prismatic
PID_State pid_r_v = { 0 }; // for Revolute
PID_State pid_p_v = { 0 }; // for Prismatic

float R_kP_vel = 90.0f;
float R_kI_vel = 4.00f;
float R_kD_vel = 0.20f;

float R_kP_pos = 5.00f;
float R_kI_pos = 4.00f;
float R_kD_pos = 1.00f;

float P_kP_vel = 1.39524f;
float P_kI_vel = 0.034f;
float P_kD_vel = 0.00f;

float P_kP_pos = 2.85f;
float P_kI_pos = 0.41f;
float P_kD_pos = 0.2034f;

float prev_TargetP = 0.0f, max_P_position = 0.0f;
float prev_TargetR = 0.0f, max_R_position = 0.0f;
float P_overshoot = 0.0f, R_overshoot = 0.0f;
//////////////////////

// Mode3///////////////
typedef enum {
	CALIB_IDLE,
	CALIB_WAIT_REMOTE,
	CALIB_MOVE_P_TO_LIMIT,
	CALIB_WAIT_BACKOFF_P,
	CALIB_BACKOFF_P,
	CALIB_WAIT_RETOUCH_P,
	CALIB_RETOUCH_P,
	CALIB_MOVE_R_TO_LIMIT,
	CALIB_WAIT_BACKOFF_R,
	CALIB_BACKOFF_R,
	CALIB_WAIT_RETOUCH_R,
	CALIB_RETOUCH_R,
	CALIB_DONE
} CalibState_t;
CalibState_t calibState = CALIB_IDLE;
uint64_t calib_timer = 0;
uint8_t Cal_Side = 0;
uint64_t servo_timer;
//////////////////////

// Mode8///////////////
uint8_t counter8 = 0;
bool goCenter8 = true;
//////////////////////

// BaseSystem//////////
uint8_t counter = 0;
bool TenPointMode = false;
bool Testbool_BaseSystem = false;

bool Test_no_BaseSystem = false;
float TargetR_BaseSystem = 0;
float TargetP_BaseSystem = 0;
uint8_t State_BaseSystem = 0;
uint8_t Pen_BaseSystem = 0;
float Last_TargetR_BaseSystem = 0;
float Last_TargetP_BaseSystem = 0;
uint8_t Last_State_BaseSystem = 0;
uint8_t Last_Pen_BaseSystem = 0;
ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];
uint16_t base_status;
float TenPointArray[20] = { 0 };
bool testArraydone = false;
uint64_t currentTimer = 0;
// float RD_Velo_Error;
//////////////////////

// Trapezoidal/////////
static float last_TargetR = 0.0f;
static float last_TargetP = 0.0f;
//////////////////////

//////////////////////
typedef enum {
	IDLE, WAITING_BEFORE, WAITING_AFTER
} ServoState;
ServoState servo_state = IDLE;
uint64_t pen_delay_timer = 0;
uint8_t CheckTolerance = 0;
VELO_PROFILE revolute;
VELO_PROFILE prismatic;
//////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);
uint64_t micros();
float PID_Update(float error, float kP, float kI, float kD, float dt,
		float outMin, float outMax, PID_State *state);
void Revolute_PosVel_Update();
void Prismatic_PosVel_Update();
void Set_Motor(int motor_num, float speed);
void Set_Servo(int Pen_Pos);
void Reset_R();
void Reset_P();
void Workspace_limit();
float updateAndGetOvershoot(float target, float *prev_target,
		float *max_position, float current_position);

bool PenDelay(void);
// Cascade//////////
int CascadeControl_Step(void);
void TrapezoidStep(void);
void PIDStep(void);
int ToleranceCheck(void);
//////////////////////

// BaseSystem//////////
void processTargets(void);
void ResetAllTargets(void);
//////////////////////

// FIBO_G09//////////
float Trapezoidal_CalcTotalTime(float distance, float vmax, float amax);
float Trapezoidal_CalcVmaxFromTime(float distance, float amax, float total_time);
void InverseKinematics(float x, float y, float *r, float *p);
int current_index = 0;
void updatePathStep(Point *path_name, int path_length);
//////////////////////

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
	MX_TIM15_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM16_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim5);

	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_Base_Start(&htim15);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	//  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	//  HAL_ADC_Start_DMA(&hadc1, ADC_RawRead, 300);
	DWT_Init();

	PID.Kp = 0.1;
	PID.Ki = 0.00001;
	PID.Kd = 0.1;
	arm_pid_init_f32(&PID, 0);

	Reset_R();
	Reset_P();
	calibState = CALIB_IDLE;

	Set_Servo(0);

	// BaseSystem//////////
	hmodbus.huart = &huart2;
	hmodbus.htim = &htim16;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize = 200;
	Modbus_init(&hmodbus, registerFrame);
	ResetAllTargets;
	REG16(REG_MOTION_STATUS) = 0;
	Set_Motor(0, 0);
	Set_Motor(1, 0);
	//////////////////////

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//////////////////////// <<BaseSystem>> /////////////////////////
		if (!Test_no_BaseSystem && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1) {
			Test_no_BaseSystem = true;
		}

		base_status = REG16(REG_BASE_STATUS);
		if (EmergencyState == 0) {
			switch (base_status) {
			case 1: // Home
				REG16(REG_MOTION_STATUS) = 1;
				break;

			case 2: // Run Jog Mode
				REG16(REG_MOTION_STATUS) = 2;
				break;

			case 4: // Run Point Mode
				REG16(REG_MOTION_STATUS) = 4;
				break;

			default:
				break;
			}
			if (Pen_BaseSystem != Last_Pen_BaseSystem) {
				Set_Servo(Pen_BaseSystem);
				Last_Pen_BaseSystem = Pen_BaseSystem;
			}
		}
		//		RD_Velo_Error = R_Velo_Error * (M_PI / 180.0f);
		Modbus_Protocal_Worker();
		//////////////////////////////////////////////////////////////

		//////////////////////// <<ENCODER>> /////////////////////////
		revolute_raw = __HAL_TIM_GET_COUNTER(&htim4);
		prismatic_raw = __HAL_TIM_GET_COUNTER(&htim3);
		// Call every 0.001 s
		static uint64_t timestamp = 0;
		int64_t currentTime = micros();
		if (currentTime > timestamp) {
			timestamp = currentTime + 1000; // us
			Revolute_PosVel_Update();
			Prismatic_PosVel_Update();
		}
		// Call every 0.01 s
		static uint64_t timestamp1 = 0;
		int64_t currentTime1 = micros();
		if (currentTime1 > timestamp1) {
			timestamp1 = currentTime1 + 10000; // us
		}
		// Call every 0.1 s
		static uint64_t timestamp2 = 0;
		int64_t currentTime2 = micros();
		if (currentTime2 > timestamp2) {
			timestamp2 = currentTime2 + 100000; // us
		}
		End_x = Prismatic_QEIdata.mmPosition
				* cosf(Revolute_QEIdata.RadPosition * -1);
		End_y = Prismatic_QEIdata.mmPosition
				* sinf(Revolute_QEIdata.RadPosition);
		//////////////////////////////////////////////////////////////

		//////////////////////// <<MODE>> ///////////////////////////
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1) {
			Mode = 0;
			EmergencyState = 1;
			Set_Servo(0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1);
		}
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == 1) {
			EmergencyState = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 0);
		}

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1) {
			Pen_Status = 0;
		}
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 1) {
			Pen_Status = 1;
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1) {
			z_temp = 1;
		} else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0) {
			z_temp = 0;
		}

		//////////////////////////////////////////////////////////////
		if (EmergencyState == 1) {
			REG16(REG_MOTION_STATUS) = 0;
		}
		//////////////////////////////////////////////////////////////

		//////////////////////// <<GOTO>> ////////////////////////////
		if ((Mode == 2 && (base_status == 2 || Test_no_BaseSystem))
				|| base_status == 8) {

			P_overshoot = updateAndGetOvershoot(TargetP, &prev_TargetP,
					&max_P_position, Prismatic_QEIdata.mmPosition);
			R_overshoot = updateAndGetOvershoot(TargetR, &prev_TargetR,
					&max_R_position, Revolute_QEIdata.RadPosition);

			if (base_status == 8) {
				REG16(REG_MOTION_STATUS) = 8;
			}

			if (base_status == 8) {
				TargetR = TargetR_BaseSystem;
				TargetP = TargetP_BaseSystem;
			} else if (TenPointMode) {
				TargetR = TenPointArray[(counter * 2) + 1];
				TargetP = TenPointArray[counter * 2];
			}

			if (CascadeControl_Step()) {
				if (base_status == 8) {
					REG16(REG_MOTION_STATUS) = 0;
					REG16(REG_BASE_STATUS) = 0;
				} else {
					if (PenDelay()) {
						if (TenPointMode) {
							if (counter == 9) {
								TenPointMode = false;
								counter = 0;
								Mode = 1;
							} else {
								counter++;
							}
						}
					}
				}
			}
		}
		//////////////////////////////////////////////////////////////

		//////////////////////// <<CALIBRATING>> /////////////////////
		if ((Mode == 3 && (base_status == 2 || Test_no_BaseSystem))
				|| base_status == 1) {
			switch (calibState) {
			case CALIB_IDLE:
				Set_Motor(0, 0);
				Set_Motor(1, 0);
				P_Limit = 0;
				R_Limit = 0;
				calibState = CALIB_WAIT_REMOTE;
				calib_timer = micros();
				break;

			case CALIB_WAIT_REMOTE:
				if (Receiver[0] > 80) {
					Cal_Side = 1;
					calibState = CALIB_MOVE_P_TO_LIMIT;
				} else if (Receiver[0] < -80) {
					Cal_Side = 2;
					calibState = CALIB_MOVE_P_TO_LIMIT;
				}
				break;

			case CALIB_MOVE_P_TO_LIMIT:
				Set_Motor(1, -30);

				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1 || P_Limit > 0) {
					Set_Motor(1, 0);
					calib_timer = micros();
					P_Limit = 0;
					calibState = CALIB_WAIT_BACKOFF_P;
				}
				break;

			case CALIB_WAIT_BACKOFF_P:
				Set_Motor(1, 0);

				if (micros() - calib_timer > 100000) {
					Set_Motor(1, 30);
					calib_timer = micros();
					P_Limit = 0;
					calibState = CALIB_BACKOFF_P;
				}

			case CALIB_BACKOFF_P:
				Set_Motor(1, 30);

				if (micros() - calib_timer > 100000) {
					Set_Motor(1, 0);
					calib_timer = micros();
					P_Limit = 0;
					calibState = CALIB_WAIT_RETOUCH_P;
				}
				break;

			case CALIB_WAIT_RETOUCH_P:
				Set_Motor(1, 0);

				if (micros() - calib_timer > 100000) {
					Set_Motor(1, -20);
					calib_timer = micros();
					P_Limit = 0;
					calibState = CALIB_RETOUCH_P;
				}
				break;

			case CALIB_RETOUCH_P:
				Set_Motor(1, -20);

				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1 || P_Limit > 0) {
					Set_Motor(1, 0);
					Reset_P();
					P_Limit = 0;
					calibState = CALIB_MOVE_R_TO_LIMIT;
				}
				break;

			case CALIB_MOVE_R_TO_LIMIT:
				if (Cal_Side == 1)
					Set_Motor(0, -50);
				else
					Set_Motor(0, 50);

				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 1 || R_Limit > 0) {
					Set_Motor(0, 0);
					calib_timer = micros();
					R_Limit = 0;
					calibState = CALIB_WAIT_BACKOFF_R;
				}
				break;

			case CALIB_WAIT_BACKOFF_R:
				Set_Motor(0, 0);

				if (micros() - calib_timer > 100000) {
					if (Cal_Side == 1)
						Set_Motor(0, 50);
					else
						Set_Motor(0, -50);
					calib_timer = micros();
					R_Limit = 0;
					calibState = CALIB_BACKOFF_R;
				}
				break;

			case CALIB_BACKOFF_R:
				if (Cal_Side == 1)
					Set_Motor(0, 50);
				else
					Set_Motor(0, -50);

				if (micros() - calib_timer > 200000) {
					Set_Motor(0, 0);
					calib_timer = micros();
					R_Limit = 0;
					calibState = CALIB_WAIT_RETOUCH_R;
				}
				break;

			case CALIB_WAIT_RETOUCH_R:
				Set_Motor(0, 0);
				Z_index_R = 0;

				if (micros() - calib_timer > 100000) {
					if (Cal_Side == 1)
						Set_Motor(0, -35);
					else
						Set_Motor(0, 35);
					calib_timer = micros();
					R_Limit = 0;
					calibState = CALIB_RETOUCH_R;
				}
				break;

			case CALIB_RETOUCH_R:
				if (Cal_Side == 1)
					Set_Motor(0, -35);
				else
					Set_Motor(0, 35);

				//				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 1 || R_Limit > 0) {
				if (Z_index_R > 0) {
					Set_Motor(0, 0);
					Reset_R();
					R_Limit = 0;
					calibState = CALIB_DONE;
				}
				break;

			case CALIB_DONE:
				Cal_Side = 0;
				Mode = 0;
				P_Limit = 0;
				R_Limit = 0;
				calibState = CALIB_IDLE;
				//////////////////////////////////////////////////////////////
				if (base_status == 1) {
					REG16(REG_MOTION_STATUS) = 0;
				}
				//////////////////////////////////////////////////////////////
				break;
			}
		}
		//////////////////////////////////////////////////////////////

		////////////////////////// <<BASESYSTEM>> ////////////////////////
		if (base_status == 2 || Test_no_BaseSystem) {
			currentTimer = micros(); // Current time in microseconds

			//////////////////////// <<RECEIVER>> ////////////////////////
			Receiver_Period[0] = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
			Receiver_Period[1] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
			float RX_temp = map(
			__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2) - 1500.00, -500.00,
					500.00, -100.00, 100.00);
			float RY_temp = map(
			__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) - 18530.00, -500.00,
					500.00, -100.00, 100.00);

			if (RX_temp >= -4 && RX_temp <= 4)
				Receiver[0] = 0.00;
			else if (RX_temp > 100)
				Receiver[0] = 100.00;
			else if (RX_temp < -100)
				Receiver[0] = -100.00;
			else
				Receiver[0] = RX_temp;
			if (RY_temp >= -4 && RY_temp <= 4)
				Receiver[1] = 0.00;
			else if (RY_temp > 100)
				Receiver[1] = 100.00;
			else if (RY_temp < -100)
				Receiver[1] = -100.00;
			else
				Receiver[1] = RY_temp;

			if (pulse_width_us[0] > 4000.00)
				pulse_width_us[0] = pulse_width_us[0] - 4900.00;
			if (pulse_width_us[1] > 4000.00)
				pulse_width_us[1] = pulse_width_us[1] - 4900.00;
			if (pulse_width_us[2] > 4000.00)
				pulse_width_us[2] = pulse_width_us[2] - 4900.00;

			Receiver[2] = map((float) pulse_width_us[0] - 1500.00, -500.00,
					500.00, -100.00, 100.00);
			Receiver[3] = map((float) pulse_width_us[1] - 1500.00, -500.00,
					500.00, -100.00, 100.00);
			Receiver[4] = map((float) pulse_width_us[2] - 1500.00, -500.00,
					500.00, -100.00, 100.00);

			//		if(Receiver[2] > 1500.00) Receiver[2] = Receiver[2] - 3400.00;
			//		if(Receiver[3] > 1500.00) Receiver[3] = Receiver[3] - 3400.00;
			//		if(Receiver[4] > 1500.00) Receiver[4] = Receiver[4] - 3400.00;

			// Adjust as needed
			vx = map((float) Receiver[0], -100.00, 100.00, -300.00, 300.00);
			vy = map((float) Receiver[1], -100.00, 100.00, -300.00, 300.00);
			//////////////////////////////////////////////////////////////

			//////////////////////// <<MODE>> ///////////////////////////

			if (EmergencyState == 1) {
				Mode = 0;
			} else if (Receiver[2] < -30 && Receiver[4] < -30) {
				Mode = 0;
			} else if (Receiver[3] > 0 && IsPress == 0) {
				IsPress = 1;
				calibState = CALIB_IDLE;
				if (Receiver[2] > -30 && Receiver[2] < 30
						&& Receiver[4] < -30) {
					if (Mode != 1) {
						Mode = 1;
					} else {
						TargetX = End_x;
						TargetY = End_y;
						TargetR = Revolute_QEIdata.RadPosition;
						TargetP = Prismatic_QEIdata.mmPosition;
					}
				} else if (Receiver[2] > 30 && Receiver[4] < -30) {
					revolute.finished = 0;
					prismatic.finished = 0;
					Mode = 2;
				} else if (Receiver[2] < -30 && Receiver[4] > -30
						&& Receiver[4] < 30) {
					Mode = 3;
				} else if (Receiver[2] > -30 && Receiver[2] < 30
						&& Receiver[4] > -30 && Receiver[4] < 30) {
					revolute.finished = 0;
					prismatic.finished = 0;
					Mode = 4;
				} else if (Receiver[2] > 30 && Receiver[4] > -30
						&& Receiver[4] < 30) {
					Mode = 5;
				} else if (Receiver[2] < -30 && Receiver[4] > 30) {
					Mode = 6;
				} else if (Receiver[2] > -30 && Receiver[2] < 30
						&& Receiver[4] > 30) {
					Mode = 7;
				} else if (Receiver[2] > 30 && Receiver[4] > 30) {
					// loop_counter = 0;
					TargetR = 4.18879;
					TargetP = 50;
					Mode = 8;
				}
			} else {
				IsPress = 0;
			}
			//////////////////////////////////////////////////////////////

			//////////////////////// <<STOP>> ////////////////////////////
			if (Mode == 0) {
				Set_Motor(0, 0);
				Set_Motor(1, 0);
				if (Last_Mode != Mode) {
					Set_Servo(0);
					Last_Mode = Mode;
				}
			} else {
				Last_Mode = 255;
			}
			//////////////////////////////////////////////////////////////

			//////////////////////// <<MANUAL>> //////////////////////////
			if (Mode == 1) {
				//////////////////////// <<CONTROL>> /////////////////////////
				//		inv_L = (Prismatic_QEIdata.mmPosition > 1.0f) ? (1.0f / Prismatic_QEIdata.mmPosition) : 0.0f;
				//		TargetRVel 	= (-sinf(Revolute_QEIdata.RadPosition) * vx + cosf(Revolute_QEIdata.RadPosition) * vy) / inv_L;
				//		TargetPVel  =  cosf(Revolute_QEIdata.RadPosition) * vx + sinf(Revolute_QEIdata.RadPosition) * vy;
				TargetRVel = (map((float) Receiver[0], -100.00, 100.00, -1.00,
						1.00));
				TargetPVel = map((float) Receiver[1], -100.00, 100.00, -500.00,
						500.00);
				//////////////////////////////////////////////////////////////

				//////////////////////// <<MOTOR>> ///////////////////////////
				R_Velo_Error = (TargetRVel - Revolute_QEIdata.Velocity_f);
				P_Velo_Error = TargetPVel - Prismatic_QEIdata.Velocity;

				//Call every 0.001 s
				static uint64_t timestampState1 = 0;
				int64_t currentTimeState1 = micros();
				if (currentTimeState1 > timestampState1) {
					timestampState1 = currentTimeState1 + 1000;		//us
					R_PWM = PID_Update(R_Velo_Error, R_kP_vel, R_kI_vel,
							R_kD_vel, 0.01f, -100.0f, 100.0f, &pid_r_v);
					P_PWM = PID_Update(P_Velo_Error, P_kP_vel, P_kI_vel,
							P_kD_vel, 0.01f, -100.0f, 100.0f, &pid_p_v);
				}

				//			R_PWM = Receiver[0];
				//			P_PWM = Receiver[1];

				Workspace_limit();

				Set_Motor(0, R_PWM);
				Set_Motor(1, P_PWM);
				//////////////////////////////////////////////////////////////
			}
			//////////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////////
			if (Mode == 4) {
				static bool sequence_active = false;
				static bool waiting_for_up = false;
				static uint64_t pen_timestamp = 0;
				const uint64_t pen_delay = 200000; // 200 ms

				static float last_TargetX = 0.0f;
				static float last_TargetY = 0.0f;
				static bool new_target = true;

				// Detect change in target (with small tolerance to avoid float jitter)
				if (fabsf(TargetX - last_TargetX) > 1e-3f
						|| fabsf(TargetY - last_TargetY) > 1e-3f) {
					last_TargetX = TargetX;
					last_TargetY = TargetY;
					new_target = true;
				}

				InverseKinematics(TargetX, TargetY, &TargetR, &TargetP);
				TargetR_Deg = TargetR * 180.0f / M_PI;

				// Only start when there's a new target AND we're at the target
				if (!sequence_active && new_target && Pen_Status == 1
						&& CascadeControl_Step()) {
					Set_Servo(1); // Tell pen to press
					pen_timestamp = micros();
					sequence_active = true;
					waiting_for_up = true;
					new_target = false; // consume the new target
				}

				if (sequence_active && waiting_for_up
						&& micros() - pen_timestamp >= pen_delay) {
					Set_Servo(0); // Tell pen to lift
					pen_timestamp = micros();
					waiting_for_up = false;
				}

				if (sequence_active && !waiting_for_up && Pen_Status == 1
						&& micros() - pen_timestamp >= pen_delay) {
					Set_Motor(0, 0);
					Set_Motor(1, 0);
					sequence_active = false;
				}
			}
			//////////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////////
			if (Mode == 5) {
				static uint64_t timestampState5 = 0;
				int64_t currentTimeState5 = micros();
				if (currentTimeState5 > timestampState5) {
					timestampState5 = currentTimeState5 + 1000; // us

					P_kP_vel = 1.05845642f;
					P_kI_vel = 0.0496f;
					P_kD_vel = 0.00f;

					P_kP_pos = 3.0367f;
					P_kI_pos = 0.10198f;
					P_kD_pos = 0.0047f;

					bool reachedR = fabsf(
							TargetR - Revolute_QEIdata.RadPosition) < 0.068;
					bool reachedP = fabsf(
							TargetP - Prismatic_QEIdata.mmPosition) < 0.5;
					bool all_reached = revolute.finished && prismatic.finished
							&& reachedR && reachedP;

					int path_len = path_FIBO_length;

					if (current_index >= path_len) {
						if (all_reached) {
							Set_Servo(0);
							Set_Motor(0, 0);
							Set_Motor(1, 0);
						}
					} else {
						if (all_reached) {
							current_index++;
						}
					}

					Point target_point = path_FIBO[current_index];

					InverseKinematics(target_point.x, target_point.y, &TargetR,
							&TargetP);
					Set_Servo(target_point.p);

					R_Pos_Error = TargetR - Revolute_QEIdata.RadPosition;
					P_Pos_Error = TargetP - Prismatic_QEIdata.mmPosition;
					float R_Time = Trapezoidal_CalcTotalTime(R_Pos_Error, 1.4f,
							9.0f);
					float P_Time = Trapezoidal_CalcTotalTime(P_Pos_Error,
							300.0f, 1500.0f);
					float new_R_vmax = 1.4f;
					float new_P_vmax = 300.0f;
					if (R_Time > P_Time) {
						// Revolute is slower -> reduce Prismatic vmax
						new_P_vmax = Trapezoidal_CalcVmaxFromTime(P_Pos_Error,
								1500.0f, R_Time);
						P_Time = R_Time;
					} else if (P_Time > R_Time) {
						// Prismatic is slower -> reduce Revolute vmax
						new_R_vmax = Trapezoidal_CalcVmaxFromTime(R_Pos_Error,
								9.0f, P_Time);
						R_Time = P_Time;
					}
					if (revolute.finished
							&& fabsf(TargetR - last_TargetR) > 0.001f) {
						Trapezoidal_Init(&revolute, R_Pos_Error, new_R_vmax,
								9.0f);
						last_TargetR = TargetR;
					}
					if (prismatic.finished
							&& fabsf(TargetP - last_TargetP) > 0.01f) {
						Trapezoidal_Init(&prismatic, P_Pos_Error, new_P_vmax,
								1500.0f);
						last_TargetP = TargetP;
					}
					Trapezoidal_Update(&revolute, 0.001f);
					TargetRPos = revolute.current_position;
					TargetRVel = revolute.current_velocity;
					TargetRAcc = revolute.current_acceleration;
					Trapezoidal_Update(&prismatic, 0.001f);
					TargetPPos = prismatic.current_position;
					TargetPVel = prismatic.current_velocity;
					TargetPAcc = prismatic.current_acceleration;
					PIDStep();
					Workspace_limit();
					Set_Motor(0, R_PWM);
					Set_Motor(1, P_PWM);
				}
			}
			//////////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////////
			if (Mode == 6) {
				static bool PenIsNotDelay = true;
				static uint64_t lastPressTime6 = 0;
				if (PenIsNotDelay) {
					if (IsPress && currentTimer - lastPressTime6 >= 2000000) {
						lastPressTime6 = currentTimer;
						if (TenPointMode) {
							Mode = 2;
						} else {
							TenPointArray[counter * 2] =
									Prismatic_QEIdata.mmPosition;
							TenPointArray[(counter * 2) + 1] =
									Revolute_QEIdata.RadPosition;
							SET_TARGET(counter, Prismatic_QEIdata.mmPosition,
									Revolute_QEIdata.RadPosition);
							PenIsNotDelay = PenDelay();

							counter++;
							if (counter >= 10) {
								counter = 0;
								testArraydone = true;
								TenPointMode = true;
							}
						}
					} else if (!TenPointMode) {
						Mode = 1;
					}
				} else {
					PenIsNotDelay = PenDelay();
				}
			}
			//////////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////////
			if (Mode == 7) {
				static uint64_t lastPressTime7 = 0;
				if (testArraydone && IsPress
						&& currentTimer - lastPressTime7 >= 2000000) {
					lastPressTime7 = currentTimer;
					TenPointMode = true;
				} else {
					Mode = 2;
				}
			}
			//////////////////////////////////////////////////////////////

			//////////////////////////////////////////////////////////////
			if (Mode == 8) {
				if (counter8 < 10) {
					goCenter8 = true;

					if (goCenter8) {
						TargetR = M_PI_2;
						TargetP = 0;
					} else {
						TargetR = M_PI_4;
						TargetP = 150;
					}

					if (CascadeControl_Step()) {
						if (PenDelay()) {
							if (goCenter8) {
								counter8++;
							}
							goCenter8 = !goCenter8;
						}
					}
				} else if (counter8 >= 10 && IsPress) {
					counter8 = 0;
				}
			}
			//////////////////////////////////////////////////////////////
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 169;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 169;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 4;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 4;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 4;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 4;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 169;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 169;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 100;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 100;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 169;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 20000;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */
	HAL_TIM_MspPostInit(&htim15);

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 169;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 1145;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_9B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_EVEN;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_12,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 PC2 PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC5 PC6 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB2 PB11 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float updateAndGetOvershoot(float target, float *prev_target,
		float *max_position, float current_position) {
	// Reset max_position if the target changes
	if (target != *prev_target) {
		*prev_target = target;
		*max_position = current_position;
	}

	// Track maximum deviation from target (not just max raw position)
	if (fabsf(current_position - target) > fabsf(*max_position - target)) {
		*max_position = current_position;
	}

	if (target == 0.0f)
		return 0.0f; // avoid division by zero

	float overshoot = (fabsf(*max_position - target) / fabsf(target)) * 100.0f;
	return overshoot;
}

bool PenDelay(void) {
	static int state = 0;
	unsigned long now = micros();
	bool Done = false;

	switch (state) {
	case 0: // Start sequence with initial wait
		pen_delay_timer = now;
		state = 1;
		break;
	case 1: // Waiting for first 500ms
		if (now - pen_delay_timer >= 500000UL) {
			Set_Servo(1);
			pen_delay_timer = now;
			state = 2;
		}
		break;
	case 2: // Waiting for second 500ms after servo set to 1
		if (now - pen_delay_timer >= 500000UL) {
			Set_Servo(0);
			pen_delay_timer = now;
			state = 3;
		}
		break;
	case 3: // Waiting for third 500ms after servo set to 0
		if (now - pen_delay_timer >= 500000UL) {
			state = 0;
			Done = true;
		}
		break;
	}

	return Done;
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DWT_Init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_9) {
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 1) {
			Z_index_R += 1;
		}
	}
	if (GPIO_Pin == GPIO_PIN_10) {
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == 1) {
			Mode = 0;
			EmergencyState = 1;
		}
	}
	if (GPIO_Pin == GPIO_PIN_12) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1) {
			P_Limit = 1;
		} else {
			P_Limit = 0;
		}
	}
	if (GPIO_Pin == GPIO_PIN_13) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 1) {
			R_Limit = 1;
		} else {
			R_Limit = 0;
		}
	}
	if (GPIO_Pin == GPIO_PIN_15) {
		EmergencyState = 0;
	}

	static uint8_t state[3] = { 0 }; // 0 = waiting for rise, 1 = waiting for fall

	uint32_t now = DWT->CYCCNT;
	uint32_t idx;

	if (GPIO_Pin == GPIO_PIN_0)
		idx = 0; // PC0
	else if (GPIO_Pin == GPIO_PIN_2)
		idx = 1; // PC2
	else if (GPIO_Pin == GPIO_PIN_3)
		idx = 2; // PC3
	else
		return;

	if (state[idx] == 0) {
		rise_time[idx] = now;
		state[idx] = 1;
	} else {
		uint32_t delta =
				(now >= rise_time[idx]) ?
						(now - rise_time[idx]) :
						(0xFFFFFFFF - rise_time[idx] + now);
		pulse_width_us[idx] = delta / (SystemCoreClock / 1000000);
		state[idx] = 0;
	}
}

// MicroSecondTimer Implement
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += UINT32_MAX;
	}
}
uint64_t micros() {
	return __HAL_TIM_GET_COUNTER(&htim5) + _micros;
}

float PID_Update(float error, float kP, float kI, float kD, float dt,
		float outMin, float outMax, PID_State *state) {
	/* --- Proportional -------------------------------------- */
	float Pout = kP * error;

	/* --- Integral (with anti‑windup clamp) ----------------- */
	state->integ += error * dt;
	if (state->integ > outMax / kI)
		state->integ = outMax / kI;
	if (state->integ < outMin / kI)
		state->integ = outMin / kI;
	if (error > 0 && state->integ < 0)
		state->integ = 0;
	if (error < 0 && state->integ > 0)
		state->integ = 0;
	float Iout = kI * state->integ;

	/* --- Derivative (on error) ----------------------------- */
	float deriv = (error - state->prevError) / dt;
	float Dout = kD * deriv;
	state->prevError = error;

	/* --- Sum and clamp ------------------------------------- */
	float out = Pout + Iout + Dout;
	if (out > outMax)
		out = outMax;
	if (out < outMin)
		out = outMin;

	return out;
}

void Revolute_PosVel_Update() {
// Collect data
	Revolute_QEIdata.TimeStamp[NEW] = micros();
	Revolute_QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim4);

// Position within one turn
	Revolute_QEIdata.QEIPostion_1turn = Revolute_QEIdata.Position[NEW] % 16384;

// Calculate position difference
	int32_t diffPosition_r = Revolute_QEIdata.Position[NEW]
			- Revolute_QEIdata.Position[OLD];

// Handle wrap-around
	if (diffPosition_r > (65536 / 2))
		diffPosition_r -= 65536;
	else if (diffPosition_r < -(65536 / 2))
		diffPosition_r += 65536;

// Time difference in seconds
	float diffTime_r = (Revolute_QEIdata.TimeStamp[NEW]
			- Revolute_QEIdata.TimeStamp[OLD]) * 0.000001f;
	if (diffTime_r == 0)
		return;

// Raw angular velocity in counts/sec
	float Vel_counts_r = (float) diffPosition_r / diffTime_r;

// Raw angular acceleration in counts/sec²
	Revolute_QEIdata.QEIAcceleration = (Vel_counts_r
			- Revolute_QEIdata.QEIVelocity) / diffTime_r;

// Store raw velocity
	Revolute_QEIdata.QEIVelocity = Vel_counts_r;

// Angular velocity in rad/s
	Revolute_QEIdata.Velocity = Vel_counts_r * (2.0f * M_PI / 16384.0f);
	float R_alpha = 0.4f;
	Revolute_QEIdata.Velocity_f = R_alpha * Revolute_QEIdata.Velocity
			+ (1 - R_alpha) * Revolute_QEIdata.Velocity_f;

// Angular acceleration in rad/s²
	Revolute_QEIdata.Acceleration = Revolute_QEIdata.QEIAcceleration
			* (2.0f * M_PI / 16384.0f);

// Absolute position update
	Revolute_QEIdata.AbsolutePosition += diffPosition_r;

// Rad position
	Revolute_QEIdata.RadPosition = Revolute_QEIdata.AbsolutePosition
			* (2.0f * M_PI / 16384.0f);

// Deg position
	Revolute_QEIdata.DegPosition = Revolute_QEIdata.RadPosition * 180 / M_PI;

// Store previous values
	Revolute_QEIdata.Position[OLD] = Revolute_QEIdata.Position[NEW];
	Revolute_QEIdata.TimeStamp[OLD] = Revolute_QEIdata.TimeStamp[NEW];
}

void Prismatic_PosVel_Update() {
// Collect data
	Prismatic_QEIdata.TimeStamp[NEW] = micros();
	Prismatic_QEIdata.Position[NEW] = __HAL_TIM_GET_COUNTER(&htim3);

// Calculate position difference
	int32_t diffPosition_p = Prismatic_QEIdata.Position[NEW]
			- Prismatic_QEIdata.Position[OLD];

// Handle wrap-around
	if (diffPosition_p > (65536 / 2))
		diffPosition_p -= 65536;
	else if (diffPosition_p < -(65536 / 2))
		diffPosition_p += 65536;

// Time difference in seconds
	float diffTime_p = (Prismatic_QEIdata.TimeStamp[NEW]
			- Prismatic_QEIdata.TimeStamp[OLD]) * 0.000001f;
	if (diffTime_p == 0)
		return;

// Raw angular velocity in counts/sec
	float Vel_counts_p = (float) diffPosition_p / diffTime_p;

// Raw angular acceleration in counts/sec²
	Prismatic_QEIdata.QEIAcceleration = (Vel_counts_p
			- Prismatic_QEIdata.QEIVelocity) / diffTime_p;

// Store raw velocity
	Prismatic_QEIdata.QEIVelocity = Vel_counts_p;

// Velocity in mm/s
	Prismatic_QEIdata.Velocity = Vel_counts_p * (10.0f / 8192.0f);
	float P_alpha = 0.4f;
	Prismatic_QEIdata.Velocity_f = P_alpha * Prismatic_QEIdata.Velocity
			+ (1 - P_alpha) * Prismatic_QEIdata.Velocity_f;

// Acceleration in mm/s²
	Prismatic_QEIdata.Acceleration = Prismatic_QEIdata.QEIAcceleration
			* (10.0f / 8192.0f);

// Absolute position update
	Prismatic_QEIdata.AbsolutePosition += diffPosition_p;

// mm position
	Prismatic_QEIdata.mmPosition = Prismatic_QEIdata.AbsolutePosition
			* (10.0f / 8192.0f);

// Store previous values
	Prismatic_QEIdata.Position[OLD] = Prismatic_QEIdata.Position[NEW];
	Prismatic_QEIdata.TimeStamp[OLD] = Prismatic_QEIdata.TimeStamp[NEW];
}

void Set_Motor(int motor_num, float speed) {
	if (speed > 100.0f)
		speed = 100.0f;
	if (speed < -100.0f)
		speed = -100.0f;
	uint32_t pwm_value = (uint32_t) ((fabsf(speed) * 100) / 100);
	if (motor_num == 0) {
		if (speed > 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		} else if (speed < 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm_value);
	} else if (motor_num == 1) {
		if (speed > 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		} else if (speed < 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		}
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm_value);
	}
}

void Set_Servo(int Pen_Pos) {
	if (Pen_Pos == 0) {
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 500);
		Pen_Status_in = 0;
		servo_timer = micros();
	} else {
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 1000);
		Pen_Status_in = 1;
	}
}

void Reset_R() {
	Revolute_QEIdata.AbsolutePosition = M_PI_2 / (2.0f * M_PI / 16384.0f);
//	Revolute_QEIdata.RadPosition = M_PI_2;
}
void Reset_P() {
	Prismatic_QEIdata.AbsolutePosition = -2.00 / (10.0f / 8192.0f);
//	Prismatic_QEIdata.mmPosition = 0;
}

void Workspace_limit() {
	if (Revolute_QEIdata.RadPosition < -1.91986 && R_PWM > 0) {
		R_PWM = 0;
	}
	if (Revolute_QEIdata.RadPosition > 5.06145 && R_PWM < 0) {
		R_PWM = 0;
	}
	if (Prismatic_QEIdata.mmPosition > 305 && P_PWM > 0) {
		P_PWM = 0;
	}
	if (Prismatic_QEIdata.mmPosition < -1 && P_PWM < 0) {
		P_PWM = 0;
	}
}

void Get_QRIdata(float *prism_vel_mm, float *prism_acc_mm, float *prism_mm_pos,
		float *rev_ang_vel_rad, float *rev_ang_acc_rad, float *rev_rad_pos) {
	if (prism_vel_mm)
		*prism_vel_mm = Prismatic_QEIdata.Velocity;
	if (prism_acc_mm)
		*prism_acc_mm = Prismatic_QEIdata.Acceleration;
	if (prism_mm_pos)
		*prism_mm_pos = Prismatic_QEIdata.mmPosition;
	if (rev_ang_vel_rad)
		*rev_ang_vel_rad = Revolute_QEIdata.Velocity;
	if (rev_ang_acc_rad)
		*rev_ang_acc_rad = Revolute_QEIdata.Acceleration;
	if (rev_rad_pos)
		*rev_rad_pos = Revolute_QEIdata.RadPosition;
}

float Trapezoidal_CalcTotalTime(float distance, float vmax, float amax) {
	float t_acc = vmax / amax;
	float d_acc = 0.5f * amax * t_acc * t_acc;
	if (2 * d_acc > distance) {
		t_acc = sqrtf(distance / amax);
		return 2 * t_acc;
	}
	float d_const = distance - 2 * d_acc;
	float t_const = d_const / vmax;
	return 2 * t_acc + t_const;
}

float Trapezoidal_CalcVmaxFromTime(float distance, float amax, float total_time) {
	float t_half = total_time / 2.0f;
	float d_half = distance / 2.0f;

	float v_peak = amax * t_half;
	if (0.5f * v_peak * t_half >= d_half) {
		return sqrtf(distance * amax);
	}
	return (distance - 0.5f * amax * t_half * t_half) / t_half;
}

void InverseKinematics(float x, float y, float *r, float *p) {
	*r = atan2f(x * -1, y) + M_PI_2;
	*p = sqrtf(x * x + y * y);
}

void TrapezoidStep(void) {
//	static float last_TargetR = 0.0f;
//	static float last_TargetP = 0.0f;

// 2a) Detect setpoint jump (revolute, in radians)
	float r_diff = fabsf(TargetR - last_TargetR);
	if (r_diff > 0.001f) {
		// Re‐init revolute trapezoid: distance_to_go = R_Pos_Error (rad)
		Trapezoidal_Init(&revolute, R_Pos_Error, /*maxVel*/1.40f, /*maxAcc*/
		9.0f);
		last_TargetR = TargetR;
	}

// 2b) Detect setpoint jump (prismatic, in mm)
	float p_diff = fabsf(TargetP - last_TargetP);
	if (p_diff > 0.01f) {
		// Re‐init prismatic trapezoid: distance_to_go = P_Pos_Error (mm)
		Trapezoidal_Init(&prismatic, P_Pos_Error, /*maxVel*/600.0f, /*maxAcc*/
		3000.0f);
		last_TargetP = TargetP;
	}

// 2c) Advance both trapezoids by 1 ms → update feedforward pos/vel/acc
	Trapezoidal_Update(&revolute, 0.001f);
	TargetRPos = revolute.current_position;
	TargetRVel = revolute.current_velocity;
	TargetRAcc = revolute.current_acceleration;

	Trapezoidal_Update(&prismatic, 0.001f);
	TargetPPos = prismatic.current_position;
	TargetPVel = prismatic.current_velocity;
	TargetPAcc = prismatic.current_acceleration;
}

void PIDStep(void) {
	static int loop_counter1 = 0;
	static float R_Target_Velocity = 0.0f;
	static float P_Target_Velocity = 0.0f;

	loop_counter1++;
// 3a) Outer‐loop (position) PID every 10 ms
	if (loop_counter1 >= 10) {
		loop_counter1 = 0;

		// Recompute “true” pos‐errors
		float R_Pos_now = TargetR - Revolute_QEIdata.RadPosition;
		float P_Pos_now = TargetP - Prismatic_QEIdata.mmPosition;

		// Position‐PID → corrective velocity for revolute
		float R_corr_vel = PID_Update(R_Pos_now, R_kP_pos, R_kI_pos, R_kD_pos,
				0.010f, // dt = 10 ms
				-100.0f, +100.0f, &pid_r);

		// Position‐PID → corrective velocity for prismatic
		float P_corr_vel = PID_Update(P_Pos_now, P_kP_pos, P_kI_pos, P_kD_pos,
				0.010f, -100.0f, +100.0f, &pid_p);

		// Combine with feedforward velocities
		R_Target_Velocity = TargetRVel + R_corr_vel;
		P_Target_Velocity = TargetPVel + P_corr_vel;
	}

// 3b) Inner‐loop (velocity) PID _every_ 1 ms:
	R_Velo_Error = R_Target_Velocity - Revolute_QEIdata.Velocity_f;
	R_PWM = PID_Update(R_Velo_Error, R_kP_vel, R_kI_vel, R_kD_vel, 0.001f, // dt = 1 ms
			-100.0f, +100.0f, &pid_r_v);

	P_Velo_Error = P_Target_Velocity - Prismatic_QEIdata.Velocity_f;
	P_PWM = PID_Update(P_Velo_Error, P_kP_vel, P_kI_vel, P_kD_vel, 0.001f,
			-100.0f, +100.0f, &pid_p_v);
}

int ToleranceCheck(void) {
	static uint64_t lock_timer_us = 0;

	if ((fabsf(TargetR - Revolute_QEIdata.RadPosition) < R_ERR_TOL_RAD)
			&& (fabsf(TargetP - Prismatic_QEIdata.mmPosition) < P_ERR_TOL_MM)) {
		if (lock_timer_us == 0) {
			lock_timer_us = micros();
		} else if ((micros() - lock_timer_us) >= HOLD_TIME_US) {
			// We have stayed inside tolerance for long enough → “lock & hold”
			return 1;
		}
	} else {
		lock_timer_us = 0;
	}

	return 0;
}

int CascadeControl_Step(void) {
	static uint64_t timestampState2 = 0;

// 1a) Convert desired‐angle (deg) → (rad) and compute current pos‐errors
// float TargetR = TargetR_Deg * (M_PI / 180.0f);
	R_Pos_Error = TargetR - Revolute_QEIdata.RadPosition;
	P_Pos_Error = TargetP - Prismatic_QEIdata.mmPosition;
	R_Pos_Error_Deg = R_Pos_Error * (180.0f / M_PI);

// 1b) 1 ms timer check
	uint64_t nowtimestamp = micros();
	if (nowtimestamp <= timestampState2) {
		// Not yet 1 ms since last run → bail out
		return 0;
	}
// Advance to next 1 ms tick
	timestampState2 = nowtimestamp + 1000;

// 2) Trapezoid logic: init if needed + update (1 ms)
	TrapezoidStep();

// 3) PID logic: 10 ms outer, 1 ms inner
	PIDStep();

// 4) Apply workspace limits (joint‐limits, etc.) and send the PWM commands
	Workspace_limit();
	Set_Motor(0, R_PWM);
	Set_Motor(1, P_PWM);

// 5) Tolerance‐check + “lock & hold” (servo + zero motors) if arrived
	CheckTolerance = ToleranceCheck();
	return CheckTolerance;
//	return ToleranceCheck();
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
