/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Bluetooth Motor Control
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* New pins for the L298N Motor Driver */
#define IN1_PORT        GPIOB
#define IN1_PIN         GPIO_PIN_5
#define IN2_PORT        GPIOB
#define IN2_PIN         GPIO_PIN_4

/* Macroes for easy to read timer-names */
#define MOTOR_TIMER     htim1
#define SERVO_TIMER     htim2
#define ENCODER_TIMER   htim5

/* Servo pulse constants (microseconds at 1 MHz timer tick) */
#define SERVO_PULSE_MIN       600   // Full left
#define SERVO_PULSE_CENTER    1100   // Straight
#define SERVO_PULSE_MAX       1500   // Full right

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;
uint8_t rx_data = 0;

/* Bluetooth line buffer */
#define RX_BUFFER_SIZE 32
char rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
volatile uint8_t command_ready = 0;

/* Trapezoidal motion profile variables */
float time_ctl = 0.0f;   // Elapsed motion time in seconds
float Sn = 70.0f;  // Target cruising speed (% duty cycle)
float t1 = 1.0f;   // Acceleration ramp duration (seconds)
float t2 = 0.0f;   // Start of deceleration ramp (seconds)
float T = 0.0f;   // Total motion duration (seconds)
float SP_Vel = 0.0f;   // Calculated setpoint speed (0 - 100%)
int profile_state = 0;      // 0 = Idle, 1 = Accel, 2 = Cruise, 3 = Decel

uint32_t last_profile_tick = 0; // Timestamp for 10ms loop

float target_distance_meters = 0.0f;

/* Information for the cm/s */
volatile int position = 0;
int32_t last_pos = 0;
float current_speed_cm_s = 0.0f;
float distance_meters = 0.0f;
const float movement = 1.0f / 1560.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Calculates timings and starts a trapezoidal motion profile
 * @param dist_m Target distance to drive in meters
 * @param max_speed_pct Cruising speed in percent (e.g., 70.0f)
 * @param ramp_time Ramp-up/down duration in seconds (e.g., 1.0f)
 */
void Start_Motion_Profile(float dist_m, float max_speed_pct, float ramp_time) {
	/* Reset wheel encoder counter to 0 */
	__HAL_TIM_SET_COUNTER(&ENCODER_TIMER, 0);
	target_distance_meters = dist_m;

	Sn = max_speed_pct;
	t1 = ramp_time;

	/* Approximate speed in m/s at nominal duty cycle (calibrate if needed) */
	float speed_mps = (Sn / 100.0f) * 0.8f;

	/* Minimum distance required to reach full speed and brake */
	float min_dist = speed_mps * t1;

	if (dist_m < min_dist) {
		/* Distance too short for cruise phase: triangular profile */
		t1 = sqrtf(dist_m / (speed_mps / t1));
		t2 = t1;
		T = 2.0f * t1;
	} else {
		/* Trapezoidal profile: ramp up -> cruise -> ramp down */
		float t_cruise = (dist_m - min_dist) / speed_mps;
		t2 = t1 + t_cruise;
		T = t2 + t1;
	}

	time_ctl = 0.0f;
	profile_state = 1;

	/* Set motor direction forward */
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
}

void Set_Speed_cm_s(float target_cm_s) {
	if (target_cm_s <= 0.0f) {
		__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, 0);
		return;
	}
	// Clamp to estimated max speed
	if (target_cm_s > 80.0f)
		target_cm_s = 80.0f;

	// Map target_cm_s (0 - 80 cm/s) to actual duty cycle (52% - 90%)
	float duty_pct = 52.0f + (target_cm_s / 80.0f) * 38.0f;
	uint32_t compare_val = (uint32_t) (duty_pct * 144.0f);

	__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, compare_val);
}
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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	/* Ensure motor is stopped on startup */
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);

	/* Start PWM for the motor (Using TIM1_CH1 for ENA med 14400 perioden) */
	HAL_TIM_PWM_Start(&MOTOR_TIMER, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, 0); // 0% speed start

	/* Start servo PWM (TIM2 Channel 3) and center the steering */
	HAL_TIM_PWM_Start(&SERVO_TIMER, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_3, SERVO_PULSE_CENTER);

	/* Start hardware-encoder in the background and restart the counter*/
	HAL_TIM_Encoder_Start(&ENCODER_TIMER, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&ENCODER_TIMER, 0);

	/* START BLUETOOTH INTERRUPT LISTENER */
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* Read wheel distance from TIM5 encoder */
		position = (int32_t) __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);

		/* Handle negative counting direction safely */
		if (position < 0) {
			distance_meters = (float) (-position) * movement;
		} else {
			distance_meters = (float) position * movement;
		}

		/* 10 ms periodic loop (100 Hz) */
		if (HAL_GetTick() - last_profile_tick >= 10) {
			last_profile_tick = HAL_GetTick();

			/* Compute real-time speed in cm/s */
			int32_t current_pos = position;
			int32_t delta_ticks = current_pos - last_pos;
			last_pos = current_pos;

			/* 1560 ticks = 1 m = 100 cm -> scale by 100 / 1560 over 0.010s */
			current_speed_cm_s = fabsf(
					((float) delta_ticks * (100.0f / 1560.0f)) / 0.010f);

			/* Update motion profile every 10 ms (100 Hz loop) */
			if (profile_state != 0) {
				time_ctl += 0.010f; // 10 ms time step

				/* Primary stop condition: Target distance reached */
				if (distance_meters >= target_distance_meters) {
					profile_state = 0;
					SP_Vel = 0.0f;
					HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
					__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, 0);
				} else {
					switch (profile_state) {
					case 1: /* Area 1: Acceleration ramp up */
						SP_Vel = (time_ctl * Sn) / t1;
						if (time_ctl >= t1)
							profile_state = 2;
						break;

					case 2: /* Area 2: Constant cruising speed */
						SP_Vel = Sn;
						if (time_ctl >= t2)
							profile_state = 3;
						break;

					case 3: /* Area 3: Deceleration ramp down */
						SP_Vel = Sn * (T - time_ctl) / (T - t2);
						/* Keep minimum crawl speed to avoid stalling before reaching target */
						if (SP_Vel < 15.0f)
							SP_Vel = 15.0f;
						break;

					default:
						profile_state = 0;
						break;
					}

					/* Apply PWM with deadband floor compensation */
					uint32_t compare_val = 0;
					if (profile_state != 0 && SP_Vel > 5.0f) {
						/* Maps SP_Vel (0..100%) to actual duty cycle (52%..90%) */
						float actual_pct = 52.0f + (SP_Vel / 100.0f) * 38.0f;
						compare_val = (uint32_t) (actual_pct * 144.0f);
					}

					if (compare_val > 14400)
						compare_val = 14400;
					__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1,
							compare_val);
				}
			}
		} /* End of 10 ms profile update block */

		/* Process commands received via Bluetooth interrupt */
		if (command_ready) {
			command_ready = 0;

			/* Distance command: e.g. "d1.2" or "D2.5" */
			if (rx_buffer[0] == 'd' || rx_buffer[0] == 'D') {
				float desired_D = atof(&rx_buffer[1]);
				if (desired_D > 0.0f) {
					Start_Motion_Profile(desired_D, 70.0f, 1.0f);
				}
			} else if (rx_buffer[0] == 'v' || rx_buffer[0] == 'V') {
				/* Velocity command in cm/s: e.g., "v40" */
				float target_v = atof(&rx_buffer[1]);
				profile_state = 0;
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				Set_Speed_cm_s(target_v);
			}
			/* Manual drive commands */
			else if (rx_buffer[0] == 'f') /* Forward */
			{
				profile_state = 0;
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, 60 * 144);
			} else if (rx_buffer[0] == 'b') /* Backward */
			{
				profile_state = 0;
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, 60 * 144);
			} else if (rx_buffer[0] == 's') /* Stop */
			{
				profile_state = 0;
				SP_Vel = 0.0f;
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, 0);
			} else if (rx_buffer[0] == 'l') /* Steer left */
			{
				__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_3,
						SERVO_PULSE_MIN);
			} else if (rx_buffer[0] == 'c') /* Steer center */
			{
				__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_3,
						SERVO_PULSE_CENTER);
			} else if (rx_buffer[0] == 'r') /* Steer right */
			{
				__HAL_TIM_SET_COMPARE(&SERVO_TIMER, TIM_CHANNEL_3,
						SERVO_PULSE_MAX);
			} else if (rx_buffer[0] >= '1' && rx_buffer[0] <= '6') /* Manual speed */
			{
				profile_state = 0;
				int pct = (rx_buffer[0] - '0' + 4) * 10;
				__HAL_TIM_SET_COMPARE(&MOTOR_TIMER, TIM_CHANNEL_1, pct * 144);
			}

			rx_index = 0; /* Reset buffer index */
		}

		/* Clear UART overrun flag to prevent receiver freeze */
		__HAL_UART_CLEAR_OREFLAG(&huart1);

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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 14399;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
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
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 6;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 6;
	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Motor_IN1_Pin | Motor_IN2_Pin | Sonar_TRIG_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Sonar_ECHO_Pin */
	GPIO_InitStruct.Pin = Sonar_ECHO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Sonar_ECHO_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Startknap_Pin */
	GPIO_InitStruct.Pin = Startknap_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Startknap_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Motor_IN1_Pin Motor_IN2_Pin Sonar_TRIG_Pin */
	GPIO_InitStruct.Pin = Motor_IN1_Pin | Motor_IN2_Pin | Sonar_TRIG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		/* DIAGNOSTIC: Toggle the on-board green LED on ANY byte received */
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		if (rx_data == '\r' || rx_data == '\n') {
			if (rx_index > 0) {
				rx_buffer[rx_index] = '\0';
				command_ready = 1;
			}
		} else {
			if (rx_index < (RX_BUFFER_SIZE - 1)) {
				rx_buffer[rx_index++] = (char) rx_data;
			} else {
				rx_index = 0;
			}
		}

		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		/* Clear error flags */
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);

		/* Restart listening */
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
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
