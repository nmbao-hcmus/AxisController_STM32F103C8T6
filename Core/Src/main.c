/* USER CODE BEGIN Header */
/*
 Code STM32 Dieu Khien 3 truc cua thay Trung

 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Axis_v2.h"
#include "uart_handler.h"
#include "gcode_parser.h"
#include <string.h> // strstr
#include <stdlib.h> // atof
#include <stdio.h> // Required for sprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Defines the main states of the machine
typedef enum {
	MACHINE_STATE_IDLE = 0, // Idle state, ready for any command
	MACHINE_STATE_BUSY = 1  // Busy state, executing a movement command
} MachineState_t;

// Defines identifiers for each button for debugging purposes
typedef enum {
	BUTTON_NONE,
	BUTTON_A,
	BUTTON_B,
	BUTTON_C,
	BUTTON_D,
	BUTTON_Z_UP,
	BUTTON_Z_DOWN,
	BUTTON_L,
	BUTTON_R,
	BUTTON_SWITCH_STOP,
} ButtonPressed_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Global variable for the machine's current state
volatile MachineState_t g_machineState = MACHINE_STATE_IDLE;

// ================== DEBUG VARIABLES ==================
// You can monitor these in the "Live Expressions" window of the IDE
int32_t dbg_pos_x = 0;
int32_t dbg_pos_y = 0;
int32_t dbg_pos_z = 0;
uint8_t dbg_x_is_busy = 0;
uint8_t dbg_y_is_busy = 0;
uint8_t dbg_z_is_busy = 0;
uint8_t dbg_code = 0;  // A general-purpose code for tracking execution flow

//Tracking Button Pressed
volatile ButtonPressed_t g_lastButtonPressed = BUTTON_NONE;

// ================== JOYSTICK VARIABLES ==================
uint16_t rawX, rawY;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ================== AXIS DECLARATIONS ==================
Axis_t axisX, axisY, axisZ;
// ================== PRESET POSITIONS ==================
typedef struct {
	int32_t x, y, z;
} Position_t;

Position_t posA = { 1000, 2000, -9999 };
Position_t posB = { 2000, 1000, -9999 };
Position_t posC = { 1500, 1500, -9999 };
Position_t posD = { 0, 0, -9999 };

uint32_t freq_FreeMoving = 800; // Frequency to move around
uint32_t freq_Z = 20000; // Frequency Z
// ================== HELP FUNCTION ==================
/**
 * @brief  Reads the state of a button (assumes active low).
 * @param  port: GPIO port of the button.
 * @param  pin: GPIO pin of the button.
 * @retval 1 if pressed, 0 if not pressed.
 */
static inline uint8_t ReadButton(GPIO_TypeDef *port, uint16_t pin) {
	return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

/**
 * @brief  Plans and starts a move to a preset position.
 * @param  pos: Pointer to the target position struct.
 */
void Axis_GotoPosition(Position_t *pos) {
	Axis_RunToPosConst(&axisX, pos->x);
	Axis_RunToPosConst(&axisY, pos->y);
	int32_t pos_z_temp = axisZ.current_pos_pulse;
	// Special value -9999 means "do not move Z axis"
	if (pos->z == -9999) {
		Axis_RunToPosConst(&axisZ, pos_z_temp);
	} else {
		Axis_RunToPosConst(&axisZ, pos->z);
	}
	Axis_StartAxisConst(&axisX);
	Axis_StartAxisConst(&axisY);
	Axis_StartAxisConst(&axisZ);
}

// ================== JOYSTICK SUPPORT ==================
#define JOY_DEADZONE 400
#define JOY_MAX_VALUE 4095
#define JOY_CENTER    (JOY_MAX_VALUE/2 )

/**
 * @brief  Reads a single ADC channel using polling mode.
 * @param  channel: The ADC channel to read (e.g., ADC_CHANNEL_0).
 * @retval The 12-bit raw ADC value.
 */
uint16_t ADC_ReadChannel(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return value;
}

/**
 * @brief  Scales the raw ADC value for the X-axis to a -100 to 100 range.
 * @param  raw: The raw 12-bit ADC value.
 * @retval Scaled value from -100 to 100.
 */
int16_t Joystick_ScaleX(uint16_t raw) {
	int32_t diff = (int32_t) raw - JOY_CENTER;

	if (diff > -JOY_DEADZONE && diff < JOY_DEADZONE)
		return 0;

	int16_t scaled = (diff * 100) / (JOY_CENTER - JOY_DEADZONE);
	if (scaled > 100)
		scaled = 100;
	if (scaled < -100)
		scaled = -100;
	return scaled;
}

/**
 * @brief  Scales the raw ADC value for the Y-axis to a -100 to 100 range.
 * @param  raw: The raw 12-bit ADC value.
 * @retval Scaled value from -100 to 100.
 */
int16_t Joystick_ScaleY(uint16_t raw) {
	int32_t diff = (int32_t) raw - JOY_CENTER;

	if (diff > -JOY_DEADZONE && diff < JOY_DEADZONE)
		return 0;

	int16_t scaled = (diff * 100) / (JOY_CENTER - JOY_DEADZONE);
	if (scaled > 100)
		scaled = 100;
	if (scaled < -100)
		scaled = -100;
	return scaled;
}

/**
 * @brief  Processes joystick input to move the X and Y axes.
 * @param  rawX: The raw 12-bit ADC value for the X-axis.
 * @param  rawY: The raw 12-bit ADC value for the Y-axis.
 */
void HandleJoystick(uint16_t rawX, uint16_t rawY) {
	int16_t joyX_scaled = Joystick_ScaleX(rawX);
	int16_t joyY_scaled = Joystick_ScaleY(rawY);

	// Handle X-axis movement
	if (joyX_scaled != 0) {
		if (!Axis_IsBusy(&axisX)) {
			// Move a small distance. The direction depends on the joystick sign.
			int32_t dist = (joyX_scaled > 0) ? -100 : 100; // Inverted due to axis setup
			Axis_RunPosConst(&axisX, dist);
			Axis_StartAxisConst(&axisX);
		}
	}

	// Handle Y-axis movement
	if (joyY_scaled != 0) {
		if (!Axis_IsBusy(&axisY)) {
			int32_t dist = (joyY_scaled > 0) ? 100 : -100;
			Axis_RunPosConst(&axisY, dist);
			Axis_StartAxisConst(&axisY);
		}
	}
}

/**
 * @brief  Processes input from manual control buttons (A,B,C,D,Z, L, R, Stop).
 */
void HandleManualButtons(void) {
	if (ReadButton(A_BUTTON_GPIO_Port, A_BUTTON_Pin))
		Axis_GotoPosition(&posA);
	if (ReadButton(B_BUTTON_GPIO_Port, B_BUTTON_Pin))
		Axis_GotoPosition(&posB);
	if (ReadButton(C_BUTTON_GPIO_Port, C_BUTTON_Pin))
		Axis_GotoPosition(&posC);
	if (ReadButton(D_BUTTON_GPIO_Port, D_BUTTON_Pin))
		Axis_GotoPosition(&posD);

	// Z-axis up/down
	if (ReadButton(ZUP_GPIO_Port, ZUP_Pin)) {
		g_lastButtonPressed = BUTTON_Z_UP;
		if (!Axis_IsBusy(&axisZ)) {
			Axis_RunPosConst(&axisZ, +50);
			Axis_StartAxisConst(&axisZ);
		}
	} else if (ReadButton(ZDOWN_GPIO_Port, ZDOWN_Pin)) {
		g_lastButtonPressed = BUTTON_Z_DOWN;
		if (!Axis_IsBusy(&axisZ)) {
			Axis_RunPosConst(&axisZ, -50);
			Axis_StartAxisConst(&axisZ);
		}
	}

	// X-axis left/right
	if (ReadButton(L_BUTTON_GPIO_Port, L_BUTTON_Pin)) {
		g_lastButtonPressed = BUTTON_L;
		if (!Axis_IsBusy(&axisX)) {
			Axis_RunPosConst(&axisX, -100);
			Axis_StartAxisConst(&axisX);
		}
	} else if (ReadButton(R_BUTTON_GPIO_Port, R_BUTTON_Pin)) {
		g_lastButtonPressed = BUTTON_R;
		if (!Axis_IsBusy(&axisX)) {
			Axis_RunPosConst(&axisX, +100);
			Axis_StartAxisConst(&axisX);
		}
	}

	// Stop button
	if (ReadButton(SWITCH_BUTTON_GPIO_Port, SWITCH_BUTTON_Pin)) {
		g_lastButtonPressed = BUTTON_SWITCH_STOP;
		Axis_Stop(&axisX);
		Axis_Stop(&axisY);
		Axis_Stop(&axisZ);
		g_machineState = MACHINE_STATE_IDLE;
	}
}

/**
 * @brief  Parses and executes a "SET" command to update preset positions.
 * @param  line: The command line received from UART.
 * @retval true if the line was a SET command, false otherwise.
 */
bool Parse_SetCommand(const char *line) {
	if (strncmp(line, "SET", 3) != 0) {
		return false;
	}

	Position_t *target_pos = NULL;
	const char *ptr = line + 4; // Skip "SET "

	if (*ptr == 'A')
		target_pos = &posA;
	else if (*ptr == 'B')
		target_pos = &posB;
	else if (*ptr == 'C')
		target_pos = &posC;
	else if (*ptr == 'D')
		target_pos = &posD;
	else {
		UART_Transmit("Error: Invalid position name.\r\n");
		return true; // It was a SET command, but it failed.
	}

	char *coord_ptr;
	if ((coord_ptr = strstr(ptr, "X"))) {
		target_pos->x = (float) (atof(coord_ptr + 1) * axisX.pulse_per_mm);
	}
	if ((coord_ptr = strstr(ptr, "Y"))) {
		target_pos->y = (float) (atof(coord_ptr + 1) * axisY.pulse_per_mm);
	}
	if ((coord_ptr = strstr(ptr, "Z"))) {
		target_pos->z = (float) (atof(coord_ptr + 1) * axisZ.pulse_per_mm);
	}

	UART_Transmit("Position set OK\r\n");
	return true;
}

/**
 * @brief  Parses and executes a "GOTO" command to Move to a point.
 * @param  line: The command line received from UART.
 * @retval true if the line was a GOTO command, false otherwise.
 */
bool Parse_GotoCommand(const char *line) {
	if (strncmp(line, "GOTO", 4) != 0) {
		return false;
	}
	const char *ptr = line + 5; // Skip "GOTO "
	if (*ptr == 'A')
		Axis_GotoPosition(&posA);
	else if (*ptr == 'B')
		Axis_GotoPosition(&posB);
	else if (*ptr == 'C')
		Axis_GotoPosition(&posC);
	else if (*ptr == 'D')
		Axis_GotoPosition(&posD);
	else {
		UART_Transmit("Error: Invalid Go To Position.\r\n");
		return true; // It was a Go To command, but it failed.
	}
	UART_Transmit("Go TO Position OK\r\n");
	return true;
}

/**
 * @brief  Formats and sends the current machine status (state and position) over UART.
 * This mimics the real-time DRO (Digital Read-Out) of systems like Mach3/GRBL.
 */
void Send_StatusReport(void) {
	// A buffer to hold the formatted status string. 128 bytes is safe.
	char status_buffer[128];

	// Convert the current pulse positions to millimeters (float)
	float pos_x_mm = Axis_GetCurrentPositionMM(&axisX);
	float pos_y_mm = Axis_GetCurrentPositionMM(&axisY);
	float pos_z_mm = Axis_GetCurrentPositionMM(&axisZ);

	// Determine the current state string based on the state machine
	const char *state_str = "Idle";
	if (g_machineState == MACHINE_STATE_BUSY) {
		state_str = "Run";
	}
	// Format the string into a GRBL-like status report: <State|WPos:X,Y,Z>
	// %.3f formats the float to 3 decimal places.
	sprintf(status_buffer, "<%s|WPos:%.3f,%.3f,%.3f>\r\n", state_str, pos_x_mm,
			pos_y_mm, pos_z_mm);

	// Transmit the final string
	UART_Transmit(status_buffer);
}

// ================== CALLBACK ==================
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	Axis_TIM_PWM_PulseFinishedCallback_Const(htim);

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		UART_Receive_Callback();
	}
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
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	UART_Init(&huart1);
	// cấu hình trục
	axisX.GPIOx_Dir = DirX_GPIO_Port;
	axisX.GPIO_Pin_Dir = DirX_Pin;
	axisX.htim_Pul = &htim2;
	axisX.Channel_Pul = TIM_CHANNEL_2;
	axisX.pulse_per_mm = 20000;
	axisX.invert_dir = 1;
	axisX.frequency_max = 20000;
	axisX.frequency_min = 100;

	axisY.GPIOx_Dir = DirY_GPIO_Port;
	axisY.GPIO_Pin_Dir = DirY_Pin;
	axisY.htim_Pul = &htim2;
	axisY.Channel_Pul = TIM_CHANNEL_1;
	axisY.pulse_per_mm = 20000;
	axisY.invert_dir = 1;
	axisY.frequency_max = 20000;
	axisY.frequency_min = 100;

	axisZ.GPIOx_Dir = DirZ_GPIO_Port;
	axisZ.GPIO_Pin_Dir = DirZ_Pin;
	axisZ.htim_Pul = &htim3;
	axisZ.Channel_Pul = TIM_CHANNEL_1;
	axisZ.pulse_per_mm = 12800;
	axisZ.invert_dir = 1;
	axisZ.frequency_max = 20000;
	axisZ.frequency_min = 100;

//      // Axis_Init(&axis_struct, DIR_Port, DIR_Pin, TIM_Handle, TIM_Channel, freq_min, freq_max, pulse_per_mm, invert_dir);
//      Axis_Init(&axisX, axisX.irX_GPIO_Port, DirX_Pin, &htim2, TIM_CHANNEL_2, 100, 20000, 20000, 0);
//      Axis_Init(&axisY, DirY_GPIO_Port, DirY_Pin, &htim2, TIM_CHANNEL_1, 100, 20000, 20000, 0);
//      Axis_Init(&axisZ, DirZ_GPIO_Port, DirZ_Pin, &htim3, TIM_CHANNEL_1, 100, 10000, 12800, 0);
	// Set default running frequency for all axes
	Axis_SetFrequencyHz(&axisX, 2800);
	Axis_SetFrequencyHz(&axisY, 2800);
	Axis_SetFrequencyHz(&axisZ, 20000);

	// *** REGISTER THE AXES! ***
	Axis_Register(&axisX);
	Axis_Register(&axisY);
	Axis_Register(&axisZ);

	// Timer for periodic status reporting
	static uint32_t last_status_report_tick = 0;
	// Timer for periodic joystick reading
	static uint32_t last_joystick_read_tick = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint32_t current_tick = HAL_GetTick();
		g_lastButtonPressed = BUTTON_NONE;
		switch (g_machineState) {

		case MACHINE_STATE_IDLE:
			// Priority 1: Check for commands from UART
			if (UART_IsNewLineReady()) {
				char command_line[RX_BUFFER_SIZE];
				UART_GetLine(command_line);

				// Check for special "SET" command first
				if (Parse_SetCommand(command_line)) {
					// Command was handled, do nothing else.
				} else if (Parse_GotoCommand(command_line)) {
					// Check for special "GOTO" command first

				}
				// Otherwise, treat it as a G-code command
				else {
					GCode_Command_t cmd;
					if (GCode_ParseLine(command_line, &cmd)) {
						GCode_ExecuteCommand(&cmd, &axisX, &axisY, &axisZ);
						g_machineState = MACHINE_STATE_BUSY; // Switch to busy state
					}
				}
			}
			// Priority 2: Handle manual controls if no UART command
			else {
				// Read joystick periodically (every 100ms) to reduce CPU load
				if (current_tick - last_joystick_read_tick >= 100) {
					rawX = ADC_ReadChannel(ADC_CHANNEL_0);
					rawY = ADC_ReadChannel(ADC_CHANNEL_1);
					HandleJoystick(rawX, rawY);
					last_joystick_read_tick = current_tick;
				}

				// Buttons can be checked every loop cycle
				HandleManualButtons();
			}
			break;

		case MACHINE_STATE_BUSY:
			// While busy, only check if all movements are complete
			if (!Axis_IsBusy(&axisX) && !Axis_IsBusy(&axisY)
					&& !Axis_IsBusy(&axisZ)) {
				g_machineState = MACHINE_STATE_IDLE; // Return to idle state
			}
			break;
		}

		// Update debug variables at the end of each loop
		dbg_pos_x = Axis_GetCurrentPositionPulse(&axisX);
		dbg_pos_y = Axis_GetCurrentPositionPulse(&axisY);
		dbg_pos_z = Axis_GetCurrentPositionPulse(&axisZ);
		dbg_x_is_busy = Axis_IsBusy(&axisX);
		dbg_y_is_busy = Axis_IsBusy(&axisY);
		dbg_z_is_busy = Axis_IsBusy(&axisZ);
		// --- Periodic Status Reporting (like Mach3 DRO) ---
		// It sends a position update every 200 milliseconds.
		if (current_tick - last_status_report_tick >= 200) {
			Send_StatusReport();
			last_status_report_tick = current_tick;
		}
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	htim2.Init.Prescaler = 200 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	sConfigOC.Pulse = 8 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 200 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 8;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DirX_Pin | DirY_Pin | DirZ_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : HOME_BUTTON_Pin SWITCH_BUTTON_Pin L_BUTTON_Pin R_BUTTON_Pin
	 ZDOWN_Pin ZUP_Pin */
	GPIO_InitStruct.Pin = HOME_BUTTON_Pin | SWITCH_BUTTON_Pin | L_BUTTON_Pin
			| R_BUTTON_Pin | ZDOWN_Pin | ZUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : D_BUTTON_Pin C_BUTTON_Pin B_BUTTON_Pin A_BUTTON_Pin */
	GPIO_InitStruct.Pin = D_BUTTON_Pin | C_BUTTON_Pin | B_BUTTON_Pin
			| A_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : EndY_Pin EndX_Pin */
	GPIO_InitStruct.Pin = EndY_Pin | EndX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : DirX_Pin DirY_Pin DirZ_Pin */
	GPIO_InitStruct.Pin = DirX_Pin | DirY_Pin | DirZ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
