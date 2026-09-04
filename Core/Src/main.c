/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "tmc2209.h"
#include "mt6816.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TIM1: 100 MHz / (99+1) / (999+1) = 1 kHz control loop */
#define CONTROL_LOOP_HZ     1000.0f
#define CONTROL_LOOP_DT     (1.0f / CONTROL_LOOP_HZ)

/* TIM2 counts at 100 MHz / (99+1) = 1 MHz, so ARR = 1e6/step_freq - 1 */
#define STEP_TIMER_HZ       1000000U
#define STEP_FREQ_MAX       7500U    /* Upper step rate the mechanics tolerate */
#define STEP_FREQ_MIN       10U      /* Below this the motor is treated as stopped */
#define STEP_FREQ_HOMING    2000U    /* Open-loop speed while seeking a limit switch */

/* Motion tolerances */
/* Smallest error the step generator can still act on: below STEP_FREQ_MIN it
 * emits no pulses, so with output = Kp * error that floor sits at
 * STEP_FREQ_MIN / Kp = 10 / 15 = 0.67 deg. The deadband is set just above it
 * and the arrival test just above that, so a move always finishes inside the
 * band the controller has actually parked in. */
#define PID_DEADBAND_DEG    0.70f
#define POS_TOLERANCE_DEG   0.75f    /* Considered "arrived" within this error */
/* A move's allowance has to scale with its length: at STEP_FREQ_MAX the axis
 * covers roughly 844 deg/s, so a flat timeout would fail long feeds purely for
 * being long. The fixed part absorbs acceleration and settling. */
#define MOVE_TIMEOUT_BASE_MS 3000U
#define MOVE_DEG_PER_SEC     844.0f
#define MOVE_TIMEOUT_MS(deg) (MOVE_TIMEOUT_BASE_MS + (uint32_t)((deg) * (3000.0f / MOVE_DEG_PER_SEC)))
#define HOMING_TIMEOUT_MS   15000U   /* Guards against a dead or miswired switch */
#define DRIVER_WAKE_MS      2U       /* TMC2209 settle time after its EN goes low */

/* --- Blade geometry -------------------------------------------------------
 * PLACEHOLDER VALUES - these must be measured on the real machine before the
 * blade is allowed near a wire. Zero is the top (home) position and the blade
 * travels DOWN in the positive direction.
 *
 * Stripping only cuts through the insulation, so its depth is deliberately
 * shallower than a full cut. See TUNING.md for the measuring procedure.        */
#define BLADE_CUT_DEG       180.0f   /* Depth that severs the conductor */
#define BLADE_STRIP_DEG     120.0f   /* Depth that scores insulation only */
#define BLADE_RETRACT_DEG   0.0f     /* Fully retracted = the homed origin */
#define BLADE_SETTLE_MS     150U     /* Dwell at depth before retracting */

/* Wire pull-back after the head strip, so the jaws grip fresh insulation. */
#define STRIP_PULL_MM       8.0f

/* DIR polarity for a positive (increasing-position) move. Both axes share one
 * DIR line, so each names its own sense rather than assuming a common one.
 *   Rollers: HIGH turns counter-clockwise, pulling the wire in.
 *   Blade:   HIGH drives the blade down (positive = deeper). */
#define DIR_ROLLER_FORWARD  GPIO_PIN_SET
#define DIR_BLADE_DOWN      GPIO_PIN_SET
#define DIR_OPPOSITE(d)     (((d) == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET)

/* Motor selection: the rollers share one STEP/DIR pair and are enabled
 * together, while the blade is driven alone on the same pair. */
typedef enum {
    DRIVE_NONE = 0,
    DRIVE_ROLLERS,   /* M1 + M2 enabled together, both follow PA0/PA1 */
    DRIVE_BLADE      /* M3 alone */
} DriveTarget_t;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum {
    STATE_IDLE,
    STATE_HOMING_BLADE,  /* Blade retracts to its top stop and defines zero */
    STATE_FEEDING,       /* Closed-loop wire advance */
    STATE_STRIP_HEAD,    /* Score the insulation near the leading end */
    STATE_STRIP_TAIL,    /* Score the insulation at the trailing end */
    STATE_CUTTING,       /* Sever the conductor */
    STATE_ERROR
} SystemState_t;

volatile SystemState_t current_state = STATE_IDLE;

/* Job Tracking */
volatile int job_qty = 0;
volatile int current_piece = 0;
volatile float job_length_mm = 0.0f;
volatile float target_angle_deg = 0.0f;
volatile float current_pos_deg = 0.0f;
/* Shaft rotation per millimetre of wire, from the roller's circumference:
 *   360 / (pi * 30 mm) = 3.8197 deg/mm
 * With a NEMA 17 (1.8 deg/step) at 1/16 microstepping the feed resolves to
 * 0.029 mm per microstep, and the MT6816 reads back 0.006 mm per count, so
 * the encoder is finer than the step the motor can take.
 * Re-measure this if the rollers are ever changed - see TUNING.md. */
const float DEG_PER_MM = 3.8197f;

/* Hardware Objects
 * motor_1 / motor_2 are the front and rear feed rollers. They share the STEP
 * and DIR lines and are enabled together, so only the front roller carries an
 * encoder - the rear one mirrors its motion.
 * motor_3 drives the blade and has its own encoder on CS2 (PA3). */
TMC2209_t motor_1, motor_2, motor_3;
MT6816_t encoder_roller, encoder_blade;
PID_Controller_t pid_roller, pid_blade;

volatile DriveTarget_t active_target = DRIVE_NONE;
volatile MT6816_Status encoder_status = MT6816_OK;
volatile bool step_pwm_running = false;
volatile uint32_t encoder_error_count = 0;

/* Blade position, tracked continuously by the same 1 kHz loop. */
volatile float blade_target_deg = 0.0f;
volatile float blade_pos_deg = 0.0f;

/* ESP32 UART Buffer */
uint8_t rx_byte;
char rx_buffer[128];
volatile uint8_t rx_index = 0;
volatile bool msg_received = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Control_Loop_1kHz(void);
static void Step_SetFrequency(uint32_t step_freq);
static void Step_Stop(void);
void Select_Drive_Target(DriveTarget_t target);
static bool Wait_For_Move(volatile float *target_deg, volatile float *pos_deg);
static bool Blade_MoveTo(float depth_deg);
static bool Home_Blade(void);
static bool Feed_Wire_mm(float distance_mm);
static bool Blade_Stroke(float depth_deg);
static void Report_State(const char *name);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Send_ESP32_Msg(const char *msg) {
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                msg_received = true;
                rx_index = 0;
            }
        } else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = rx_byte;
        }
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
    }
}

void Process_ESP32_Message(void) {
    if (strstr(rx_buffer, "\"command\":\"abort\"")) {
        current_state = STATE_IDLE;
        Step_Stop();
        Select_Drive_Target(DRIVE_NONE);
        Send_ESP32_Msg("{\"state\":\"IDLE\"}\n");
    }
    else if (strstr(rx_buffer, "\"quantity\":")) {
        char *ptr_qty = strstr(rx_buffer, "\"quantity\":");
        char *ptr_len = strstr(rx_buffer, "\"total_length\":");
        if (ptr_qty) job_qty = atoi(ptr_qty + 11);
        if (ptr_len) job_length_mm = atof(ptr_len + 15);
        current_piece = 0;
        /* Home the blade before the rollers so it is clear of the wire path. */
        current_state = STATE_HOMING_BLADE;
    }
    msg_received = false;
}

/* Routes the shared STEP/DIR pair to one group of drivers.
 *
 * Both rollers are enabled at once so they receive the identical pulse train
 * and stay mechanically in sync - the wire would stretch or buckle if only one
 * of them turned. The blade is enabled alone because it must move independently
 * of the feed. Enabling everything at once is never valid. */
void Select_Drive_Target(DriveTarget_t target) {
    if (active_target == target) return;

    /* Park the step generator before rerouting it, so no driver picks up a
     * pulse that was meant for another axis. */
    Step_Stop();

    /* Disable all drivers by driving EN pins HIGH (they are active LOW) */
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET);

    switch (target) {
        case DRIVE_ROLLERS:
            HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET);
            break;
        case DRIVE_BLADE:
            HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_RESET);
            break;
        case DRIVE_NONE:
        default:
            break;
    }

    active_target = target;

    /* A TMC2209 needs a moment after its enable line drops before it will
     * track step pulses reliably; stepping immediately can lose position. */
    if (target != DRIVE_NONE) HAL_Delay(DRIVER_WAKE_MS);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 1. Encoders: CS1 (PA4) reads the front feed roller, CS2 (PA3) the blade. */
  MT6816_Init(&encoder_roller, &hspi1, CS1_GPIO_Port, CS1_Pin);
  MT6816_Init(&encoder_blade,  &hspi1, CS2_GPIO_Port, CS2_Pin);

  /* 2. Configure all three TMC2209 drivers over the shared single-wire UART.
   * Each has its own node address set by its MS1/MS2 strapping. */
  TMC2209_Init(&motor_1, &huart1, 0);
  TMC2209_Init(&motor_2, &huart1, 1);
  TMC2209_Init(&motor_3, &huart1, 2);

  /* 3. Initialize PID controllers.
   *
   * The deadband must cover every error the step generator cannot act on.
   * Below STEP_FREQ_MIN no pulses are emitted, so any error smaller than
   * STEP_FREQ_MIN/Kp is unactuatable - treating it as zero stops the
   * integrator from charging against an error the motor can never clear. */
  PID_Init(&pid_roller, 15.0f, 0.5f, 0.08f, -20000.0f, 20000.0f);
  PID_Init(&pid_blade,  15.0f, 0.5f, 0.08f, -20000.0f, 20000.0f);
  pid_roller.deadband = PID_DEADBAND_DEG;
  pid_blade.deadband  = PID_DEADBAND_DEG;

  /* 4. The rollers only ever turn - there is no home position to seek - so
   * wherever the shaft sits at power-on becomes zero and every feed is
   * measured relative to it. Seeding the tracker here means the very first
   * feed starts from a known reference instead of a stale one. */
  {
      float raw_deg = 0.0f;
      if (MT6816_ReadDegrees(&encoder_roller, &raw_deg) == MT6816_OK) {
          pid_roller.last_raw_angle_deg    = raw_deg;
          pid_roller.accumulated_angle_deg = 0.0f;
          pid_roller.is_initialized        = true;
          pid_roller.prev_feedback         = 0.0f;
      }
  }

  /* 5. Start 1 kHz Control Loop Timer and UART RX */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (msg_received) {
	            Process_ESP32_Message();
	        }

	        switch (current_state) {
	            case STATE_IDLE:
	                break;

	            /* Blade is homed first: it must be clear of the wire path
	             * before any wire is pushed through. */
	            case STATE_HOMING_BLADE:
	                if (Home_Blade()) {
	                    Send_ESP32_Msg("{\"state\":\"RUNNING\",\"pg\":0}\n");
	                    current_state = STATE_FEEDING;
	                }
	                break;

	            /* One piece: feed -> strip head -> feed -> strip tail -> cut. */
	            case STATE_FEEDING:
	                if (current_piece >= job_qty) {
	                    Send_ESP32_Msg("{\"done\":true}\n");
	                    current_state = STATE_IDLE;
	                    Select_Drive_Target(DRIVE_NONE);
	                    break;
	                }

	                /* Present the leading end to the blade. */
	                if (!Feed_Wire_mm(STRIP_PULL_MM)) break;
	                current_state = STATE_STRIP_HEAD;
	                break;

	            case STATE_STRIP_HEAD:
	                Report_State("STRIP_HEAD");
	                if (!Blade_Stroke(BLADE_STRIP_DEG)) break;

	                /* Advance the full piece length before the tail strip. */
	                if (!Feed_Wire_mm(job_length_mm)) break;
	                current_state = STATE_STRIP_TAIL;
	                break;

	            case STATE_STRIP_TAIL:
	                Report_State("STRIP_TAIL");
	                if (!Blade_Stroke(BLADE_STRIP_DEG)) break;
	                current_state = STATE_CUTTING;
	                break;

	            case STATE_CUTTING:
	                Report_State("CUTTING");
	                if (!Blade_Stroke(BLADE_CUT_DEG)) break;

	                current_piece++;
	                char pg_msg[32];
	                snprintf(pg_msg, sizeof(pg_msg), "{\"pg\":%d}\n", current_piece);
	                Send_ESP32_Msg(pg_msg);

	                current_state = STATE_FEEDING;
	                break;

	            case STATE_ERROR:
	                Step_Stop();
	                Select_Drive_Target(DRIVE_NONE);
	                Send_ESP32_Msg("{\"error\":\"motion fault - check wire path and limit switches\"}\n");
	                current_state = STATE_IDLE;
	                break;
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* --- Step generator helpers (TIM2 CH1) ------------------------------------ */

/* Sets the STEP pulse train frequency. The PWM peripheral is only started once
 * and then re-tuned in place; restarting it every loop would truncate pulses. */
static void Step_SetFrequency(uint32_t step_freq)
{
    if (step_freq > STEP_FREQ_MAX) step_freq = STEP_FREQ_MAX;
    if (step_freq < STEP_FREQ_MIN) step_freq = STEP_FREQ_MIN;

    uint32_t arr_val = (STEP_TIMER_HZ / step_freq) - 1U;
    if (arr_val > 65535U) arr_val = 65535U;
    if (arr_val < 2U)     arr_val = 2U;

    TIM2->ARR  = arr_val;
    TIM2->CCR1 = arr_val / 2U;   /* 50 % duty */

    /* Force the new period to take effect on the next cycle rather than
     * waiting out the previously loaded (possibly very long) one. */
    if (TIM2->CNT > arr_val) TIM2->CNT = 0;

    if (!step_pwm_running) {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        step_pwm_running = true;
    }
}

static void Step_Stop(void)
{
    if (step_pwm_running) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        step_pwm_running = false;
    }
}

/* --- 1 kHz closed-loop control -------------------------------------------- */

/* Runs once per TIM1 update event (1 kHz). Reads the encoder belonging to
 * whichever axis currently owns the STEP/DIR pair, updates its PID controller
 * and converts the result into a step frequency.
 *
 * Only one axis is ever live: the rollers move together as one unit, or the
 * blade moves alone. The axis that is not selected has its driver disabled and
 * simply holds position. */
void Control_Loop_1kHz(void)
{
    MT6816_t         *encoder;
    PID_Controller_t *pid;
    volatile float   *pos_out;
    float             setpoint;
    GPIO_PinState     dir_positive;

    switch (active_target) {
        case DRIVE_ROLLERS:
            encoder      = &encoder_roller;
            pid          = &pid_roller;
            pos_out      = &current_pos_deg;
            setpoint     = target_angle_deg;
            dir_positive = DIR_ROLLER_FORWARD;
            break;

        case DRIVE_BLADE:
            encoder      = &encoder_blade;
            pid          = &pid_blade;
            pos_out      = &blade_pos_deg;
            setpoint     = blade_target_deg;
            dir_positive = DIR_BLADE_DOWN;
            break;

        default:
            /* Nothing selected (idle, or homing runs open-loop). */
            return;
    }

    float raw_deg = 0.0f;
    encoder_status = MT6816_ReadDegrees(encoder, &raw_deg);

    if (encoder_status != MT6816_OK) {
        /* Never keep stepping blind: a bad read means the position is unknown. */
        Step_Stop();
        if (++encoder_error_count > 50) {   /* 50 ms of continuous failure */
            current_state = STATE_ERROR;
        }
        return;
    }
    encoder_error_count = 0;

    *pos_out = PID_UnwrapAngle(pid, raw_deg);
    float step_velocity = PID_Update(pid, setpoint, *pos_out, CONTROL_LOOP_DT);

    uint32_t step_freq = (uint32_t)fabsf(step_velocity);

    /* Deadband stop: below the minimum step rate the motor just holds. */
    if (step_freq < STEP_FREQ_MIN) {
        Step_Stop();
        return;
    }

    /* A positive output means "increase position", which is forward for the
     * rollers and downward for the blade. */
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,
                      (step_velocity > 0.0f) ? dir_positive
                                             : DIR_OPPOSITE(dir_positive));

    Step_SetFrequency(step_freq);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        Control_Loop_1kHz();
    }
}

/* --- Motion sequencing helpers -------------------------------------------- */

static void Report_State(const char *name)
{
    char msg[48];
    snprintf(msg, sizeof(msg), "{\"state\":\"%s\"}\n", name);
    Send_ESP32_Msg(msg);
}

/* Blocks until the closed-loop axis settles on its target, an abort arrives,
 * or the move overruns. Returns false if the move did not complete. */
static bool Wait_For_Move(volatile float *target_deg, volatile float *pos_deg)
{
    uint32_t start_time = HAL_GetTick();
    uint32_t timeout_ms = MOVE_TIMEOUT_MS(fabsf(*target_deg - *pos_deg));

    while (fabsf(*target_deg - *pos_deg) > POS_TOLERANCE_DEG) {
        if (msg_received) {
            Process_ESP32_Message();
            if (current_state == STATE_IDLE) return false;   /* aborted */
        }
        if (current_state == STATE_ERROR) return false;      /* raised by the ISR */
        if (HAL_GetTick() - start_time > timeout_ms) {
            current_state = STATE_ERROR;
            return false;
        }
    }
    return true;
}

/* Drives the blade to an absolute depth measured from the homed top position. */
static bool Blade_MoveTo(float depth_deg)
{
    Select_Drive_Target(DRIVE_BLADE);
    blade_target_deg = depth_deg;
    PID_SetTarget(&pid_blade, depth_deg);
    return Wait_For_Move(&blade_target_deg, &blade_pos_deg);
}

/* Retracts the blade open-loop until it presses its top limit switch (PA10),
 * then declares that point to be zero. Homing must run open-loop because the
 * absolute position is unknown until the switch is found.
 *
 * This is the only homing the machine does: the feed rollers have no reference
 * position, they simply turn, and their count is relative to power-on. */
static bool Home_Blade(void)
{
    Report_State("HOMING_BLADE");
    Select_Drive_Target(DRIVE_BLADE);

    /* Retracting is the opposite of the blade's downward direction. */
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, DIR_OPPOSITE(DIR_BLADE_DOWN));
    Step_SetFrequency(STEP_FREQ_HOMING);

    uint32_t start_time = HAL_GetTick();
    while (HAL_GPIO_ReadPin(Blade_Limit_GPIO_Port, Blade_Limit_Pin) == GPIO_PIN_SET) {
        if (msg_received) {
            Process_ESP32_Message();
            if (current_state == STATE_IDLE) { Step_Stop(); return false; }
        }
        /* A switch that never closes would otherwise drive the blade into its
         * end stop forever. */
        if (HAL_GetTick() - start_time > HOMING_TIMEOUT_MS) {
            Step_Stop();
            current_state = STATE_ERROR;
            return false;
        }
    }
    Step_Stop();
    HAL_Delay(200);   /* Let the mechanism stop ringing before sampling */

    float raw_deg = 0.0f;
    if (MT6816_ReadDegrees(&encoder_blade, &raw_deg) != MT6816_OK) {
        current_state = STATE_ERROR;
        return false;
    }

    /* Define this point as zero without disturbing the multi-turn tracking. */
    PID_Reset(&pid_blade);
    pid_blade.last_raw_angle_deg   = raw_deg;
    pid_blade.accumulated_angle_deg = 0.0f;
    pid_blade.is_initialized       = true;
    pid_blade.prev_feedback        = 0.0f;

    blade_pos_deg    = 0.0f;
    blade_target_deg = 0.0f;
    encoder_error_count = 0;
    return true;
}

/* Feeds the wire a relative distance in millimetres, closed-loop. */
static bool Feed_Wire_mm(float distance_mm)
{
    Select_Drive_Target(DRIVE_ROLLERS);
    target_angle_deg += (distance_mm * DEG_PER_MM);
    PID_SetTarget(&pid_roller, target_angle_deg);
    return Wait_For_Move(&target_angle_deg, &current_pos_deg);
}

/* Plunges the blade to a depth, dwells, then retracts to the home position. */
static bool Blade_Stroke(float depth_deg)
{
    if (!Blade_MoveTo(depth_deg)) return false;
    HAL_Delay(BLADE_SETTLE_MS);
    if (!Blade_MoveTo(BLADE_RETRACT_DEG)) return false;
    return true;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
