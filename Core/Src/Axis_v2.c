/*
 * Axis_v2.c
 *
 *  Created on: Oct 6, 2025
 *      Author: DELL
 */
#include "Axis_v2.h"
#include <string.h>

#ifndef AXIS_MAX_INSTANCES
#define AXIS_MAX_INSTANCES 8
#endif

static Axis_t *s_axis_list[AXIS_MAX_INSTANCES] = { 0 };

// Helper: xác định timer thuộc APB2 (để nhân x2 clock khi prescaler != 1)
static uint8_t Axis_IsAPB2Timer(TIM_TypeDef *TIMx) {
#ifdef TIM1
	if (TIMx == TIM1)
		return 1;
#endif
#ifdef TIM8
  if (TIMx == TIM8) return 1;
#endif
#ifdef TIM9
  if (TIMx == TIM9) return 1;
#endif
#ifdef TIM10
  if (TIMx == TIM10) return 1;
#endif
#ifdef TIM11
  if (TIMx == TIM11) return 1;
#endif
#ifdef TIM15
  if (TIMx == TIM15) return 1;
#endif
#ifdef TIM16
  if (TIMx == TIM16) return 1;
#endif
#ifdef TIM17
  if (TIMx == TIM17) return 1;
#endif
	return 0;
}

// Lấy clock timer (đã tính nhân 2 nếu APB prescaler != 1)
static uint32_t Axis_GetTimerClock(TIM_HandleTypeDef *htim) {
	if (!htim)
		return 0;

	RCC_ClkInitTypeDef clk;
	uint32_t flashLatency;
	HAL_RCC_GetClockConfig(&clk, &flashLatency);

	uint8_t is_apb2 = Axis_IsAPB2Timer(htim->Instance) ? 1 : 0;
	uint32_t pclk = is_apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();

	if (is_apb2) {
#if defined(RCC_CFGR_PPRE2_DIV1)
		if (clk.APB2CLKDivider != RCC_HCLK_DIV1)
			pclk *= 2U;
#else
    // fallback
    pclk *= 2U;
#endif
	} else {
#if defined(RCC_CFGR_PPRE1_DIV1)
		if (clk.APB1CLKDivider != RCC_HCLK_DIV1)
			pclk *= 2U;
#else
    // fallback
    pclk *= 2U;
#endif
	}
	return pclk;
}

// Map TIM_CHANNEL_x -> HAL_TIM_ACTIVE_CHANNEL_x
static uint32_t Axis_ChannelToActive(uint32_t Channel) {
	switch (Channel) {
	case TIM_CHANNEL_1:
		return HAL_TIM_ACTIVE_CHANNEL_1;
	case TIM_CHANNEL_2:
		return HAL_TIM_ACTIVE_CHANNEL_2;
	case TIM_CHANNEL_3:
		return HAL_TIM_ACTIVE_CHANNEL_3;
	case TIM_CHANNEL_4:
		return HAL_TIM_ACTIVE_CHANNEL_4;
#if defined(TIM_CHANNEL_5) && defined(HAL_TIM_ACTIVE_CHANNEL_5)
    case TIM_CHANNEL_5: return HAL_TIM_ACTIVE_CHANNEL_5;
#endif
	default:
		return 0xFFFFFFFFu;
	}
}

// Thiết lập PSC, ARR, CCR để có PWM tần số freq_hz và duty 50%
// Lưu ý: thay đổi PSC/ARR ảnh hưởng cả các kênh cùng TIM
static HAL_StatusTypeDef Axis_ApplyFrequency(Axis_t *ax, uint32_t freq_hz) {
	if (!ax || !ax->htim_Pul || freq_hz == 0)
		return HAL_ERROR;

	uint32_t tim_clk = Axis_GetTimerClock(ax->htim_Pul);
	if (tim_clk == 0)
		return HAL_ERROR;

	// Giới hạn theo min/max
	if (ax->frequency_min > 0 && freq_hz < ax->frequency_min)
		freq_hz = ax->frequency_min;
	if (ax->frequency_max > 0 && freq_hz > ax->frequency_max)
		freq_hz = ax->frequency_max;

	// Mục tiêu: tim_clk / (PSC * ARR) ~= freq_hz  (với ARR = AutoReload+1)
	// Giới hạn ARR tối đa: 65535 cho đa số timer (trừ TIM2/TIM5 32-bit)
	uint32_t max_arr = 0xFFFFu;
#ifdef TIM2
	if (ax->htim_Pul->Instance == TIM2)
		max_arr = 0xFFFFFFFFu; // 32-bit (F1/F4)
#endif
#ifdef TIM5
  if (ax->htim_Pul->Instance == TIM5) max_arr = 0xFFFFFFFFu; // 32-bit (F4)
#endif

	// Tính prescaler "ceil" để ARR không vượt quá max_arr
	uint64_t denom = (uint64_t) freq_hz * (uint64_t) max_arr;
	uint32_t presc =
			(denom == 0) ? 1 : (uint32_t) ((tim_clk + denom - 1ULL) / denom); // ceil
	if (presc < 1)
		presc = 1;
	if (presc > 0x10000u)
		presc = 0x10000u; // PSC register = presc-1 => max 0xFFFF

	uint64_t arr_calc = (uint64_t) tim_clk
			/ ((uint64_t) presc * (uint64_t) freq_hz);
	if (arr_calc < 1)
		arr_calc = 1;
	uint32_t arr_reg = (uint32_t) (arr_calc - 1ULL);

	// Cập nhật thanh ghi
	// Lưu ý: PSC register = presc - 1
	__HAL_TIM_DISABLE(ax->htim_Pul);
	__HAL_TIM_SET_PRESCALER(ax->htim_Pul, (uint16_t )(presc - 1U));
	__HAL_TIM_SET_AUTORELOAD(ax->htim_Pul, arr_reg);

////   Duty 50%
//  uint32_t ccr = (arr_reg + 1U) / 2U;
//  __HAL_TIM_SET_COMPARE(ax->htim_Pul, ax->Channel_Pul, ccr);

	//Duty 89%
	uint64_t period_ticks = (uint64_t) arr_reg + 1U;
	uint32_t ccr = (uint32_t) ((period_ticks * 89) / 100);
	__HAL_TIM_SET_COMPARE(ax->htim_Pul, ax->Channel_Pul, ccr);

	__HAL_TIM_SET_COUNTER(ax->htim_Pul, 0U);
	__HAL_TIM_ENABLE(ax->htim_Pul);

	ax->run_frequency = (uint32_t) ((uint64_t) tim_clk
			/ ((uint64_t) presc * (uint64_t) (arr_reg + 1U)));
	return HAL_OK;
}

// ============ Public API ============

void Axis_Init(Axis_t *ax, GPIO_TypeDef *GPIOx_Dir, uint16_t GPIO_Pin_Dir,
		TIM_HandleTypeDef *htim, uint32_t Channel_Pul, uint32_t frequency_min,
		uint32_t frequency_max, uint32_t pulse_per_mm, uint8_t invert_dir) {
	if (!ax)
		return;
	memset(ax, 0, sizeof(*ax));
	ax->GPIOx_Dir = GPIOx_Dir;
	ax->GPIO_Pin_Dir = GPIO_Pin_Dir;
	ax->htim_Pul = htim;
	ax->Channel_Pul = Channel_Pul;
	ax->frequency_min = frequency_min;
	ax->frequency_max = frequency_max;
	ax->pulse_per_mm = pulse_per_mm;
	ax->invert_dir = invert_dir;
	ax->state = AXIS_STATE_IDLE;
	ax->run_frequency = (frequency_min > 0) ? frequency_min : 1000U; // default
}

HAL_StatusTypeDef Axis_Register(Axis_t *ax) {
	if (!ax)
		return HAL_ERROR;
	if (ax->registered)
		return HAL_OK;
	for (uint32_t i = 0; i < AXIS_MAX_INSTANCES; ++i) {
		if (s_axis_list[i] == NULL) {
			s_axis_list[i] = ax;
			ax->registered = 1;
			return HAL_OK;
		}
	}
	return HAL_BUSY;
}

void Axis_Unregister(Axis_t *ax) {
	if (!ax || !ax->registered)
		return;
	for (uint32_t i = 0; i < AXIS_MAX_INSTANCES; ++i) {
		if (s_axis_list[i] == ax) {
			s_axis_list[i] = NULL;
			ax->registered = 0;
			break;
		}
	}
}

HAL_StatusTypeDef Axis_SetFrequencyHz(Axis_t *ax, uint32_t freq_hz) {
	if (!ax)
		return HAL_ERROR;
	return Axis_ApplyFrequency(ax, freq_hz);
}

HAL_StatusTypeDef AxisCalculatePulseToGo(Axis_t *ax, int32_t target_pos_pulse) {
	if (!ax)
		return HAL_ERROR;
	if (ax->state == AXIS_STATE_RUNNING_CONST)
		return HAL_BUSY;

	ax->target_pos_pulse = target_pos_pulse;
	int32_t delta = ax->target_pos_pulse - ax->current_pos_pulse;

	if (delta == 0) {
		ax->pulses_to_go = 0;
		ax->pulses_done = 0;
		ax->dir_sign = 0;
		return HAL_OK;
	}

	ax->dir_sign = (delta > 0) ? +1 : -1;
	ax->pulses_to_go = (delta > 0) ? (uint32_t) delta : (uint32_t) (-delta);
	ax->pulses_done = 0;

	return HAL_OK;
}

HAL_StatusTypeDef Axis_RunPosConst(Axis_t *ax, int32_t pos_relative_pulse) {
	if (!ax)
		return HAL_ERROR;
	if (ax->state == AXIS_STATE_RUNNING_CONST)
		return HAL_BUSY;

	int32_t target = ax->current_pos_pulse + pos_relative_pulse;
	return AxisCalculatePulseToGo(ax, target);
}

HAL_StatusTypeDef Axis_RunToPosConst(Axis_t *ax, int32_t pos_target_pulse) {
	return AxisCalculatePulseToGo(ax, pos_target_pulse);
}
/**
 * @brief  Lập kế hoạch di chuyển tương đối và cài đặt tần số cho lần di chuyển đó.
 * @note   Hàm này chỉ lập kế hoạch, cần gọi Axis_StartAxisConst() để bắt đầu chạy.
 */
HAL_StatusTypeDef Axis_RunPosConstWithFreq(Axis_t *ax,
		int32_t pos_relative_pulse, uint32_t freq_hz) {
	if (!ax)
		return HAL_ERROR;
	if (ax->state == AXIS_STATE_RUNNING_CONST)
		return HAL_BUSY;

	// 1. Cài đặt tần số mong muốn cho lần chạy này
	// Hàm Axis_SetFrequencyHz sẽ tự động giới hạn tần số theo min/max của trục
	HAL_StatusTypeDef status = Axis_SetFrequencyHz(ax, freq_hz);
	if (status != HAL_OK) {
		return status;
	}

	// 2. Lập kế hoạch di chuyển như bình thường
	return Axis_RunPosConst(ax, pos_relative_pulse);
}

/**
 * @brief  Lập kế hoạch di chuyển tuyệt đối và cài đặt tần số cho lần di chuyển đó.
 * @note   Hàm này chỉ lập kế hoạch, cần gọi Axis_StartAxisConst() để bắt đầu chạy.
 */
HAL_StatusTypeDef Axis_RunToPosConstWithFreq(Axis_t *ax,
		int32_t pos_target_pulse, uint32_t freq_hz) {
	if (!ax)
		return HAL_ERROR;
	if (ax->state == AXIS_STATE_RUNNING_CONST)
		return HAL_BUSY;

	// 1. Cài đặt tần số mong muốn cho lần chạy này
	HAL_StatusTypeDef status = Axis_SetFrequencyHz(ax, freq_hz);
	if (status != HAL_OK) {
		return status;
	}

	// 2. Lập kế hoạch di chuyển như bình thường
	return Axis_RunToPosConst(ax, pos_target_pulse);
}

HAL_StatusTypeDef Axis_StartAxisConst(Axis_t *ax) {
	if (!ax || !ax->htim_Pul)
		return HAL_ERROR;

	if (ax->state == AXIS_STATE_RUNNING_CONST)
		return HAL_BUSY;

	if (ax->pulses_to_go == 0) {
		// Không có gì để chạy
		return HAL_OK;
	}

	// Chốt tần số (nếu chưa set thì áp min)
	if (ax->run_frequency == 0)
		ax->run_frequency = (ax->frequency_min > 0) ? ax->frequency_min : 1000U;
	if (Axis_ApplyFrequency(ax, ax->run_frequency) != HAL_OK) {
		ax->state = AXIS_STATE_ERROR;
		return HAL_ERROR;
	}

	// Set DIR theo dir_sign và invert_dir
	if (ax->GPIOx_Dir) {
		GPIO_PinState dir_state = GPIO_PIN_RESET;
		if (ax->dir_sign > 0)
			dir_state = GPIO_PIN_SET; // mặc định: + chiều => SET
		if (ax->invert_dir)
			dir_state =
					(dir_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
		HAL_GPIO_WritePin(ax->GPIOx_Dir, ax->GPIO_Pin_Dir, dir_state);
	}

	// Optional: bật EN nếu có
	if (ax->GPIOx_En) {
		GPIO_PinState en_state = ax->invert_en ? GPIO_PIN_RESET : GPIO_PIN_SET;
		HAL_GPIO_WritePin(ax->GPIOx_En, ax->GPIO_Pin_En, en_state);
	}

	__HAL_TIM_SET_COUNTER(ax->htim_Pul, 0U);

	// Start PWM + interrupt
	HAL_StatusTypeDef st = HAL_TIM_PWM_Start_IT(ax->htim_Pul, ax->Channel_Pul);
	if (st != HAL_OK) {
		ax->state = AXIS_STATE_ERROR;
		return st;
	}

	ax->state = AXIS_STATE_RUNNING_CONST;
	return HAL_OK;
}

void Axis_Stop(Axis_t *ax) {
	if (!ax || !ax->htim_Pul)
		return;

	// Stop PWM interrupt
	HAL_TIM_PWM_Stop_IT(ax->htim_Pul, ax->Channel_Pul);

	// Optional: tắt EN nếu có (tuỳ nhu cầu)
	// if (ax->GPIOx_En) {
	//   GPIO_PinState en_off = ax->invert_en ? GPIO_PIN_SET : GPIO_PIN_RESET;
	//   HAL_GPIO_WritePin(ax->GPIOx_En, ax->GPIO_Pin_En, en_off);
	// }

	if (ax->state != AXIS_STATE_ERROR) {
		ax->state = AXIS_STATE_IDLE;
	}
}

void Axis_EStop(Axis_t *ax) {
	if (!ax)
		return;
	ax->state = AXIS_STATE_ERROR;
	Axis_Stop(ax);
}

// Gọi trong HAL_TIM_PWM_PulseFinishedCallback
void Axis_TIM_PWM_PulseFinishedCallback_Const(TIM_HandleTypeDef *htim) {
	if (!htim)
		return;

	uint32_t active_ch = htim->Channel; // HAL_TIM_ACTIVE_CHANNEL_x

	// Lặp qua registry
	for (uint32_t i = 0; i < AXIS_MAX_INSTANCES; ++i) {
		Axis_t *ax = s_axis_list[i];
		if (!ax)
			continue;
		if (ax->htim_Pul != htim)
			continue;
		if (ax->state != AXIS_STATE_RUNNING_CONST)
			continue;

		// Kiểm tra kênh
		uint32_t expect = Axis_ChannelToActive(ax->Channel_Pul);
		if (expect == 0xFFFFFFFFu)
			continue;
		if (active_ch != expect)
			continue;

		// 1 xung đã hoàn thành
		ax->pulses_done++;
		ax->current_pos_pulse += (int32_t) ax->dir_sign;

		if (ax->pulses_done >= ax->pulses_to_go) {
			// Hoàn tất
			Axis_Stop(ax);
			// Đảm bảo vị trí chốt đúng target khi chạy đủ
			ax->current_pos_pulse = ax->target_pos_pulse;
		}
	}
}

// ============ Helpers theo mm ============

static int32_t Axis_mm_to_pulse(const Axis_t *ax, float mm) {
	if (!ax || ax->pulse_per_mm == 0)
		return 0;
	double pulses = (double) mm * (double) ax->pulse_per_mm;
	// làm tròn gần nhất
	if (pulses >= 0)
		return (int32_t) (pulses + 0.5);
	else
		return (int32_t) (pulses - 0.5);
}

HAL_StatusTypeDef Axis_RunPosConst_mm(Axis_t *ax, float pos_relative_mm) {
	if (!ax)
		return HAL_ERROR;
	int32_t rel_pulse = Axis_mm_to_pulse(ax, pos_relative_mm);
	return Axis_RunPosConst(ax, rel_pulse);
}

HAL_StatusTypeDef Axis_RunToPosConst_mm(Axis_t *ax, float pos_target_mm) {
	if (!ax)
		return HAL_ERROR;
	int32_t tgt_pulse = Axis_mm_to_pulse(ax, pos_target_mm);
	return Axis_RunToPosConst(ax, tgt_pulse);
}

float Axis_GetCurrentPositionMM(const Axis_t *ax) {
	if (!ax || ax->pulse_per_mm == 0)
		return 0.0f;
	return (float) ax->current_pos_pulse / (float) ax->pulse_per_mm;
}

