/*
 * Axis_v2.h
 *
 *  Created on: Oct 6, 2025
 *      Author: DELL
 */

#ifndef INC_AXIS_V2_H_
#define INC_AXIS_V2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#elif defined(STM32F401xC) || defined(STM32F411xE)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32F1xx)
  #include "stm32f1xx_hal.h"
#elif defined(STM32F4xx)
  #include "stm32f4xx_hal.h"
#endif

// Trạng thái trục
typedef enum {
	AXIS_STATE_IDLE = 0, AXIS_STATE_RUNNING_CONST = 1, AXIS_STATE_ERROR = 2
} AxisState_t;

// Handle trục
typedef struct {
	// Phần cứng
	GPIO_TypeDef *GPIOx_Dir;
	uint16_t GPIO_Pin_Dir;
	TIM_HandleTypeDef *htim_Pul;
	uint32_t Channel_Pul;    // TIM_CHANNEL_1..4

	// Tham số
	uint32_t frequency_min;  // Hz
	uint32_t frequency_max;  // Hz
	uint32_t pulse_per_mm;   // xung/mm
	uint8_t invert_dir;     // 0: không đảo, 1: đảo chiều

	// Optional: EN pin (nếu cần)
	GPIO_TypeDef *GPIOx_En;
	uint16_t GPIO_Pin_En;
	uint8_t invert_en;      // 0: EN active high, 1: EN active low

	// Trạng thái/điều khiển chạy
	volatile int32_t current_pos_pulse;  // vị trí hiện tại (xung)
	volatile int32_t target_pos_pulse;   // vị trí đích (xung)
	volatile uint32_t pulses_to_go;      // tổng số xung cần đi
	volatile uint32_t pulses_done;       // số xung đã đi
	int8_t dir_sign;               // +1 hoặc -1
	uint32_t run_frequency;          // Hz hiện tại (const speed)
	AxisState_t state;

	// Dành cho registry/quản lý
	uint8_t registered;             // đã register vào danh sách global chưa
} Axis_t;

// ============ API ============

// Khởi tạo cấu hình cơ bản trục (có thể fill struct trực tiếp và bỏ qua hàm này)
void Axis_Init(Axis_t *ax, GPIO_TypeDef *GPIOx_Dir, uint16_t GPIO_Pin_Dir,
		TIM_HandleTypeDef *htim, uint32_t Channel_Pul, uint32_t frequency_min,
		uint32_t frequency_max, uint32_t pulse_per_mm, uint8_t invert_dir);

// Đăng ký trục vào registry để callback tìm thấy (tối đa AXIS_MAX_INSTANCES)
HAL_StatusTypeDef Axis_Register(Axis_t *ax);
void Axis_Unregister(Axis_t *ax);

// Thiết lập tần số chạy (Hz). clamp theo min/max. Có thể gọi trước Start.
HAL_StatusTypeDef Axis_SetFrequencyHz(Axis_t *ax, uint32_t freq_hz);

// Tính số xung và hướng từ vị trí hiện tại đến target (chỉ tính, không chạy)
HAL_StatusTypeDef AxisCalculatePulseToGo(Axis_t *ax, int32_t target_pos_pulse);

// Lập kế hoạch chạy hằng tốc đi tương đối/ tuyệt đối (chỉ tính, không chạy)
HAL_StatusTypeDef Axis_RunPosConst(Axis_t *ax, int32_t pos_relative_pulse);
HAL_StatusTypeDef Axis_RunToPosConst(Axis_t *ax, int32_t pos_target_pulse);
// Lập kế hoạch chạy tương đối VÀ set tần số cho riêng lần chạy đó.
HAL_StatusTypeDef Axis_RunPosConstWithFreq(Axis_t *ax,
		int32_t pos_relative_pulse, uint32_t freq_hz);
// Lập kế hoạch chạy tuyệt đối VÀ set tần số cho riêng lần chạy đó.
HAL_StatusTypeDef Axis_RunToPosConstWithFreq(Axis_t *ax,
		int32_t pos_target_pulse, uint32_t freq_hz);

// Bắt đầu chạy: set DIR, set PWM (HAL_TIM_PWM_Start_IT), state -> RUNNING_CONST
HAL_StatusTypeDef Axis_StartAxisConst(Axis_t *ax);

// Dừng: nếu state != ERROR => về IDLE, stop PWM. Nếu đang chạy dở, vị trí sẽ là vị trí thực tế đã đi.
void Axis_Stop(Axis_t *ax);

// E-Stop: dừng khẩn, state -> ERROR
void Axis_EStop(Axis_t *ax);

// Callback phải được gọi trong HAL_TIM_PWM_PulseFinishedCallback (PWM interrupt)
void Axis_TIM_PWM_PulseFinishedCallback_Const(TIM_HandleTypeDef *htim);

// Tiện ích
static inline uint8_t Axis_IsBusy(const Axis_t *ax) {
	return (ax && ax->state == AXIS_STATE_RUNNING_CONST);
}
static inline AxisState_t Axis_GetState(const Axis_t *ax) {
	return ax ? ax->state : AXIS_STATE_ERROR;
}
static inline void Axis_SetCurrentPositionPulse(Axis_t *ax, int32_t pos_pulse) {
	if (ax)
		ax->current_pos_pulse = pos_pulse;
}
static inline int32_t Axis_GetCurrentPositionPulse(const Axis_t *ax) {
	return ax ? ax->current_pos_pulse : 0;
}

// Tiện ích theo mm (nếu cần)
HAL_StatusTypeDef Axis_RunPosConst_mm(Axis_t *ax, float pos_relative_mm);
HAL_StatusTypeDef Axis_RunToPosConst_mm(Axis_t *ax, float pos_target_mm);
float Axis_GetCurrentPositionMM(const Axis_t *ax);

#ifdef __cplusplus
}
#endif

#endif /* INC_AXIS_V2_H_ */
