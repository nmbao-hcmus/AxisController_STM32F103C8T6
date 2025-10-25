/*
 * uart_handler.h
 *
 *  Created on: Oct 24, 2025
 *      Author: DELL
 */
#include "main.h"
#include <stdbool.h>
#include <string.h>

#define RX_BUFFER_SIZE 128 // Kích thước bộ đệm nhận
#ifndef INC_UART_HANDLER_H_
#define INC_UART_HANDLER_H_
// Bắt đầu nhận dữ liệu từ UART
void UART_Init(UART_HandleTypeDef *huart);

// Hàm này được gọi từ ngắt UART
void UART_Receive_Callback(void);

// Kiểm tra xem có dòng lệnh mới để xử lý không
bool UART_IsNewLineReady(void);

// Lấy dòng lệnh mới ra để xử lý
void UART_GetLine(char *buffer);

// Gửi một chuỗi qua UART
void UART_Transmit(const char *str);

#endif /* INC_UART_HANDLER_H_ */
