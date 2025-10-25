#include "uart_handler.h"
#include <string.h>

#define RX_BUFFER_SIZE 128 // Kích thước bộ đệm nhận

// Biến cục bộ
static UART_HandleTypeDef *g_huart;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_char; // Biến tạm để nhận 1 ký tự
static volatile uint16_t rx_index = 0;
static volatile bool new_line_ready = false;

// Hàm khởi tạo
void UART_Init(UART_HandleTypeDef *huart) {
	g_huart = huart;
	// Kích hoạt ngắt nhận UART, mỗi lần nhận 1 ký tự
	HAL_UART_Receive_IT(g_huart, &rx_char, 1);
}

// Hàm kiểm tra
bool UART_IsNewLineReady(void) {
	return new_line_ready;
}

// Hàm lấy dòng lệnh
void UART_GetLine(char *buffer) {
	if (new_line_ready) {
		// Tạm thời vô hiệu hóa ngắt để sao chép an toàn
		__HAL_UART_DISABLE_IT(g_huart, UART_IT_RXNE);

		memcpy(buffer, (char*) rx_buffer, rx_index);
		buffer[rx_index] = '\0'; // Kết thúc chuỗi
		rx_index = 0;

		new_line_ready = false;

		// Bật lại ngắt
		__HAL_UART_ENABLE_IT(g_huart, UART_IT_RXNE);
	}
}

// Hàm gửi chuỗi
void UART_Transmit(const char *str) {
	HAL_UART_Transmit(g_huart, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
}

// Hàm Callback được gọi khi ngắt UART xảy ra
// *** RẤT QUAN TRỌNG: Bạn phải gọi hàm này từ callback chính của HAL ***
void UART_Receive_Callback(void) {
	if (!new_line_ready) {
		// Nếu là ký tự xuống dòng, báo hiệu đã nhận xong 1 dòng
		if (rx_char == '\n' || rx_char == '\r') {
			if (rx_index > 0) { // Chỉ xử lý nếu dòng không rỗng
				rx_buffer[rx_index] = '\0'; // Kết thúc chuỗi
				new_line_ready = true;
			}
		} else {
			// Lưu ký tự vào buffer
			if (rx_index < RX_BUFFER_SIZE - 1) {
				rx_buffer[rx_index++] = rx_char;
			}
		}
	}
	// Kích hoạt lại ngắt để nhận ký tự tiếp theo
	HAL_UART_Receive_IT(g_huart, &rx_char, 1);
}
