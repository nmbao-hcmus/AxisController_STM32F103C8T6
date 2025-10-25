/*
 * gcode_parser.h
 *
 *  Created on: Oct 24, 2025
 *      Author: DELL
 */

#ifndef INC_GCODE_PARSER_H_
#define INC_GCODE_PARSER_H_

#include <stdbool.h>
#include <stdint.h>
#include "Axis_v2.h" // Cần truy cập các hàm của thư viện Axis

// Cấu trúc để lưu trữ lệnh G-code đã được phân tích
typedef struct {
	char command_type; // 'G', 'M'
	int command_num;
	float x, y, z;
	float f; // Feed rate
	float s; // Spindle speed
	bool has_x, has_y, has_z, has_f, has_s;
} GCode_Command_t;

// Phân tích một dòng G-code
bool GCode_ParseLine(const char *line, GCode_Command_t *command);

// Thực thi một lệnh G-code
void GCode_ExecuteCommand(GCode_Command_t *cmd, Axis_t *axisX, Axis_t *axisY,
		Axis_t *axisZ);

#endif /* INC_GCODE_PARSER_H_ */
