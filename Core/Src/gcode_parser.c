#include "gcode_parser.h"
#include <stdlib.h> // for atof
#include <string.h>
#include <ctype.h> // for toupper

// Biến lưu trữ trạng thái modal (lệnh G0/G1 trước đó)
static int modal_g_command = 0;

bool GCode_ParseLine(const char *line, GCode_Command_t *command) {
	// Reset cấu trúc command
	memset(command, 0, sizeof(GCode_Command_t));
	command->command_num = -1; // Mặc định không có lệnh G/M

	const char *ptr = line;

	while (*ptr) {
		// Bỏ qua khoảng trắng
		while (*ptr && isspace((unsigned char )*ptr))
			ptr++;
		if (!*ptr)
			break; //Exit ptr when finished

		char key = toupper(*ptr); //Viet Hoa
		ptr++;
		float value = atof(ptr); // Chuyển chuỗi số thành float

		switch (key) {
		case 'G':
			command->command_type = 'G';
			command->command_num = (int) value;
			modal_g_command = command->command_num;
			break;
		case 'M':
			command->command_type = 'M';
			command->command_num = (int) value;
			break;
		case 'X':
			command->x = value;
			command->has_x = true;
			break;
		case 'Y':
			command->y = value;
			command->has_y = true;
			break;
		case 'Z':
			command->z = value;
			command->has_z = true;
			break;
		case 'F':
			command->f = value;
			command->has_f = true;
			break;
		case 'S':
			command->s = value;
			command->has_s = true;
			break;
		}

		// Di chuyển con trỏ đến khoảng trắng tiếp theo hoặc cuối chuỗi
		while (*ptr && !isspace((unsigned char )*ptr))
			ptr++;
	}

	// Xử lý trường hợp không có G trong dòng (ví dụ: X30.513Y40.683)
	if (command->command_num == -1
			&& (command->has_x || command->has_y || command->has_z)) {
		command->command_type = 'G';
		command->command_num = modal_g_command; // Dùng lại lệnh G0/G1 trước đó
	}

	return command->command_num != -1;
}

// Hàm chuyển đổi Feed Rate (mm/phút) sang Tần số xung (Hz)
//static uint32_t feedrate_to_frequency(float feedrate, uint32_t pulses_per_mm) {
//    if (feedrate <= 0 || pulses_per_mm == 0) {
//        return 1000; // Tần số mặc định
//    }
//    // (mm/min) * (1 min / 60s) * (pulses / mm) = pulses / s = Hz
//    return (uint32_t)((feedrate / 60.0f) * pulses_per_mm);
//}

void GCode_ExecuteCommand(GCode_Command_t *cmd, Axis_t *axisX, Axis_t *axisY,
		Axis_t *axisZ) {
//    static float last_feedrate = 2000.0f; // Lưu lại tốc độ F cuối cùng
//
//    if (cmd->has_f) {
//        last_feedrate = cmd->f;
//    }

	if (cmd->command_type == 'G'
			&& (cmd->command_num == 0 || cmd->command_num == 1)) {
//        uint32_t freq_x = feedrate_to_frequency(last_feedrate, axisX->pulse_per_mm);
//        uint32_t freq_y = feedrate_to_frequency(last_feedrate, axisY->pulse_per_mm);
//        uint32_t freq_z = feedrate_to_frequency(last_feedrate, axisZ->pulse_per_mm);

		// Nếu G0 (rapid move), có thể dùng tốc độ tối đa
//        if (cmd->command_num == 0) {
//            freq_x = axisX->frequency_max;
//            freq_y = axisY->frequency_max;
//            freq_z = axisZ->frequency_max;
//        }
		// Lập kế hoạch di chuyển
		int32_t target_x =
				cmd->has_x ?
						(float) ((cmd->x * axisX->pulse_per_mm)) :
						axisX->current_pos_pulse;
		int32_t target_y =
				cmd->has_y ?
						(float) (cmd->y * axisY->pulse_per_mm) :
						axisY->current_pos_pulse;
		int32_t target_z =
				cmd->has_z ?
						(float) (cmd->z * axisZ->pulse_per_mm) :
						axisZ->current_pos_pulse;

//        Axis_RunToPosConstWithFreq(axisX, target_x, freq_x);
//        Axis_RunToPosConstWithFreq(axisY, target_y, freq_y);
//        Axis_RunToPosConstWithFreq(axisZ, target_z, freq_z);

		Axis_RunToPosConst(axisX, target_x);
		Axis_RunToPosConst(axisY, target_y);
		Axis_RunToPosConst(axisZ, target_z);
		// Bắt đầu chạy
		Axis_StartAxisConst(axisX);
		Axis_StartAxisConst(axisY);
		Axis_StartAxisConst(axisZ);
	} else if (cmd->command_type == 'M') {
		if (cmd->command_num == 3) {
			// TODO: Bật trục chính (Spindle) với tốc độ S
			// Ví dụ: Spindle_SetSpeed(cmd->s);
		} else if (cmd->command_num == 30) {
			// TODO: Chương trình kết thúc, có thể reset hoặc về home
		}
	}
}
