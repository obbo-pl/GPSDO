/*
 * types.h
 *
 * Created: 2020-12-01 21:51:10
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 


#ifndef TYPES_H_
#define TYPES_H_


typedef struct GPSDO {
	const char *device_info;
	uint16_t temperature_raw;
	uint16_t temperature_k;
	bool gps_fixed;
	bool calibrated;
	uint32_t frequency;
	int error_frequency;
	float error_ppm;
	uint16_t dac_value;
	int dac_value_diff_last;
	uint8_t sequential_calibrated;
	uint32_t count_checks;
	uint32_t count_calibrated;
	uint32_t count_loss_calibration;
	int16_t max_diff_above_nominal;
	int16_t max_diff_below_nominal;
	volatile bool forward_gps_message;
	volatile bool show_gps_message;
	volatile bool show_gpsdo_status;
	volatile bool gpsdo_status_format_csv;
	volatile bool disable_frequency_correction;
	uint8_t errors;
} GPSDO_State_t;




#endif /* TYPES_H_ */