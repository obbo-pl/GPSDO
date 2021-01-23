/*
 * usart_terminal.c
 *
 * Created: 27.08.2019 13:54:28
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 * AVR/GNU C Compiler : 4.8.1
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */ 

#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "terminal.h"


uint8_t terminal_commands_array_size = 0;
TERMINAL_COMMAND_t *terminal_commands_array;

#define TERMINAL_TIMEOUT_FLUSH_ms		280	//max 3276

void terminal_SendHelp(TERMINAL_t *terminal);
void terminal_SendUnknown(TERMINAL_t *terminal);
void terminal_ClearInputBuffer(TERMINAL_t *terminal);
void terminal_ClearCommandOptionBuffer(char *buffer);
void terminal_ShrinkBuffer(TERMINAL_t *terminal, uint8_t len);
bool terminal_FindCommand(TERMINAL_t *terminal, uint8_t len);


#define TERMINAL_MESSAGE_unknown_command	"? unknown command\n\r"
#define TERMINAL_MESSAGE_bad_argument		"? bad argument\n\r"
#define TERMINAL_MESSAGE_ok			"OK\n\r"
#define TERMINAL_MESSAGE_welcome		"\n\r\n\r@@@@ Terminal mode\n\r"
#ifdef TERMINAL_MESSAGE_IN_EEPROM
const char EEMEM terminal_message_001[] = TERMINAL_MESSAGE_unknown_command;
const char EEMEM terminal_message_002[] = TERMINAL_MESSAGE_bad_argument;
const char EEMEM terminal_message_003[] = TERMINAL_MESSAGE_ok;
const char EEMEM terminal_message_004[] = TERMINAL_MESSAGE_welcome;
#endif


void terminal_Init(TERMINAL_t *terminal, USART_DRIVER_t *usart, CircularBuffer_t *cbuffer)
{
	terminal->usart = usart;
	terminal->output_buffer = cbuffer;
}

void terminal_BindCommands(TERMINAL_COMMAND_t *commands, uint8_t length)
{
	terminal_commands_array = commands;
	terminal_commands_array_size = length;
}

void terminal_InterruptHandler(TERMINAL_t *terminal)
{
	char c = 0x00;
	if (!(terminal_IsBufferFull(terminal))) {
		c = usart_ReadChar(terminal->usart);
		terminal_ParseChar(terminal, c);
	}
}

void terminal_ParseChar(TERMINAL_t *terminal, char c)
{
	if ((c > 0x20) && (c < 0x7F)) {
		terminal->input_buffer[terminal->input_buffer_position] = c;
		terminal->input_buffer_position++;
		cbuffer_AppendChar(terminal->output_buffer, c);
	} else if (c == 0x20) {
		if (terminal->input_buffer_position > 0) {
			terminal->input_buffer[terminal->input_buffer_position] = c;
			terminal->input_buffer_position++;
		}
		cbuffer_AppendChar(terminal->output_buffer, c);
	} else if (c == 0x08) {
		if (terminal->input_buffer_position > 0) {
			terminal->input_buffer[terminal->input_buffer_position] = 0x00;
			terminal->input_buffer_position--;
		}
		cbuffer_AppendChar(terminal->output_buffer, c);
	} else if ((c == 0x0A) || (c == 0x0D)) {
		if (!terminal_IsBufferEmpty(terminal)) {
			terminal->input_buffer[terminal->input_buffer_position] = TERMINAL_NEW_LINE;
			terminal->input_buffer_position++;
		}
		terminal_SendNL(terminal, false);
	}
}

bool terminal_IsBufferEmpty(TERMINAL_t *terminal)
{
	if (terminal->input_buffer_position > 0) return false;
	return true;
}

bool terminal_IsBufferFull(TERMINAL_t *terminal)
{
	if (terminal->input_buffer_position >= TERMINAL_INPUT_BUFFER_LENGTH) return true;
	return false;
}

void terminal_ParseCommand(TERMINAL_t *terminal, uint8_t len)
{
	terminal_FindCommand(terminal, len);
	terminal_ShrinkBuffer(terminal, len);
	if (terminal_IsBufferEmpty(terminal)) terminal_ClearInputBuffer(terminal);
}

bool terminal_FindNewLine(TERMINAL_t *terminal, uint8_t *len)
{
	for (int i = 0; i < TERMINAL_INPUT_BUFFER_LENGTH; i++) {
		if (terminal->input_buffer[i] == TERMINAL_NEW_LINE) {
			*len = i + 1;
			return true;
		}
	}
	return false;
}

void terminal_SendWelcome(TERMINAL_t *terminal)
{
#ifdef TERMINAL_MESSAGE_IN_EEPROM
	cbuffer_AppendEEString(terminal->output_buffer, terminal_message_004);
#else
	cbuffer_AppendString(terminal->output_buffer, TERMINAL_MESSAGE_welcome);
#endif
}

void terminal_SendHelp(TERMINAL_t *terminal)
{
	for (int i = 0; i < terminal_commands_array_size; i++) {
		cbuffer_AppendString(terminal->output_buffer, terminal_commands_array[i].pattern);
		terminal_SendNL(terminal, false);
	}
	terminal_SendNL(terminal, false);
}

void terminal_SendUnknown(TERMINAL_t *terminal)
{
#ifdef TERMINAL_MESSAGE_IN_EEPROM
	cbuffer_AppendEEString(terminal->output_buffer, terminal_message_001);
#else
	cbuffer_AppendString(terminal->output_buffer, TERMINAL_MESSAGE_unknown_command);
#endif
}

void terminal_SendOK(TERMINAL_t *terminal)
{
#ifdef TERMINAL_MESSAGE_IN_EEPROM
	cbuffer_AppendEEString(terminal->output_buffer, terminal_message_003);
#else
	cbuffer_AppendString(terminal->output_buffer, TERMINAL_MESSAGE_ok);
#endif
}

void terminal_SendBadArgument(TERMINAL_t *terminal)
{
#ifdef TERMINAL_MESSAGE_IN_EEPROM
	cbuffer_AppendEEString(terminal->output_buffer, terminal_message_002);
#else
	cbuffer_AppendString(terminal->output_buffer, TERMINAL_MESSAGE_bad_argument);
#endif
}

bool terminal_FindCommand(TERMINAL_t *terminal, uint8_t len)
{
	bool result = false;
	if ((len == 2) && (terminal->input_buffer[0] == TERMINAL_HELP)) {
		terminal_SendHelp(terminal);
		result = true;
	}
	if (terminal->input_buffer[0] == TERMINAL_NEW_COMMAND) {
		for (int i = 0; i < terminal_commands_array_size; i++) {
			const char *patern;
			patern = terminal_commands_array[i].pattern;
			bool match = true;
			for (int j = 1; j < COMMAND_LENGTH; j++) {
				if (terminal->input_buffer[j] != patern[j]) match = false;
			}
			if (!((terminal->input_buffer[COMMAND_LENGTH] == TERMINAL_HELP)
			|| (terminal->input_buffer[COMMAND_LENGTH] == TERMINAL_NEW_LINE)
			|| (terminal->input_buffer[COMMAND_LENGTH] == TERMINAL_SPACE))) match = false;
			if (match) {
				for (int j = 0; j < (len - COMMAND_LENGTH); j++) {
					if (j >= (TERMINAL_OPTION_LENGTH - 1)) break;
					terminal->command_option[j] = terminal->input_buffer[j + COMMAND_LENGTH];
				}
				terminal_commands_array[i].callback(terminal);
				terminal_ClearCommandOptionBuffer(terminal->command_option);
				result = true;
			}
		}
	}
	if (!(result)) terminal_SendUnknown(terminal);
	return result;
}

void terminal_DropInputBuffer(TERMINAL_t *terminal)
{
	terminal_ClearInputBuffer(terminal);
	terminal_SendUnknown(terminal);
}

void terminal_ClearInputBuffer(TERMINAL_t *terminal)
{
	for (int i = 0; i < TERMINAL_INPUT_BUFFER_LENGTH; i++) {
		terminal->input_buffer[i] = 0x00;
	}
	terminal->input_buffer_position = 0;
}

void terminal_ShrinkBuffer(TERMINAL_t *terminal, uint8_t len)
{
	terminal->input_buffer[len - 1] = 0x00;
	for (int i = len; i < TERMINAL_INPUT_BUFFER_LENGTH; i++) {
		terminal->input_buffer[i - len] = terminal->input_buffer[i];
		terminal->input_buffer[i] = 0x00;
	}
	terminal->input_buffer_position -= len;
}

void terminal_ClearCommandOptionBuffer(char *buffer)
{
	for (int i = 0; i < TERMINAL_OPTION_LENGTH; i++) {
		buffer[i] = 0x00;
	}
}

void terminal_SendOutBuffer(TERMINAL_t *terminal)
{
	bool send_status = true;
	while (send_status && !(cbuffer_IsEmpty(terminal->output_buffer))) {
		send_status = usart_SendChar(terminal->usart, cbuffer_ReadChar(terminal->output_buffer));
		if (send_status) cbuffer_DropChar(terminal->output_buffer);
	}
}

void terminal_FlushOutBuffer(TERMINAL_t *terminal)
{
	bool send_status = true;
	uint16_t timeout = TERMINAL_TIMEOUT_FLUSH_ms * 20;
	while (timeout && !(cbuffer_IsEmpty(terminal->output_buffer))) {
		send_status = usart_SendChar(terminal->usart, cbuffer_ReadChar(terminal->output_buffer));
		if (send_status) cbuffer_DropChar(terminal->output_buffer);
		_delay_us(50);
		timeout--;
	}
}

void terminal_Print(TERMINAL_t *terminal, const char *src, bool flush)
{
	cbuffer_AppendString(terminal->output_buffer, src);
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintEE(TERMINAL_t *terminal, const char *src, bool flush)
{
	cbuffer_AppendEEString(terminal->output_buffer, src);
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintPM(TERMINAL_t *terminal, const char *src, bool flush)
{
	cbuffer_AppendPMString(terminal->output_buffer, src);
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintLn(TERMINAL_t *terminal, const char *src, bool flush)
{
	terminal_Print(terminal, src, flush);
	terminal_SendNL(terminal, flush);
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintInt(TERMINAL_t *terminal, int val, bool flush)
{
	char buff[7];
	itoa(val, buff, 10);
	cbuffer_AppendString(terminal->output_buffer, buff);
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintULong(TERMINAL_t *terminal, uint32_t val, bool flush)
{
	char buff[12];
	ultoa(val, buff, 10);
	cbuffer_AppendString(terminal->output_buffer, buff);
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintHex(TERMINAL_t *terminal, uint8_t val, bool flush)
{
	char buff[3];
	char *start = buff;
	if (val < 0x10) *start++ = 0x30;
	itoa(val, start, 16);
	cbuffer_AppendString(terminal->output_buffer, buff);
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintBin(TERMINAL_t *terminal, uint8_t val, bool flush)
{
	char buff[10];
	int i = 0x80;
	while (i > val)  {
		i = i >> 1;
		cbuffer_AppendString(terminal->output_buffer, "0");
	}
	if (val > 0) {
		itoa(val, buff, 2);
		cbuffer_AppendString(terminal->output_buffer, buff);
	}
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_PrintHexAndBin(TERMINAL_t *terminal, uint8_t val, bool flush)
{
	cbuffer_AppendString(terminal->output_buffer, "0x");
	if (flush) terminal_FlushOutBuffer(terminal);
	terminal_PrintHex(terminal, val, flush);
	cbuffer_AppendString(terminal->output_buffer, "  ");
	if (flush) terminal_FlushOutBuffer(terminal);
	terminal_PrintBin(terminal, val, flush);
	terminal_SendNL(terminal, flush);
}

void terminal_PrintBuf(TERMINAL_t *terminal, const uint8_t *buffer, uint8_t len, bool flush)
{
	while (len--) {
		terminal_PrintHex(terminal, buffer[len], flush);
	}
	if (flush) terminal_FlushOutBuffer(terminal);
}

void terminal_SendNL(TERMINAL_t *terminal, bool flush)
{
	cbuffer_AppendString(terminal->output_buffer, "\n\r");
	if (flush) terminal_FlushOutBuffer(terminal);
}


