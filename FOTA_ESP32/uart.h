#ifndef UART_H
#define UART_H

#include <Arduino.h>

void ota_statemachine_handler();
void Send_CMD_Check_Version();
void Send_CMD_Reset();

// Helper functions
bool downloadFile(String url);
uint32_t calculateCRC32(const uint8_t* data, size_t length);

#endif