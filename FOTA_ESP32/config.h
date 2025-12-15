#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>


extern const char* ssid;
extern const char* password;

// SUPABASE config
extern const char* SUPABASE_URL;
extern const char* SUPABASE_KEY;

// Pin uart esp32
#define RXD2 16  
#define TXD2 17  

// Biến
extern const int ACK_TIMEOUT_NORMAL;
extern const int ACK_TIMEOUT_ERASE;
extern const int VERIFY_TIMEOUT_MS;
extern const long CHECK_INTERVAL;

#endif