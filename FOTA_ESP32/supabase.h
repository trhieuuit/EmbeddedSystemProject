#ifndef SUPABASE_H
#define SUPABASE_H

#include <Arduino.h>

void checkCloudCommand();
void updateCommandStatus(int id, String status, String message = "");

#endif