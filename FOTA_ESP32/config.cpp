#include "config.h"

// ================= CẤU HÌNH WIFI (Gán giá trị) =================
const char* ssid = "Nha_Chi_My_2.4G";
const char* password = "23456789";

// ================= CẤU HÌNH SUPABASE (Gán giá trị) =================
const char* SUPABASE_URL = "https://dugpttewrohmplazzfsi.supabase.co";
const char* SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImR1Z3B0dGV3cm9obXBsYXp6ZnNpIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjMyOTg5NDYsImV4cCI6MjA3ODg3NDk0Nn0.9o9WauB22L80viu2F8VqouzE38S5vUfiIaONZ2nJxbQ";

// ================= TIMEOUT (Gán giá trị) =================
const int ACK_TIMEOUT_NORMAL = 2000;
const int ACK_TIMEOUT_ERASE = 5000;
const int VERIFY_TIMEOUT_MS = 15000;
const long CHECK_INTERVAL = 2000;