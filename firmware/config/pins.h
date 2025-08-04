#pragma once
/* ───────────── GPIO wiring ───────────── */
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;

/* ───────────── I²C devices ───────────── */
constexpr uint8_t PCA9685_ADDR = 0x40;

/* ───────────── Servo pulse limits ────── */
constexpr uint16_t SERVO_MIN = 80;
constexpr uint16_t SERVO_MAX = 600;

/* ───────────── Controller MAC ────────── */
constexpr char PS5_MAC[] = "4C:B9:9B:AD:03:BF";
