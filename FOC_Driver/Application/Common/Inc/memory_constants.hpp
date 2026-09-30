#pragma once

// Flash address map for parameter storage.
#define PARAMETER_VARIABLE_ADDR 0x08040000UL

// Parameter block word offsets (32-bit words).
#define KP_ADDR_OFFSET 0 // Kp at PARAMETER_VARIABLE_ADDR + OFFSET * 4.
#define KI_ADDR_OFFSET 1 // Ki at PARAMETER_VARIABLE_ADDR + OFFSET * 4.
#define KD_ADDR_OFFSET 2 // Kd at PARAMETER_VARIABLE_ADDR + OFFSET * 4.

#define FILTER_CUTOFF_FREQ_ADDR_OFFSET 10 // Cutoff frequency offset (word).

#define CONTROL_PARAM_FLASH_ADDR PARAMETER_VARIABLE_ADDR
#define CAN_ID_FLASH_ADDR 0x0805F000UL // CAN ID config (8-byte block).

// Cogging LUT persistent storage. STM32G474 flash pages are 2 KiB.
// Header uses one page; 1440 float32 values use the following three pages.
#define COGGING_LUT_HEADER_ADDR 0x0805D000UL
#define COGGING_LUT_DATA_ADDR   0x0805D800UL
#define COGGING_LUT_DATA_BYTES  5760UL

// Encoder calibration state block.
#define ENCODER_STATE_ADDR 0x0805F800UL
#define ENCODER_DIRECTION_OFFSET 0    // Direction value offset (word).
#define ENCODER_BUFFER_SIZE_OFFSET 1  // Buffer size offset (word).
#define ENCODER_FLASH_UPDATE_OFFSET 3 // Flash update flag offset (word).
#define ENCODER_FLASH_UPDATE (ENCODER_STATE_ADDR + ENCODER_FLASH_UPDATE_OFFSET * 4)
// Distinguishes MA732 calibration data from the previous AS5048 table.
#define ENCODER_CALIBRATION_MARKER 0x4D413732UL // ASCII "MA72"

// Encoder offset LUT storage.
#define ENCODER_MAX_BUFFER_SIZE 0x20000UL     // Bytes.
#define ENCODER_BUFFER_BASE_ADDR 0x08060000UL // Range: 0x08060000-0x0807FFFF.
