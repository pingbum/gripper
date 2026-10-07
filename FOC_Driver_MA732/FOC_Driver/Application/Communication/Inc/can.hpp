#pragma once

#include "main_cpp.hpp"

namespace CAN_Handler
{
    /**
     * @brief Configure FDCAN filters and start the peripheral.
     */
    void FDCAN1_SetupFiltersAndStart();

    /**
     * @brief Broadcast motor status over CAN.
     * @param position_deg Position in degrees.
     * @param speed_erpm Speed in eRPM.
     * @param current_A Current in amperes.
     * @param temp_C Temperature in Celsius.
     * @param error_code Error code.
     */
    void broadcast_motor_status(float position_deg, float speed_erpm, float current_A, int8_t rev, int8_t error_code);
}
