/**
 * @file OV2640.hpp
 * @author Petr Malaník (TheColonelYoung(at)gmail(dot)com)
 * @brief
 * @version 0.1
 * @date 11.09.2022
 */

#pragma once

#include "stm32l4xx_hal.h"
#include "main.h"

#include <vector>

using namespace std;
typedef unsigned int uint;

/**
 * @brief Captures image data from camera, control power to camera and transmit data on selected interfaces
 *          Currently is only UART interface supported
 */
class Imager
{
private:
    /**
     * @brief UART interfaces to which data will be exported, interfaces muse be configured in advance
     */
    vector<UART_HandleTypeDef> uart_output_interfaces;

public:
    /**
     * @brief Construct a new ButCube_imager object
     */
    Imager() = default;

    /**
     * @brief Add UART output to list on which data are exported
     *
     * @param uart_output   UART output to which export data from camera
     * @return int
     */
    int Add_output(UART_HandleTypeDef uart_output);

    /**
     * @brief Transmit data from source to selected interfaces
     *
     * @param source    Source of image data
     * @return int      Count of bytes exported
     */
    int Transmit(vector<uint8_t> source);
};
