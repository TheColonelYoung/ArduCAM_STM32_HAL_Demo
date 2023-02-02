/**
 * @file OV2640.hpp
 * @author Petr Malaník (TheColonelYoung(at)gmail(dot)com)
 * @brief
 * @version 0.1
 * @date 11.09.2022
 */

#pragma once

#include "SPI_camera.hpp"
#include "OV2640_regs.hpp"

using namespace std;
typedef unsigned int uint;

/**
 * @brief 2MP SPI based camera capable of JPEG compression, based on ArduChip
 */
class OV2640: protected SPI_camera
{
private:
    /**
     * @brief Initialize camera sensor for JPEG capture, without resolution settings
     */
    void Init() override final;

public:
    /**
     * @brief Construct a new OV5642 camera object
     *
     * @param I2C_handle    HAL handle of I2C which is connected to image sensor
     * @param SPI_handle    HAL handle of SPI to which is ArduChip connected
     * @param SPI_CS        GPIO description which serves as SPI Chip select
     */
    OV2640(I2C_HandleTypeDef I2C_handle, SPI_HandleTypeDef SPI_handle, Chip_select_pin SPI_CS):
        SPI_camera(I2C_handle, 0x60, SPI_handle, SPI_CS) {};

    /**
     * @brief Initialize camera sensor for JPEG capture
     *
     * @param regs Requested resolution of camera
     */
    void Init(const vector<Register_blob_8> &regs);

    /**
     * @brief Return pointer to last captured image data
     *
     * @return vector<uint8_t>  Pointer to last captured image data
     */
    vector<uint8_t> Image_data(){ return image_data; };
};


