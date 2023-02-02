/**
 * @file SPI_camera.hpp
 * @author Petr Malaník (TheColonelYoung(at)gmail(dot)com)
 * @brief
 * @version 0.1
 * @date 11.09.2022
 */

#pragma once

#include "stm32l4xx_hal.h"

#include <vector>
#include <array>
#include <memory>

using namespace std;
typedef unsigned int uint;

/**
 * @brief Class representing SPI cameras based on Arduchip solution
 *          Currently is only supported OV2640
 */
class SPI_camera
{
public:
    /**
     * @brief Describes GPIO which serves as SPI chip select pin
     */
    struct Chip_select_pin {
        GPIO_TypeDef *port;
        uint16_t      pin;
    };

    /**
     * @brief Represents address and data of sensor settings, 8bit address, 8 bit data for example for OV2640 sensor
     */
    struct Register_blob_8 {
        uint8_t address;
        uint8_t data;
    };

    /**
     * @brief Represents address and data of sensor settings, 16bit address, 8 bit data for example for OV5642 sensor
     */
    struct Register_blob_16 {
        uint16_t address;
        uint8_t  data;
    };

protected:
    /**
     * @brief HAL handle of I2C which is connected to image sensor
     */
    I2C_HandleTypeDef I2C_handle;

    /**
     * @brief I2C address of sensor
     *          Common values: 0x60 for OV2640, 0x78 for OV5642
     */
    uint8_t I2C_address = 0;

    /**
     * @brief HAL handle of SPI to which is ArduChip connected
     */
    SPI_HandleTypeDef SPI_handle;

    /**
     * @brief GPIO description which serves as SPI Chip select
     */
    Chip_select_pin SPI_CS;

    /**
     * @brief Raw image data transmitted from ArduChip via SPI to MCu memory
     */
    vector<uint8_t> image_data;

protected:
    /**
     * @brief Construct a new SPI camera object
     *
     * @param I2C_handle    HAL handle of I2C which is connected to image sensor
     * @param I2C_address   I2C address of sensor, Common values: 0x60 for OV2640, 0x78 for OV5642
     * @param SPI_handle    HAL handle of SPI to which is ArduChip connected
     * @param SPI_CS        GPIO description which serves as SPI Chip select
     */
    SPI_camera(I2C_HandleTypeDef I2C_handle, uint8_t I2C_address, SPI_HandleTypeDef SPI_handle, Chip_select_pin SPI_CS);

public:

    /**
     * @brief Virtual function for camera initialization, depends on camera model
     */
    virtual void Init() = 0;

    /**
     * @brief   Image capture start, various models of camera require some additional steps before this
     *
     * @param capture_delay Delay required for image capture before DMA trasnfer starts
     */
    virtual void Capture(uint capture_delay);

protected:

    /**
     * @brief   Write 8-bit data to 8-bit address in image sensor via I2C, used for example in OV2640
     *
     * @param address   Address (8-bit) to write
     * @param data      Data (8-bit) to write
     * @return int      HAL status
     */
    int Sensor_write_register(uint8_t address, uint8_t data);

    /**
     * @brief   Write 8-bit data to 16-bit address in image sensor via I2C, used for example in OV5642
     *
     * @param address   Address (16-bit) to write
     * @param data      Data (8-bit) to write
     * @return int      HAL status
     */
    int Sensor_write_register(uint16_t address, uint8_t data);

    /**
     * @brief   Write register representation in structure (8-bit address, 8-bit data) into image sensor via I2C
     *              used for example in OV2640
     *
     * @param reg   Register represented in structure (8-bit address, 8-bit data), used for example in OV2640
     * @return int  HAL status
     */
    int Sensor_write_register(Register_blob_8 reg);

    /**
     * @brief   Write register representation in structure (16-bit address, 8-bit data) into image sensor via I2C
     *              used for example in OV5642
     *
     * @param reg   Register represented in structure (16-bit address, 8-bit data), used for example in OV5642
     * @return int  HAL status
     */
    int Sensor_write_register(Register_blob_16 reg);

    /**
     * @brief   Write vector of register settings (8-bit address, 8-bit data) into image sensor in bulk
      *              used for example in OV2640
     *
     * @param regs  vector containing register settings (8-bit address, 8-bit data)
     */
    void Sensor_write_register_bulk(vector<Register_blob_16> regs);

    /**
     * @brief   Write vector of register settings (16-bit address, 8-bit data) into image sensor in bulk
     *              used for example in OV5642
     *
     * @param regs  vector containing register settings (16-bit address, 8-bit data), used for example in OV5642
     */
    void Sensor_write_register_bulk(vector<Register_blob_8> regs);

    /**
     * @brief Write data (8-bit) into register address (8-bit) of ArduChip via SPI
     *
     * @param addr  Address (8-bit) to write
     * @param data  Data (8-bit) to write
     * @return int  HAL status
     */
    int ArduChip_write(uint8_t addr, uint8_t data);

    /**
     * @brief Read data (8-bit) from register address (8-bit) of ArduChip via SPI
     *
     * @param address Address (8-bit) from which read
     * @return int  HAL status
     */
    int ArduChip_read(uint8_t address);

    /**
     * @brief Calculates size of FIFO which is used for image data inside ArduChip
     *              Performs communication via SPI with ArduChip
     *
     * @return int  Count of bytes in FIFO of ArduChip
     */
    int ArduChip_FIFO_length();

    /**
     * @brief   Initialize Burst read of FIFO containing image data via SPI from ArduChip
     *              Must be followed by DMA request and read, after readout is complete CS signal must be disabled
     */
    void ArduChip_start_burst_read();

    /**
     * @brief   Initialize DMA transfer of image data from ArduChip FIFO via SPI
     *              Burst read operation must be setup before,  after readout is complete CS signal must be disabled
     *
     * @param size Amount of bytes to read
     */
    void ArduChip_start_DMA_transfer(uint size);

    /**
     * @brief Enable communication with ArduChip via SPI, CS signal is active low
     */
    void ArduChip_CS_enable(){ HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_RESET); };

    /**
     * @brief Disables communication with ArduChip via SPI, CS signal is active low
     */
    void ArduChip_CS_disable(){ HAL_GPIO_WritePin(SPI_CS.port, SPI_CS.pin, GPIO_PIN_SET); };
};
