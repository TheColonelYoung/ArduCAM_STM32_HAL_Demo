#include "SPI_camera.hpp"

SPI_camera::SPI_camera(I2C_HandleTypeDef I2C_handle, uint8_t I2C_address, SPI_HandleTypeDef SPI_handle, Chip_select_pin SPI_CS) : I2C_handle(I2C_handle), I2C_address(
        I2C_address), SPI_handle(SPI_handle), SPI_CS(SPI_CS){ }

void SPI_camera::Capture(uint capture_delay){
    ArduChip_write(0x04, 0x01); // Clear FIFO
    ArduChip_write(0x04, 0x01);

    HAL_Delay(1);

    ArduChip_write(0x04, 0x02); // Start capture

    HAL_Delay(1);

    // wait for capture done
    while (1) {
        uint8_t regValue        = ArduChip_read(0x41);
        uint8_t captureDoneMask = 0x8;
        if (regValue & captureDoneMask) break;
    }

    HAL_Delay(1);

    uint32_t image_size = ArduChip_FIFO_length();

    image_data.resize(image_size);

    ArduChip_start_DMA_transfer(image_size);

    // Delay to ensure full DMA transmission, Here would be better to check for DMA transfer complete
    HAL_Delay(capture_delay);

    ArduChip_CS_disable();
}

int SPI_camera::Sensor_write_register(uint8_t address, uint8_t data){
    return Sensor_write_register(Register_blob_8{ address, data });
}

int SPI_camera::Sensor_write_register(uint16_t address, uint8_t data){
    return Sensor_write_register(Register_blob_16{ address, data });
}

int SPI_camera::Sensor_write_register(Register_blob_8 reg){
    HAL_StatusTypeDef ret;

    uint8_t buf[2];
    buf[0] = reg.address;
    buf[1] = reg.data;
    ret    = HAL_I2C_Master_Transmit(&I2C_handle, I2C_address, buf, 2, HAL_MAX_DELAY);

    if (ret == HAL_OK) return (1);

    return (0);
}

int SPI_camera::Sensor_write_register(Register_blob_16 reg){
    HAL_StatusTypeDef ret;
    uint8_t buf[3];

    buf[0] = reg.address >> 8;
    buf[1] = reg.address & 0x00ff;
    buf[2] = reg.data;
    ret    = HAL_I2C_Master_Transmit(&I2C_handle, I2C_address, buf, 3, HAL_MAX_DELAY);

    if (ret == HAL_OK) return (1);

    return (0);
}

void SPI_camera::Sensor_write_register_bulk(vector<Register_blob_8> regs){
    for (unsigned int i = 0; i < regs.size(); i++) {
        Register_blob_8 &reg = regs[i];
        if ( (reg.address == 0xff) & (reg.data == 0xff) ) {
            break;
        } else {
            Sensor_write_register(regs[i]);
        }
    }
}

void SPI_camera::Sensor_write_register_bulk(vector<Register_blob_16> regs){
    for (unsigned int i = 0; i < regs.size(); i++) {
        Register_blob_16 &reg = regs[i];
        if ( (reg.address == 0xffff) & (reg.data == 0xff) ) {
            break;
        } else {
            Sensor_write_register(regs[i]);
        }
    }
}

int SPI_camera::ArduChip_write(uint8_t addr, uint8_t data){
    HAL_StatusTypeDef ret;

    uint8_t addr_write = addr | 0x80;

    ArduChip_CS_enable();

    ret = HAL_SPI_Transmit(&SPI_handle, (uint8_t *) &addr_write, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }

    ret = HAL_SPI_Transmit(&SPI_handle, (uint8_t *) &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return ret;
    }

    ArduChip_CS_disable();
    return 0;
}

int SPI_camera::ArduChip_read(uint8_t address){
    uint8_t addrMasked = address & 0x7F;
    uint8_t empty      = 0x00;
    uint8_t ret;

    ArduChip_CS_enable();
    HAL_SPI_TransmitReceive(&SPI_handle, &addrMasked, &ret, 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&SPI_handle, &empty, &ret, 1, HAL_MAX_DELAY);
    ArduChip_CS_disable();

    return ret;
}

int SPI_camera::ArduChip_FIFO_length(){
    uint32_t len1, len2, len3, len = 0;
    len1 = ArduChip_read(0x42);
    len2 = ArduChip_read(0x43);
    len3 = ArduChip_read(0x44) & 0x7f;
    len  = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return len;
}

void SPI_camera::ArduChip_start_burst_read(){
    uint8_t BURST_FIFO_READ = 0x3c;
    uint8_t empty = 0x00;
    HAL_SPI_TransmitReceive(&SPI_handle, &BURST_FIFO_READ, &empty, 1, HAL_MAX_DELAY);
}

void SPI_camera::ArduChip_start_DMA_transfer(uint size){
    ArduChip_CS_enable();
    ArduChip_start_burst_read();
    HAL_SPI_Receive_DMA(&SPI_handle, image_data.data(), size);
}
