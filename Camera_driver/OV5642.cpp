#include "OV5642.hpp"

void OV5642::Init(){
    Sensor_write_register((uint16_t) 0x3008, 0x80);

    Sensor_write_register_bulk(OV5642_QVGA_Preview);
    Sensor_write_register_bulk(OV5642_JPEG_Capture_QSXGA);

    Sensor_write_register((uint16_t) 0x3818, 0xa8); //TIMING CONTROL - ENABLE COMPRESSION, THUMBNAIL MODE DISABLE, VERTICAL FLIP, MIRROR
    Sensor_write_register((uint16_t) 0x3621, 0x10); //REGISTER FOR CORRECT MIRROR FUNCTION
    Sensor_write_register((uint16_t) 0x3801, 0xb0); //TIMING HORIZONTAL START - ALSO FOR MIRROR
    Sensor_write_register((uint16_t) 0x4407, 0x04); // COMPRESSION CONTROL
}

void OV5642::Init(const vector<Register_blob_16> &regs){
    Init();

    Sensor_write_register_bulk(regs);

    // Setup camera, H-sync: High, V-sync:high, Sensor_delay: no Delay, FIFO_mode:FIFO enabled, power_mode:Low_power
    ArduChip_write(0x03, 0x02);
}

void OV5642::Capture(uint capture_delay){
    ArduChip_write(0x01, 0x00); // Capture Control Register - Set to capture n+1 frames

    SPI_camera::Capture(capture_delay);
} // OV5642::Capture
