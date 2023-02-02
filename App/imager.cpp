#include "imager.hpp"

int Imager::Transmit(vector<uint8_t> source){
    for( auto &uart_output : uart_output_interfaces){
        HAL_UART_Transmit(&uart_output, source.data(), source.size(), HAL_MAX_DELAY);
        HAL_Delay(200);
    }
    return uart_output_interfaces.size() * source.size();
}

int Imager::Add_output(UART_HandleTypeDef uart_output){
    uart_output_interfaces.emplace_back(uart_output);
    return uart_output_interfaces.size();
}
