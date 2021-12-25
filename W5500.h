/*
 * W5500.h
 *
 *  Created on: Jun 22, 2021
 *      Author: Bio_Unit
 */

#ifndef SRC_W5500_H_
#define SRC_W5500_H_

///////////////////////// Port defines /////////////////////////////////////
#define CS_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define RST_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define RST_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
//-------------------------------------------------------------------------

//////////////////////// API //////////////////////////////////////////////
void w5500_write_reg(uint8_t op, uint16_t address, uint8_t data);
void w5500_write_reg_fdm(uint16_t address, uint8_t *data_to_write, uint8_t memory_type, uint8_t operation_mode);
void w5500_write_reg_vdm(uint16_t address, uint8_t *data_to_write, uint8_t memory_type, uint8_t bytes_to_write);
void w5500_read_reg_fdm(uint16_t address, uint8_t *data_to_store, uint8_t memory_type, uint8_t operation_mode);
void w5500_read_reg_vdm(uint16_t address, uint8_t *data_to_store, uint8_t memory_type, uint8_t bytes_to_read);
void w5500_send_packet(uint8_t *data_to_transmit, uint8_t bytes_to_send);
uint8_t w5500_receive_packet(uint8_t *data_to_receive);
void w5500_clear_int_reg(void);
void w5500_change_destination_addr(uint8_t *dest_IP_settings);
void w5500_change_IP_addr(uint8_t *ip_settings);
void w5500_init(SPI_HandleTypeDef *hspi);
//------------------------------------------------------------------------

#endif /* SRC_W5500_H_ */
