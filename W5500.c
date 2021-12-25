/*
 * W5500.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Bio_Unit
 *
 *      Description:
 * 		This library made for W5500 network IC. For now its functions work only in UDP mode.
 * 		The most abstract functions for users are:
 * 			1) w5500_init. Initialization;
 * 			2) Send_packet. Transmits packet;
 * 			3) Receive_packet. Determines quantity of received bytes and writes it to user's buffer;
 */
#include "main.h"
#include "W5500.h"


///////////////////////// Register types ///////////////////////////////////
#define BSB_COMMON	0x00			// Common register for BSB-byte
#define BSB_S0 		0x01			// Socket 0 register for BSB-byte
#define BSB_S0_TX 	0x02			// Transmit register for BSB-byte
#define BSB_S0_RX 	0x03			// Receive register for BSB-byte
//--------------------------------------------------------------------------
#define RWB_WRITE 	1				// Write bit for BSB-byte
#define RWB_READ 	0				// Read bit for BSB-byte
//---------------------------------------------------------------------------
////////////////////////// Operational mode bits ///////////////////////////
#define OM_VDM 		0x00			// variable length operational mode
#define OM_FDM1 	0x01			// 1 byte fixed operational mode
#define OM_FDM2 	0x02			// 2 bytes fixed operational mode
#define OM_FDM4 	0x03			// 4 bytes fixed operational mode
//---------------------------------------------------------------------------
////////////////////////// Socket n registers //////////////////////////////
#define Sn_MR 			0x0000		// Mode Register
#define Sn_CR 			0x0001		// Control register
#define Sn_IR 			0x0002		// Interrupt register
#define Sn_SR 			0x0003		// Status register
#define Sn_PORT0 		0x0004 		// Socket 0 Source Port Register MSB
#define Sn_PORT1 		0x0005 		// Socket 0 Source Port Register LSB
#define Sn_DPORT0		0x0010		// Destination Port - MSB
#define Sn_DPORT1		0x0011		// Destination Port - LSB
#define Sn_DIPR0		0x000C		// Destination address - MSB
#define Sn_DIPR1		0x000D		// -
#define Sn_DIPR2		0x000E		// -
#define Sn_DIPR3		0x000F		// Destination address - LSB
#define Sn_RXBUF_SIZE 	0x001E		// Size register for socket n RX buffer MSB
#define Sn_TXBUF_SIZE 	0x001F		// Size register for socket n RX buffer LSB
#define Sn_TX_RD0		0x0022		// TX read pointer MSB
#define Sn_TX_RD1		0x0023		// TX read pointer LSB
#define Sn_TX_WR0		0x0024		// TX write pointer MSB
#define Sn_TX_WR1		0x0025		// TX write pointer LSB
#define Sn_RX_RSR0 		0x0026		// Received data size register MSB - this register shouldn't be used for income data
#define Sn_RX_RSR1		0x0027		// Received data size register LSB
#define Sn_RX_RD0 		0x0028		// Read pointer register MSB
#define Sn_RX_RD1 		0x0029		// Read pointer register LSB
#define Sn_RX_WR0 		0x002A		// Write pointer register MSB
#define Sn_RX_WR1		0x002B		// Write pointer register LSB - but this one does
//-------------------------------------------------------------------------
////////////////////////// Socket protocol mode ////////////////////////////
#define Mode_CLOSED 		0x00
#define Mode_TCP 			0x01
#define Mode_UDP 			0x02
#define Mode_MACRAV 		0x04
//-------------------------------------------------------------------------
///////////////////////// Socket states ///////////////////////////////////
#define SOCK_CLOSED 		0x00
#define SOCK_INIT 			0x13
#define SOCK_LISTEN 		0x14
#define SOCK_ESTABLISHED 	0x17
//-------------------------------------------------------------------------
///////////////////// Common registers ///////////////////////////////////
#define MR 			0x0000			// Mode Register
#define GWR0 		0x0001			// Gateway IP Address Register MSB
#define GWR1 		0x0002			// -
#define GWR2 		0x0003			// -
#define GWR3 		0x0004			// Gateway IP Address Register LSB
#define SUBR0 		0x0005			// Subnet Mask Register MSB
#define SUBR1 		0x0006			// -
#define SUBR2 		0x0007			// -
#define SUBR3 		0x0008			// Subnet Mask Register LSB
#define SHAR0 		0x0009			// Source Hardware Address Register MSB
#define SHAR1 		0x000A			// -
#define SHAR2 		0x000B			// -
#define SHAR3 		0x000C			// -
#define SHAR4 		0x000D			// -
#define SHAR5 		0x000E			// Source Hardware Address Register LSB
#define SIPR0 		0x000F			// Source IP Address Register MSB
#define SIPR1 		0x0010			// -
#define SIPR2 		0x0011			// -
#define SIPR3 		0x0012			// Source IP Address Register LSB
#define INTLEVEL0 	0x0013			// Interrupt time delay MSB
#define INTLEVEL1 	0x0014			// Interrupt time delay LSB
#define IR 			0x0015			// error INT status register
#define IMR 		0x0016			// error INT mask register
#define SIR 		0x0017			// socket INT status register
#define SIMR 		0x0018			// socket INT mask register
#define PHYCFGR 	0x002E			//
#define RTR0 		0x0019			// Retry Time-value register MSB - it means time to response
#define RTR1 		0x001A			// Retry Time-value register LSB
#define RCR 		0x001B			// Retry count register
//----------------------------------------------------------------------------

////////////////////// Global variables ////////////////////////////////
extern UART_HandleTypeDef huart1;

uint8_t macaddr[6] = {0x00, 0x15, 0x42, 0xBF, 0xF0, 0x51};	// w5500 MAC-address
uint8_t ipaddr[4] = {10, 0, 23, 15};						// W5500 IP-address
uint8_t ipgate[4] = {10, 0, 20, 8};							// W5500 Gate address
uint8_t ipmask[4] = {255, 255, 0, 0};						// W5500 mask
uint8_t dipaddr[4] = {10, 0, 23, 9};						// Destination address
uint16_t local_port = 80;									// W5500 port
uint8_t rx_pointer[2] = {0};								// Pointer for Socket RX buffer
uint16_t tx_point = 0;										// Pointer for Socket TX buffer
unsigned int w5500_spi_pointer = 0;							// storage for SPI structure pointer value from "main.c"
//----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
/*
 * This function writes 1 byte in W5500 register
 * Parameters:
 * 		hspi    : SPI structure pointer
 * 		op      : ready opcode. Should be recoded
 * 		address : 16bit register address to write in
 * 		data    : data to write in
 * Return value : none
 */
void w5500_write_reg(uint8_t op, uint16_t address, uint8_t data)
{
	uint8_t buf[4] = {address >> 8, address, op|(RWB_WRITE<<2), data};			// loading packet to send command on reading
	CS_ON;																		// CS enable
	HAL_SPI_Transmit(w5500_spi_pointer,(uint8_t *) &buf, 4, 2);					// If delay will be less than 2, the SPI wont able to keep up with data rate
	CS_OFF;																		// CS disable
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function WRITES 1,2 or 4 bytes to W5500 registers
 * Parameters:
 * 		hspi           : SPI structure pointer
 * 		address        : 16bit address of a register to WRITE in
 * 		data_to_write  : an array/buffer to WRITE in
 * 		memory_type    : type of W5500 internal registers memory (BSB_COMMON, BSB_S0 etc.)
 * 		operation_mode : 1, 2 or 4 bytes to WRITE
 * Return value        : none (should be return_status)
*/
void w5500_write_reg_fdm(uint16_t address, uint8_t *data_to_write, uint8_t memory_type, uint8_t operation_mode)
{
	uint8_t buf[3] = {address >> 8, address, (memory_type<<3)|(RWB_WRITE<<2)|operation_mode};	// packet to send with address and BSB-byte
	if( operation_mode > 2 )
	{
		operation_mode = 4;																		// converting operation_mode to bytes count
	}
	CS_ON;																						// CS enable
	HAL_SPI_Transmit(w5500_spi_pointer, (uint8_t *) &buf, 3, 2);								// If delay will be less than 2, the SPI wont able to keep up with data rate
	HAL_SPI_Transmit(w5500_spi_pointer, (uint8_t *) &data_to_write, operation_mode, 2);			// If delay will be less than 2, the SPI wont able to keep up with data rate
	CS_OFF;																						// CS disable
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function WRITES n-bytes to W5500 registers
 * Parameters:
 * 		hspi           : SPI structure pointer
 * 		address        : 16bit address of a register to WRITE in
 * 		data_to_write  : an array/buffer to WRITE in
 * 		memory_type    : type of W5500 internal registers memory (BSB_COMMON, BSB_S0 etc.)
 * 		bytes_to_write : 1 - 255 number of bytes
 * Return value        : none (should be return_status)
 */
void w5500_write_reg_vdm(uint16_t address, uint8_t *data_to_write, uint8_t memory_type, uint8_t bytes_to_write)
{
	uint8_t buf[3] = {address >> 8, address, (memory_type<<3)|(RWB_WRITE<<2)};					// packet to send with address and BSB-byte
	CS_ON;																						// CS enable
	HAL_SPI_Transmit(w5500_spi_pointer, (uint8_t *) &buf, 3, 2);								// If delay will be less than 2, the SPI wont able to keep up with data rate
	HAL_SPI_Transmit(w5500_spi_pointer, (uint8_t *) data_to_write, bytes_to_write, 2);			// If delay will be less than 2, the SPI wont able to keep up with data rate
	CS_OFF;																						// CS disable
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function READS 1,2 or 4 bytes from W5500 registers
 * Parameters:
 * 		hspi           : SPI structure pointer
 * 		address        : 16bit address of a register to READ from
 * 		data_to_store  : an array/buffer to store data
 * 		memory_type    : type of W5500 internal registers memory (BSB_COMMON, BSB_S0 etc.)
 * 		operation_mode : 1, 2 or 4 bytes to READ
 * Return value        : none
*/
void w5500_read_reg_fdm(uint16_t address, uint8_t *data_to_store, uint8_t memory_type, uint8_t operation_mode)
{
	uint8_t buf[3] = {address >> 8, address, (memory_type<<3)|operation_mode};		// packet to send with address and BSB-byte
	if( operation_mode > 2 )
	{
		operation_mode = 4;															// converting operation_mode to bytes count
	}
	CS_ON;																			// CS enable
	HAL_SPI_Transmit(w5500_spi_pointer,(uint8_t *) &buf, 3, 2);						// If delay will be less than 2, the SPI wont able to keep up with data rate
	HAL_SPI_Receive(w5500_spi_pointer, data_to_store, operation_mode, 2);			// If delay will be less than 2, the SPI wont able to keep up with data rate
	CS_OFF;																			// CS disable
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function READS n-bytes from W5500 registers (perhaps it works only with TX and RX buffer registers)
 * Parameters:
 * 		hspi          : SPI structure pointer
 * 		address       : 16bit address of a register to READ from
 * 		data_to_store : an array/buffer to store data
 * 		memory_type   : type of W5500 internal registers memory (BSB_COMMON, BSB_S0 etc.)
 * 		bytes_to_read : 1 - 255 number of bytes
 * Return value       : none
 */
void w5500_read_reg_vdm(uint16_t address, uint8_t *data_to_store, uint8_t memory_type, uint8_t bytes_to_read)
{
	uint8_t buf[3] = {address >> 8, address, memory_type<<3};						// packet to send with address and BSB-byte
	CS_ON;																			// CS enable
	HAL_SPI_Transmit(w5500_spi_pointer, (uint8_t *) &buf, 3, 2);					// If delay will be less than 2, the SPI wont able to keep up with data rate
	HAL_SPI_Receive(w5500_spi_pointer, data_to_store, bytes_to_read, 2);			// If delay will be less than 2, the SPI wont able to keep up with data rate
	CS_OFF;																			// CS disable
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function sends n-bytes of user's data by UDP
 * Parameters:
 * 		data_to_transmit : an array/buffer of data to send to destination address
 * 		bytes_to_send    : 1 - 255 number of bytes
 * Return value          : none
 */
void w5500_send_packet(uint8_t *data_to_transmit, uint8_t bytes_to_send)
{
	w5500_write_reg_vdm(tx_point, data_to_transmit, BSB_S0_TX, bytes_to_send);	// Writing data to TX_BUF
	tx_point += bytes_to_send;													// increasing value of pointer to write next time
	w5500_write_reg(9, Sn_TX_WR0, tx_point >> 8);								// Writing MSB tx_pointer to Sn_TX_WR0
	w5500_write_reg(9, Sn_TX_WR1, tx_point & 0xFF);								// Writing LSB tx_pointer to Sn_TX_WR0
	w5500_write_reg(9, Sn_CR, 0x20);											// SEND command to start transmitting procedure
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function checks and READS all received data including: host address, port and data
 * Parameters:
 * 		data_to_receive : an array/buffer to store received data
 * Return values        : 1 - no data received
 * 				          0 - data received
 */
uint8_t w5500_receive_packet(uint8_t *data_to_receive)
{
	uint16_t difference = 0;
	uint8_t rx_compare[2] = {rx_pointer[0], rx_pointer[1]};										// registers for previous values of pointer to compare
	w5500_read_reg_fdm(Sn_RX_WR0, rx_pointer, BSB_S0, OM_FDM2);									// read a size of income data
	difference = ((rx_pointer[0] - rx_compare[0]) << 8) | (rx_pointer[1] - rx_compare[1]);
	if( difference > 0 )															// if previous value is same - no data received
	{
		w5500_read_reg_vdm(rx_compare[0]<<8 | rx_compare[1], data_to_receive, BSB_S0_RX, difference);	// read data from RX buffer
		w5500_write_reg(9, Sn_RX_RD0, rx_pointer[0]);											// update pointer of RX buffer
		w5500_write_reg(9, Sn_RX_RD1, rx_pointer[1]);											// update pointer of RX buffer
		w5500_write_reg(9, Sn_CR, 0x40);														// RECV command to end receiving procedure
		return 0;
	}
	else
	{
		return 1;																				// if no data received - return 1
	}
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function clears interrupt registers
 * Parameters    : none
 * Return values : none
 */
void w5500_clear_int_reg(void)
{
	  w5500_write_reg(1, IR, 0xFF);													// clearing error interrupt status register
	  w5500_write_reg(9, Sn_IR, 0xFF);												// clearing socket interrupt status register
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function changes IP destination address
 * Parameters       :
 * 		ip_settings : 6-byte array (4-byte destination IP address, 2-byte port)
 * Return values 	: none
 */
void w5500_change_destination_addr(uint8_t *dest_IP_settings)
{
	uint8_t opcode = (BSB_S0<<3)|OM_FDM1;;

	w5500_write_reg(opcode, Sn_DIPR0, dest_IP_settings[0]);
	w5500_write_reg(opcode, Sn_DIPR1, dest_IP_settings[1]);
	w5500_write_reg(opcode, Sn_DIPR2, dest_IP_settings[2]);
	w5500_write_reg(opcode, Sn_DIPR3, dest_IP_settings[3]);
	w5500_write_reg(opcode, Sn_DPORT0, dest_IP_settings[4]);
	w5500_write_reg(opcode, Sn_DPORT1, dest_IP_settings[5]);
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function changes W5500 IP, mask and port
 * Parameters       :
 * 		ip_settings : 8-byte array (4-byte IP, 4-byte mask, 2-byte port)
 * Return values 	: none
 */
void w5500_change_IP_addr(uint8_t *ip_settings)
{
	uint8_t opcode = (BSB_COMMON<<3)|OM_FDM1;

	w5500_write_reg(opcode, SIPR0, ip_settings[0]);
	w5500_write_reg(opcode, SIPR1, ip_settings[1]);
	w5500_write_reg(opcode, SIPR2, ip_settings[2]);
	w5500_write_reg(opcode, SIPR3, ip_settings[3]);
	w5500_write_reg(opcode, SUBR0, ip_settings[4]);
	w5500_write_reg(opcode, SUBR1, ip_settings[5]);
	w5500_write_reg(opcode, SUBR2, ip_settings[6]);
	w5500_write_reg(opcode, SUBR3, ip_settings[7]);

	opcode = (BSB_S0<<3)|OM_FDM1;
	w5500_write_reg(opcode, Sn_PORT0, ip_settings[8]);	//socket source port (port of w5500 socket)
	w5500_write_reg(opcode, Sn_PORT1, ip_settings[9]);
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
/*
 * This function initializes W5500 in UDP mode
 * Parameters:
 * 		hspi : SPI structure pointer
 * Return value: none
 */
void w5500_init(SPI_HandleTypeDef *hspi)
{
	w5500_spi_pointer = hspi;
	uint8_t opcode=0;
	///////////////// Hard Reset /////////////////////////////
	RST_ON;
	HAL_Delay(50);
	RST_OFF;
	HAL_Delay(50);
	///////////////// Soft Reset ////////////////////////////
	opcode = (BSB_COMMON<<3)|OM_FDM1;
	w5500_write_reg(opcode, MR, 0x80);
	HAL_Delay(50);

	/////////////////////////PHY//////////////////////////////
	w5500_write_reg(opcode, PHYCFGR, 0x00);				//PHY reset
	w5500_write_reg(opcode, PHYCFGR, 0xFF);
	/////////////////////////////////////////////////////////

	///////////////// Net address ///////////////////////////
	w5500_write_reg(opcode, SHAR0,macaddr[0]);
	w5500_write_reg(opcode, SHAR1,macaddr[1]);
	w5500_write_reg(opcode, SHAR2,macaddr[2]);
	w5500_write_reg(opcode, SHAR3,macaddr[3]);
	w5500_write_reg(opcode, SHAR4,macaddr[4]);
	w5500_write_reg(opcode, SHAR5,macaddr[5]);
	w5500_write_reg(opcode, GWR0,ipgate[0]);
	w5500_write_reg(opcode, GWR1,ipgate[1]);
	w5500_write_reg(opcode, GWR2,ipgate[2]);
	w5500_write_reg(opcode, GWR3,ipgate[3]);
	w5500_write_reg(opcode, SUBR0,ipmask[0]);
	w5500_write_reg(opcode, SUBR1,ipmask[1]);
	w5500_write_reg(opcode, SUBR2,ipmask[2]);
	w5500_write_reg(opcode, SUBR3,ipmask[3]);
	w5500_write_reg(opcode, SIPR0,ipaddr[0]);
	w5500_write_reg(opcode, SIPR1,ipaddr[1]);
	w5500_write_reg(opcode, SIPR2,ipaddr[2]);
	w5500_write_reg(opcode, SIPR3,ipaddr[3]);
	////////////////////////////////////////////////////////

	//////////////// Interrupts ////////////////////////////
	w5500_write_reg(opcode, INTLEVEL0, 0x00);			//INT time MSB
	w5500_write_reg(opcode, INTLEVEL1, 0x10);			//INT time LSB
	w5500_write_reg(opcode, IMR, 0x00);					//all error interrupts suspended
	w5500_write_reg(opcode, IR, 0xFF);					//clearing error interrupt status register
	w5500_write_reg(opcode, SIMR, 0x01);				//switching on socket_0 interrupts
	///////////////////////////////////////////////////////
	w5500_write_reg(opcode, RTR0, 0x07);
	w5500_write_reg(opcode, RTR1, 0xD0);
	//////////////// Socket 0 /////////////////////////////
	opcode = (BSB_S0<<3)|OM_FDM1;
	w5500_write_reg(opcode, Sn_DIPR0, dipaddr[0]);		//setting destination IP address
	w5500_write_reg(opcode, Sn_DIPR1, dipaddr[1]);
	w5500_write_reg(opcode, Sn_DIPR2, dipaddr[2]);
	w5500_write_reg(opcode, Sn_DIPR3, dipaddr[3]);
	w5500_write_reg(opcode, Sn_DPORT0, 0);				//setting destination port
	w5500_write_reg(opcode, Sn_DPORT1, 80);
	w5500_write_reg(opcode, Sn_IR, 0xFF);				//clearing socket interrupt status register
	w5500_write_reg(opcode, Sn_RXBUF_SIZE, 16);			//setting RX buffer size
	w5500_write_reg(opcode, Sn_TXBUF_SIZE, 16);
	w5500_write_reg(opcode, Sn_MR, Mode_UDP);			//Setting mode UDP
	w5500_write_reg(opcode, Sn_PORT0,local_port>>8);	//socket source port (port of w5500 socket)
	w5500_write_reg(opcode, Sn_PORT1,local_port);
	w5500_write_reg(opcode, Sn_CR, 0x01);				//send command
	HAL_Delay(50);
}

