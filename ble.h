#ifndef BLE_H
#define BLE_H

#include "btstack.h" 
#define SERVICE_UUID        "94f493c8_c579_41c2_87ac_e12c02455864"  // UART Service
#define CHAR_UUID_RX        "94f493c9_c579_41c2_87ac_e12c02455864"  // Write Characteristic (phone -> pico) 
#define APP_AD_FLAGS 0x06 //The flag 0x06 indicates: LE General Discoverable Mode and BR/EDR ncot supported.
#define MOTOR_CONTROL_PACKET 0xA1 //Inidcates the start of a motor control packet
#define AUTO_MODE_PACKET 0xAA //Indicates a automatic mode packet

void ble_setup();
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);


#endif