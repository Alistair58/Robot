#include <stdio.h>
#include "ble.h"
#include "controller_ble.h" //The gatt file compiled by cmake
#include "globals.h"
#include "driving_motors.h" //for update_throttle
#include "stepper_motor.h" //for reset_stepper
#include "auto_mode.h"

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_context_callback_registration_t context_registration;
const uint8_t adv_data[] = { //Advertising data
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x06, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'R','o','b','o','t'
};
const uint8_t adv_data_len = sizeof(adv_data);
hci_con_handle_t con_handle = 0;
bool stuck_notifications_enabled = false;
bool turret_notifications_enabled = false;

void ble_setup(){
    l2cap_init();
    // setup SM (security manager): Display only
    sm_init();
    // setup ATT server
    att_server_init(profile_data,NULL, att_write_callback);//We don't have a readable characteristic
    printf("\natt server init complete");
    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, sizeof(bd_addr_t));
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);
    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    // register for ATT event
    att_server_register_packet_handler(packet_handler);
    //Bluetooth Host controller interface
    hci_power_control(HCI_POWER_ON);
}

//HCI events such as connection and disconnection
//These are the lower level packet interactions
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    //printf("\nPacket handler; packet_type %d",packet_type);
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t addr;
    bd_addr_type_t addr_type; 
    
    if (packet_type != HCI_EVENT_PACKET) return;
    uint8_t hci_event = hci_event_packet_get_type(packet);
    switch (hci_event) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("\nDisconnection complete");
            connected = 0;
            set_auto_mode(0);
            break;
        case HCI_EVENT_LE_META:
            switch(hci_event_le_meta_get_subevent_code(packet)){
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    con_handle = hci_subevent_le_connection_update_complete_get_connection_handle(packet);
                    printf("\nConnected. con_handle: %d",con_handle);
                    connected = 1;
                    break;
                default:
                    break;
            }
            break;
            
        default:
            break;
    }
}

//Higher level ATT (GATT) characteristic writes (phone sends data to pico)
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    switch (att_handle){
        case ATT_CHARACTERISTIC_94f493ce_c579_41c2_87ac_e12c02455864_01_CLIENT_CONFIGURATION_HANDLE:
            stuck_notifications_enabled =  little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            printf("\nStuck notifications enabled: %d",stuck_notifications_enabled);
            break;
        case ATT_CHARACTERISTIC_94f493d0_c579_41c2_87ac_e12c02455864_01_CLIENT_CONFIGURATION_HANDLE:
            turret_notifications_enabled =  little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            printf("\nTurret calibration notifications enabled: %d",turret_notifications_enabled);
            break;
        case ATT_CHARACTERISTIC_94f493ca_c579_41c2_87ac_e12c02455864_01_VALUE_HANDLE:
            //Characteristic packets are in the form: doubleCheck leftThrottle rightThrottle
            //0xa1 is the doubleCheck first packet (MOTOR_CONTROL_PACKET)
            if(buffer_size>=3 && buffer[0]==MOTOR_CONTROL_PACKET && !auto_mode){//Rogue packets after auto_mode can occur
                //printf("\nIncoming motor control packet, b1: %d b2: %d",buffer[1],buffer[2]);
                update_throttle(buffer[1],buffer[2]);
            }
            break;
        case ATT_CHARACTERISTIC_94f493cc_c579_41c2_87ac_e12c02455864_01_VALUE_HANDLE:
            //0xaa is the doubleCheck first packet (AUTO_MODE_PACKET)
            printf("\nAuto mode packet");
            if(buffer[0]==AUTO_MODE_PACKET){
                printf("\nIncoming auto mode packet");
                set_auto_mode(buffer[1]);
            }
            break;
        case ATT_CHARACTERISTIC_94f493d0_c579_41c2_87ac_e12c02455864_01_VALUE_HANDLE:
            //0xab is the doubleCheck first packet (TURRET_CALIBRATION_PACKET)
            printf("\nTurret calibration packet");
            if(buffer[0]==TURRET_CALIBRATION_PACKET && buffer[1]==1 && !auto_mode){
                turret_calibration();
            }
            break;
        default:
            break;
    }
    return 0;
}

int send_stuck_notification(){
    if(connected && con_handle){
        uint8_t buffer[2] = {STUCK_PACKET,1};
        int result = att_server_notify(con_handle, ATT_CHARACTERISTIC_94f493ce_c579_41c2_87ac_e12c02455864_01_VALUE_HANDLE,buffer,2);
        printf("\nSent stuck, result: %d",result);
    }
}


int send_turret_calibrated_notification(bool success){
    if(connected && con_handle){
        uint8_t buffer[2] = {TURRET_CALIBRATION_PACKET,success?2:3};
        int result = att_server_notify(con_handle, ATT_CHARACTERISTIC_94f493d0_c579_41c2_87ac_e12c02455864_01_VALUE_HANDLE,buffer,2);
        printf("\nSent turret calibrated, result: %d",result);
    }
}
