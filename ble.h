#define SERVICE_UUID        "94f493c8_c579_41c2_87ac_e12c02455864"  // UART Service
#define CHAR_UUID_RX        "94f493c9_c579_41c2_87ac_e12c02455864"  // Write Characteristic (phone -> pico) 

#define APP_AD_FLAGS 0x06 //The flag 0x06 indicates: LE General Discoverable Mode and BR/EDR ncot supported.
#define MOTOR_CONTROL_PACKET 0xA1 //Inidcates the start of a motor control packet
#define AUTO_MODE_PACKET 0xAA //Indicates a automatic mode packet
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_context_callback_registration_t context_registration;
const uint8_t adv_data[] = { //Advertising data
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x06, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'R','o','b','o','t'
};
const uint8_t adv_data_len = sizeof(adv_data);


void ble_setup();
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

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
            auto_mode = false;
            reset_stepper();
            break;
        case HCI_EVENT_LE_META:
            switch(hci_event_le_meta_get_subevent_code(packet)){
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    printf("\nConnected");
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
        // case ATT_CHARACTERISTIC_94f493ca_c579_41c2_87ac_e12c02455864_01_CLIENT_CONFIGURATION_HANDLE:
        //     con_handle = connection_handle;
        //     break;
        case ATT_CHARACTERISTIC_94f493ca_c579_41c2_87ac_e12c02455864_01_VALUE_HANDLE:
            //Characteristic packets are in the form: doubleCheck leftThrottle rightThrottle
            //0xa1 is the doubleCheck first packet (MOTOR_CONTROL_PACKET)
            if(buffer[0]==MOTOR_CONTROL_PACKET){
                //printf("\nIncoming motor control packet");
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
        default:
            break;
    }
    return 0;
}
