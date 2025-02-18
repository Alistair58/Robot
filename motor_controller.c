#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "hardware/pwm.h"
#include "motor_controller.h" //The gatt file compiled by cmake



//TODO 
//Try running Le counter example in a clean project


#define SERVICE_UUID        "94f493c8_c579_41c2_87ac_e12c02455864"  // UART Service
#define CHAR_UUID_RX        "94f493c9_c579_41c2_87ac_e12c02455864"  // Receive Characteristic (phone -> pico)
#define CHAR_UUID_TX        "94f493ca_c579_41c2_87ac_e12c02455864"  // Transmit Characteristic (pico -> phone)
#define APP_AD_FLAGS 0x06 //The flag 0x06 indicates: LE General Discoverable Mode and BR/EDR ncot supported.
#define MOTOR_CONTROL_PACKET 0xA1
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_context_callback_registration_t context_registration;
static hci_con_handle_t con_handle;
static int  connected = 0;
const uint8_t adv_data[] = { //Advertising data
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x06, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'R','o','b','o','t', 
    // Incomplete List of 16-bit Service Class UUIDs -- FF10 - only valid for testing!
    0x03, BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x10, 0xff,
};
const uint8_t adv_data_len = sizeof(adv_data);
//Facing the direction of the robot
const uint l_pwm = 21;
const uint l_forwards = 20;
const uint l_backwards = 19;
const uint r_pwm = 16;
const uint r_forwards = 17;
const uint r_backwards = 18;
uint l_slice_num;
uint l_channel;
uint r_slice_num;
uint r_channel;
static void ble_setup();
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static uint16_t att_read_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
void motor_control(uint8_t leftThrottle,uint8_t rightThrottle);
void gpio_pins_init(void);

int main()
{
    
    stdio_init_all();

    // Initialise the Wi-Fi, Bluetooth and light chip
    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return -1;
    }
    // Turn on the LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("\nLED on");    
    gpio_pins_init();
    
    ble_setup();

    //Bluetooth Host controller interface
    hci_power_control(HCI_POWER_ON);

    while(1) {
        tight_loop_contents(); //Keep the device on
    }
    return 0;
    
}

static void ble_setup(){
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
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);
    printf("\nAdvertising");

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(packet_handler);
}

/* 
 * @section Packet Handler
 *
 * @text The packet handler is used to:
 *        - stop the counter after a disconnect
 *        - send a notification when the requested ATT_EVENT_CAN_SEND_NOW is received
 */


static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    printf("\nPacket handler; packet_type %d",packet_type);
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t addr;
    bd_addr_type_t addr_type; 
    
    if (packet_type != HCI_EVENT_PACKET) return;
    uint8_t hci_event = hci_event_packet_get_type(packet);
    printf("\nhci_event: %d",hci_event);
    switch (hci_event) {
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("\nDisconnection complete");
            connected = 0;
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


/*
 * @section ATT Write
 *
 * @text The only valid ATT writes in this example are to the Client Characteristic Configuration, which configures notification
 * and indication and to the the Characteristic Value.
 * If the ATT handle matches the client configuration handle, the new configuration value is stored and used
 * in the heartbeat handler to decide if a new value should be sent.
 * If the ATT handle matches the characteristic value handle, we print the write as hexdump
 * See Listing attWrite.
 */

/* LISTING_START(attWrite): ATT Write */
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    printf("\nWrite callback");
    switch (att_handle){
        case ATT_CHARACTERISTIC_94f493ca_c579_41c2_87ac_e12c02455864_01_CLIENT_CONFIGURATION_HANDLE:
            //le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
            con_handle = connection_handle;
            break;
        case ATT_CHARACTERISTIC_94f493ca_c579_41c2_87ac_e12c02455864_01_VALUE_HANDLE:
            //Characteristic packets are in the form: doubleCheck leftThrottle rightThrottle
            //0xa1 is the doubleCheck first packet (MOTOR_CONTROL_PACKET)
            printf("\nWrite: transaction mode %u, offset %u, data (%u bytes): ", transaction_mode, offset, buffer_size);
            printf_hexdump(buffer, buffer_size);
            if(buffer[0]==MOTOR_CONTROL_PACKET){
                motor_control(buffer[1],buffer[2]);
            }
            
            break;
        default:
            break;
    }
    return 0;
}

void gpio_pins_init(){
     //Enable PWM on GPIO 21
     gpio_set_function(l_pwm, GPIO_FUNC_PWM);
     l_slice_num = pwm_gpio_to_slice_num(l_pwm);
     l_channel = pwm_gpio_to_channel(l_pwm);
     gpio_set_function(r_pwm, GPIO_FUNC_PWM);
     r_slice_num = pwm_gpio_to_slice_num(r_pwm);
     r_channel = pwm_gpio_to_channel(r_pwm);
 
     //Init direction pins
     gpio_init(l_forwards);
     gpio_init(l_backwards);
     gpio_init(r_forwards);
     gpio_init(r_backwards);
     gpio_set_dir(l_forwards,GPIO_OUT);
     gpio_set_dir(l_backwards,GPIO_OUT);
     gpio_set_dir(r_forwards,GPIO_OUT);
     gpio_set_dir(r_backwards,GPIO_OUT);
}
void motor_control(uint8_t leftThrottle,uint8_t rightThrottle){
    //leftThrottle and rightThrottle are both 0<= <=100

    //Set direction pins
    gpio_put(l_forwards,(leftThrottle<50)?0:1);
    gpio_put(l_backwards,(leftThrottle<50)?1:0);
    gpio_put(r_forwards,(rightThrottle<50)?0:1);
    gpio_put(r_backwards,(rightThrottle<50)?1:0);

    // Set period of 1000 cycles (0 to 999 inclusive)
    pwm_set_wrap(l_slice_num, 999);
    pwm_set_wrap(r_slice_num, 999);
 
    uint16_t l_duty = (uint16_t) round(((float)(abs(leftThrottle-50))/50) * 1000);
    //Abs as we want the values furthest from 50 (0 and 100 to give the fastest outputs)
    uint16_t r_duty = (uint16_t) round(((float)(abs(rightThrottle-50))/50) * 1000);
    pwm_set_chan_level(l_slice_num, l_channel, l_duty); //set duty cycle
    pwm_set_chan_level(r_slice_num, r_channel, r_duty);
    // Set the PWM running
    pwm_set_enabled(l_slice_num, true);
    pwm_set_enabled(r_slice_num, true);
}