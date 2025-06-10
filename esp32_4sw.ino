#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <Preferences.h>       // Use Preferences instead of EEPROM (NVS storage)
#include <ezButton.h>

//---------------------------------------------------
const char *service_name = "Prov_Tahidul";
const char *pop = "12345678";

//---------------------------------------------------
// Device Names
char device1[] = "Switch1";
char device2[] = "Switch2";
char device3[] = "Switch3";
char device4[] = "Switch4";

//---------------------------------------------------
// GPIO mapping
static uint8_t RELAY_1 = 5;   // D23
static uint8_t RELAY_2 = 18;  // D22
static uint8_t RELAY_3 = 19;  // D21
static uint8_t RELAY_4 = 21;  // D19

ezButton button1(34);
ezButton button2(35);
ezButton button3(32);
ezButton button4(33);

static uint8_t WIFI_LED    = 2;   // D2
static uint8_t gpio_reset = 0;

//---------------------------------------------------
// Relay State
bool STATE_RELAY_1 = LOW;
bool STATE_RELAY_2 = LOW;
bool STATE_RELAY_3 = LOW;
bool STATE_RELAY_4 = LOW;

//---------------------------------------------------
// RainMaker device objects
static Switch my_switch1(device1, &RELAY_1);
static Switch my_switch2(device2, &RELAY_2);
static Switch my_switch3(device3, &RELAY_3);
static Switch my_switch4(device4, &RELAY_4);

// Preferences for NVS storage
Preferences preferences;

/****************************************************************************************************
 * sysProvEvent Function
*****************************************************************************************************/
void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {      
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
        printQR(service_name, pop, "ble");
#else
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
        printQR(service_name, pop, "softap");
#endif        
        break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.printf("\nConnected to Wi-Fi!\n");
        digitalWrite(WIFI_LED, HIGH);
        break;
    }
}

/****************************************************************************************************
 * write_callback Function
*****************************************************************************************************/
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();

    if(strcmp(device_name, device1) == 0) {
        Serial.printf("Lightbulb1 = %s\n", val.val.b? "true" : "false");
        if(strcmp(param_name, "Power") == 0) {
            STATE_RELAY_1 = val.val.b;
            STATE_RELAY_1 = !STATE_RELAY_1;
            control_relay(1, RELAY_1, STATE_RELAY_1);
        }
    }
    else if(strcmp(device_name, device2) == 0) {
        Serial.printf("Switch value = %s\n", val.val.b? "true" : "false");
        if(strcmp(param_name, "Power") == 0) {
            STATE_RELAY_2 = val.val.b;
            STATE_RELAY_2 = !STATE_RELAY_2;
            control_relay(2, RELAY_2, STATE_RELAY_2);
        }
    }
    else if(strcmp(device_name, device3) == 0) {
        Serial.printf("Switch value = %s\n", val.val.b? "true" : "false");
        if(strcmp(param_name, "Power") == 0) {
            STATE_RELAY_3 = val.val.b;
            STATE_RELAY_3 = !STATE_RELAY_3;
            control_relay(3, RELAY_3, STATE_RELAY_3);        
        }
    }
    else if(strcmp(device_name, device4) == 0) {
        Serial.printf("Switch value = %s\n", val.val.b? "true" : "false");
        if(strcmp(param_name, "Power") == 0) {
            STATE_RELAY_4 = val.val.b;
            STATE_RELAY_4 = !STATE_RELAY_4;
            control_relay(4, RELAY_4, STATE_RELAY_4);
        } 
    }
}

/****************************************************************************************************
 * setup Function
*****************************************************************************************************/
void setup(){
    uint32_t chipId = 0;
    Serial.begin(115200);

    // Preferences (NVS) - no need to define size
    preferences.begin("relays", false);



    // Set the Relays GPIOs as output mode
    pinMode(RELAY_1, OUTPUT);
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);
    pinMode(RELAY_4, OUTPUT);

    // set debounce time to 100 milliseconds
    button1.setDebounceTime(100);
    button2.setDebounceTime(100);
    button3.setDebounceTime(100);
    button4.setDebounceTime(100);

    pinMode(gpio_reset, INPUT);
    pinMode(WIFI_LED, OUTPUT);
    digitalWrite(WIFI_LED, LOW);

    // Retrieve relay states from Preferences (NVS) or set to default LOW
    STATE_RELAY_1 = preferences.getBool("relay1", LOW);
    STATE_RELAY_2 = preferences.getBool("relay2", LOW);
    STATE_RELAY_3 = preferences.getBool("relay3", LOW);
    STATE_RELAY_4 = preferences.getBool("relay4", LOW);

    // Write to the GPIOs the default state on booting
    digitalWrite(RELAY_1, STATE_RELAY_1);
    digitalWrite(RELAY_2, STATE_RELAY_2);
    digitalWrite(RELAY_3, STATE_RELAY_3);
    digitalWrite(RELAY_4, STATE_RELAY_4);

    Node my_node;    
    my_node = RMaker.initNode("Ahmad_Logs");

    //Standard switch device
    my_switch1.addCb(write_callback);
    my_switch2.addCb(write_callback);
    my_switch3.addCb(write_callback);
    my_switch4.addCb(write_callback);

    //Add switch device to the node   
    my_node.addDevice(my_switch1);
    my_node.addDevice(my_switch2);
    my_node.addDevice(my_switch3);
    my_node.addDevice(my_switch4);

    RMaker.enableOTA(OTA_USING_PARAMS);
    RMaker.enableTZService();
    RMaker.enableSchedule();

    for(int i=0; i<17; i=i+8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }

    Serial.printf("\nChip ID:  %d Service Name: %s\n", chipId, service_name);
    Serial.printf("\nStarting ESP-RainMaker\n");
    RMaker.start();

    WiFi.onEvent(sysProvEvent);
    #if CONFIG_IDF_TARGET_ESP32
        WiFiProv.beginProvision(NETWORK_PROV_SCHEME_BLE, NETWORK_PROV_SCHEME_HANDLER_FREE_BTDM, NETWORK_PROV_SECURITY_1, pop, service_name);
    #else
        WiFiProv.beginProvision(NETWORK_PROV_SCHEME_SOFTAP, NETWORK_PROV_SCHEME_HANDLER_NONE, NETWORK_PROV_SECURITY_1, pop, service_name);
    #endif

    my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_1);
    my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_2);
    my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_3);
    my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_4);

    Serial.printf("Relay1 is %s \n", STATE_RELAY_1? "ON" : "OFF");
    Serial.printf("Relay2 is %s \n", STATE_RELAY_2? "ON" : "OFF");
    Serial.printf("Relay3 is %s \n", STATE_RELAY_3? "ON" : "OFF");
    Serial.printf("Relay4 is %s \n", STATE_RELAY_4? "ON" : "OFF");
}

/****************************************************************************************************
 * loop Function
*****************************************************************************************************/
void loop()
{
    // Read GPIO0 (external button to reset device)
    if(digitalRead(gpio_reset) == LOW) { //Push button pressed
        Serial.printf("Reset Button Pressed!\n");
        delay(100);
        int startTime = millis();
        while(digitalRead(gpio_reset) == LOW) delay(50);
        int endTime = millis();

        if ((endTime - startTime) > 10000) {
            // If key pressed for more than 10secs, reset all
            Serial.printf("Reset to factory.\n");
            RMakerFactoryReset(2);
        } 
        else if ((endTime - startTime) > 3000) {
            Serial.printf("Reset Wi-Fi.\n");
            // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
            RMakerWiFiReset(2);
        }
    }

    delay(100);

    if (WiFi.status() != WL_CONNECTED){
        digitalWrite(WIFI_LED, LOW);
    }
    else{
        digitalWrite(WIFI_LED, HIGH);
    }

    button_control();
}

/*******************************************************************************
 * button_control function:
 ******************************************************************************/
void button_control(){
    button1.loop();
    if(button1.isPressed()){
        control_relay(1, RELAY_1, STATE_RELAY_1);
        my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_1);
    }
    button2.loop();
    if(button2.isPressed()){
        control_relay(2, RELAY_2, STATE_RELAY_2);
        my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_2);
    }
    button3.loop();
    if(button3.isPressed()){
        control_relay(3, RELAY_3, STATE_RELAY_3);
        my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_3);
    }
    button4.loop();
    if(button4.isPressed()){
        control_relay(4, RELAY_4, STATE_RELAY_4);
        my_switch4.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_4);
    }
}

/****************************************************************************************************
 * control_relay Function
*****************************************************************************************************/
void control_relay(int relay_no, int relay_pin, boolean &status){
    status = !status;
    digitalWrite(relay_pin, status);
    preferences.begin("relays", false);
    switch(relay_no) {
        case 1: preferences.putBool("relay1", status); break;
        case 2: preferences.putBool("relay2", status); break;
        case 3: preferences.putBool("relay3", status); break;
        case 4: preferences.putBool("relay4", status); break;
        default: break;
    }
    preferences.end();
    String text = (status)? "ON" : "OFF";
    Serial.println("Relay"+String(relay_no)+" is "+text);
}

