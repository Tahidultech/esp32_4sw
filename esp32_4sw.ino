#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <Preferences.h>       // Use Preferences instead of EEPROM (NVS storage)
#include <ezButton.h>

//---------------------------------------------------
const char* service_name = "Prov_Tahidul";
const char* pop = "12345678";

//---------------------------------------------------
// Device Names
const char* device_names[4] = {"Switch1", "Switch2", "Switch3", "Switch4"};

//---------------------------------------------------
// GPIO mapping
static const uint8_t RELAY_PINS[4] = {5, 18, 19, 21};   // D23, D22, D21, D19
static const uint8_t BUTTON_PINS[4] = {34, 35, 32, 33};
static const uint8_t WIFI_LED = 2;   // D2
static const uint8_t gpio_reset = 0;

//---------------------------------------------------
// Relay State
bool relay_states[4] = {LOW, LOW, LOW, LOW};

//---------------------------------------------------
// RainMaker device objects
static Switch* my_switches[4];

// Preferences for NVS storage
Preferences preferences;

// ezButton objects
ezButton buttons[4] = {ezButton(BUTTON_PINS[0]), ezButton(BUTTON_PINS[1]), ezButton(BUTTON_PINS[2]), ezButton(BUTTON_PINS[3])};

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
    const char* device_name = device->getDeviceName();
    const char* param_name = param->getParamName();

    for (int i = 0; i < 4; i++) {
        if(strcmp(device_name, device_names[i]) == 0 && strcmp(param_name, "Power") == 0) {
            Serial.printf("%s value = %s\n", device_names[i], val.val.b ? "true" : "false");
            relay_states[i] = !val.val.b; // invert logic if needed
            control_relay(i, relay_states[i]);
        }
    }
}

/****************************************************************************************************
 * setup Function
 *****************************************************************************************************/
void setup(){
    uint32_t chipId = 0;
    Serial.begin(115200);

    preferences.begin("relays", false);

    // Set the Relays GPIOs as output mode
    for (int i = 0; i < 4; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
        buttons[i].setDebounceTime(100);

        // Retrieve relay states from Preferences (NVS) or set to default LOW
        char key[8];
        snprintf(key, sizeof(key), "relay%d", i+1);
        relay_states[i] = preferences.getBool(key, LOW);

        digitalWrite(RELAY_PINS[i], relay_states[i]);
    }

    pinMode(gpio_reset, INPUT);
    pinMode(WIFI_LED, OUTPUT);
    digitalWrite(WIFI_LED, LOW);

    Node my_node = RMaker.initNode("Ahmad_Logs");

    // Initialize switch devices
    for (int i = 0; i < 4; i++) {
        my_switches[i] = new Switch(device_names[i], &RELAY_PINS[i]);
        my_switches[i]->addCb(write_callback);
        my_node.addDevice(*my_switches[i]);
    }

    // Uncomment if you need these services:
    // RMaker.enableOTA(OTA_USING_PARAMS);
    // RMaker.enableTZService();
    // RMaker.enableSchedule();

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

    // Report initial states
    for (int i = 0; i < 4; i++) {
        my_switches[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, relay_states[i]);
        Serial.printf("Relay%d is %s \n", i+1, relay_states[i] ? "ON" : "OFF");
    }
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

    digitalWrite(WIFI_LED, WiFi.status() == WL_CONNECTED ? HIGH : LOW);

    button_control();
}

/*******************************************************************************
 * button_control function:
 ******************************************************************************/
void button_control(){
    for (int i = 0; i < 4; i++) {
        buttons[i].loop();
        if(buttons[i].isPressed()){
            control_relay(i, relay_states[i]);
            my_switches[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, relay_states[i]);
        }
    }
}

/****************************************************************************************************
 * control_relay Function
 *****************************************************************************************************/
void control_relay(int idx, boolean &status){
    status = !status;
    digitalWrite(RELAY_PINS[idx], status);
    preferences.begin("relays", false);
    char key[8];
    snprintf(key, sizeof(key), "relay%d", idx+1);
    preferences.putBool(key, status);
    preferences.end();
    Serial.printf("Relay%d is %s\n", idx+1, status ? "ON" : "OFF");
}
