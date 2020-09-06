#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <limits.h>
#include <unistd.h>
#include <string.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_adc_cal.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "aws_iot_shadow_interface.h"

//Logging details
static const char *TAG = "plantWater";

//GPIO Definitions
#define GPIO_RELAY_1 32
#define GPIO_RELAY_2 33
#define GPIO_SENSOR_1 25
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_RELAY_1)) | ((1ULL<<GPIO_RELAY_2))
#define GPIO_SENSOR_PIN_SEL (1ULL << GPIO_SENSOR_1)
#define ESP_INTR_FLAG_DEFAULT 0

#define DEFAULT_VREF 1100
#define ADC_SAMPLES 32


//ADC Variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;
static const adc_bits_width_t adc_width = ADC_WIDTH_12Bit;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//WIFI Variables
const int WIFI_CONNECTED_BIT = BIT0;
const int WIFI_FAIL_BIT = BIT1;
static EventGroupHandle_t wifi_event_group;

//IoT Variables
AWS_IoT_Client mqttClient;
IoT_Client_Init_Params mqttInitParams;
IoT_Client_Connect_Params mqttConnectParams;

//Mqtt variables
char mqttHostAddress[255] = AWS_IOT_MQTT_HOST;
uint32_t mqttPort = AWS_IOT_MQTT_PORT;

//Certificate variables
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");


//Function declarations
void init_gpio();
void init_adc();
static void init_wifi();
void init_aws_iot();
uint32_t read_sensor();
void enable_sensor();
void disable_sensor();
bool manage_water_pump(uint32_t value);
void run_loop();
void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data);
void mqttSubscriptionCallback(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen, IoT_Publish_Message_Params *params, void *pData);

void init_gpio(){
    gpio_config_t io_conf;

    //SETUP RELAY GPIO PINS
    gpio_set_level(GPIO_RELAY_1, 1);
    gpio_set_level(GPIO_RELAY_2, 1);

    //Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //Set GPIO output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //Masks pins to setup GPIO
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //Disable pull-down mode
    io_conf.pull_down_en = 0;
    //Disable pull up mode
    io_conf.pull_up_en = 0;
    //Configure GPIO pins
    gpio_config(&io_conf);

    //=================================================

    //SETUP SENSOR GPIO PINS
    //Select pins
    io_conf.pin_bit_mask = GPIO_SENSOR_PIN_SEL;
    //Configure pins
    gpio_config(&io_conf);
}

void init_adc(){
    //Configuration
    adc1_config_width(adc_width);
    adc1_config_channel_atten((adc1_channel_t) channel, atten);

    //Set ADC characteristics
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, adc_width, DEFAULT_VREF, adc_chars);
}

//Reads sensor output via the adc
uint32_t read_sensor(){
    uint32_t adc_value = 0;

    //Read sensor value and average results
    for(int i = 0; i < ADC_SAMPLES; i++){
        adc_value += adc1_get_raw((adc1_channel_t) channel);
    }
    adc_value /= ADC_SAMPLES;

    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_value, adc_chars);

    printf("VOLTAGE: %d\n", voltage);

    return voltage;
}

//Enable plant sensor
void enable_sensor(){
    gpio_set_level(GPIO_SENSOR_1, 1);
}

//Disable plant sensors
void disable_sensor(){
    gpio_set_level(GPIO_SENSOR_1, 0);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    int wifi_retry_count = 0;

    if(event_base == WIFI_EVENT){
        if(event_id == WIFI_EVENT_STA_START){
            esp_wifi_connect();
        }else if(event_id == WIFI_EVENT_STA_DISCONNECTED){
            if(wifi_retry_count < CONFIG_ESP_MAXIMUM_RETRY){
                esp_wifi_connect();
                wifi_retry_count++;
            }else{
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    }else if(event_base == IP_EVENT){
        if(event_id == IP_EVENT_STA_GOT_IP){
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            wifi_retry_count = 0;
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

static void init_wifi(){
    //Init TCP/IP Adapater
    tcpip_adapter_init();

    //Create wifi event group
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //Initilization configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    //esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD
        },
    };

    //Set wifi mode and start WIFI adapter
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if(bits & WIFI_CONNECTED_BIT){
        printf("Wifi connection established!\n");
    }else if(bits & WIFI_FAIL_BIT){
        printf("Failed to connected to wifi\n");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
    vEventGroupDelete(wifi_event_group);
}

//Initializes connection to AWS IoT Core
void init_aws_iot(){
    mqttInitParams = iotClientInitParamsDefault;
    mqttConnectParams = iotClientConnectParamsDefault;

    //Setup mqtt Init
    mqttInitParams.enableAutoReconnect = false;
    mqttInitParams.pHostURL = mqttHostAddress;
    mqttInitParams.port = mqttPort;
    mqttInitParams.pRootCALocation = (const char *) aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *) certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *) private_pem_key_start;
    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler; //Create function
    mqttInitParams.disconnectHandlerData = NULL;

    //Start mqtt initialization
    IoT_Error_t initStatus = aws_iot_mqtt_init(&mqttClient, &mqttInitParams);

    if(initStatus != SUCCESS){
        ESP_LOGE(TAG, "Failed to initialize mqtt client. Gave error: %d\n", initStatus);
        abort();
    }

    //Start MQTT connection
    mqttConnectParams.keepAliveIntervalInSec = 100;
    mqttConnectParams.isCleanSession = true;
    mqttConnectParams.MQTTVersion = MQTT_3_1_1;
    mqttConnectParams.pClientID = CONFIG_AWS_IOT_CLIENT_ID;
    mqttConnectParams.clientIDLen = (uint16_t) strlen(CONFIG_AWS_IOT_CLIENT_ID);
    mqttConnectParams.isWillMsgPresent = false;

    //Connect to MQTT
    ESP_LOGI(TAG, "Connecting to AWS IoT Core...\n");
    IoT_Error_t connectStatus;
    do {
        connectStatus = aws_iot_mqtt_connect(&mqttClient, &mqttConnectParams);
        if(connectStatus != SUCCESS){
            ESP_LOGE(TAG, "Error(%d) connecting to AWS IoT Core %s:%d\n", connectStatus, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }else if(connectStatus == SUCCESS){
            ESP_LOGI(TAG, "Connected to AWS IoT Core!  (%d)\n", connectStatus);
        }
    } while(SUCCESS != connectStatus);

    //Re-enable auto reconnect
    IoT_Error_t reconErr = aws_iot_mqtt_autoreconnect_set_status(&mqttClient, true);

    if(reconErr != SUCCESS){
        IOT_ERROR("Error setting auto-reconnect to true: %d\n", reconErr);
    }else if(reconErr == SUCCESS){
        ESP_LOGI(TAG, "Successfully enabled auto-reconnect\n");
    }

    //Create subscription to IoT topic
    IoT_Error_t subStatus = aws_iot_mqtt_subscribe(&mqttClient, "plantControl", 13, QOS1, mqttSubscriptionCallback, NULL);

    if(subStatus == SUCCESS){
        ESP_LOGI(TAG, "Successfully subscribed to plantControl topic");
    }else{
        ESP_LOGE(TAG, "Error subscribing to IoT Topic: %d", subStatus);
    }

}


void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data){
    ESP_LOGW(TAG, "MQTT Disconnected!\n");
    IoT_Error_t status = FAILURE;

    if(pClient == NULL){
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient) == SUCCESS) {
        ESP_LOGI(TAG, "Auto reconnect is enabled. Will try to reconnect.\n");
    }else{
        ESP_LOGW(TAG, "Manually attempting reconnect to MQTT.\n");
        status = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED){
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        }else{
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d\n", status);
        }
    }
}

//Topic subscription callback
void mqttSubscriptionCallback(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen, IoT_Publish_Message_Params *params, void *pData){
    printf("RECEIVED MESSAGE\n");
    ESP_LOGI(TAG, "%s: %s", topicName, (char*) params->payload);
}


bool manage_water_pump(uint32_t value){
    const TickType_t pump_time = 15000 / portTICK_PERIOD_MS; //15 s
    if(value > 2500){
        gpio_set_level(GPIO_RELAY_1, 0);
        vTaskDelay(pump_time);
        gpio_set_level(GPIO_RELAY_1, 1);
        return true;
    }

    return false;
}


void run_loop(){
    const TickType_t delay_time = 2000 / portTICK_PERIOD_MS; //2 s
    const char *topic = "plantWater";
    const uint64_t sleepTime = 4 * 60 * 60; //Every 4 hours #hrs * mins * seconds

    IoT_Error_t conErr = SUCCESS;

    //Enable sensor
    enable_sensor();
    vTaskDelay(delay_time);

    while(conErr == NETWORK_ATTEMPTING_RECONNECT || conErr == NETWORK_RECONNECTED || conErr == SUCCESS){
        conErr = aws_iot_mqtt_yield(&mqttClient, 200);
        if(conErr == NETWORK_ATTEMPTING_RECONNECT){
            printf("MQTT attempting reconnect\n");
            continue;
        }
        
        //Get sensor value
        uint32_t voltage = read_sensor();

        bool pumpStatus = manage_water_pump(voltage);
        
        //Format message
        char msg[36];
        snprintf(msg, 36, "Voltage: %d  Pump: %d", voltage, pumpStatus);
        IoT_Publish_Message_Params pubParams;
        pubParams.qos = QOS1;
        pubParams.payload = (void *) msg;
        pubParams.payloadLen = strlen(msg);
        pubParams.isRetained = 0;

        //Publish message to AWS IoT Core
        conErr = aws_iot_mqtt_publish(&mqttClient, topic, strlen(topic), &pubParams);

        if(conErr == MQTT_REQUEST_TIMEOUT_ERROR){
            printf("MQTT Message timed out!\n");
        }else if(conErr != SUCCESS){
            printf("MQTT ERROR(%d)\n", conErr);
        }

        vTaskDelay(delay_time);

        //Disable sensor
        disable_sensor();

        //Go into sleep mode
        esp_sleep_enable_timer_wakeup(sleepTime * 1000000);
        printf("Going to sleep in a few seconds...\n");
        vTaskDelay(delay_time);
        esp_deep_sleep_start();
        break;
    }
}

void app_main()
{

    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    init_wifi();
    init_gpio();
    init_adc();
    init_aws_iot();
    run_loop();
}
