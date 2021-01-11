/* mqtt.c 

	All security channel access code, including Tasks
	
*/
#ifndef MQTT_C
#define MQTT_C		// define allowing the header file to identify the module using it

// ------- ADC 
#include "driver/gpio.h"			// added with ADC sample
#include "driver/adc.h"				// added with ADC sample
#include "esp_adc_cal.h"			// added with ADC sample

// ------- I2C
#include "esp_log.h"
#include "driver/i2c.h"

#include <math.h>         

// ------- MQTT
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "AiEngine.h"		// to replace microai.h
#include "esp32ai.h"
#include "securitychannels.h"

#include "mqtt.h"


// CONDITIONAL COMPILE OPTIONS (see esp32ai.h)


// External references
extern SemaphoreHandle_t print_mux;
extern SemaphoreHandle_t mqtt_mux;
 
static const char *TAG = "ESP32_MQTT";

#define INITIAL_MQTT_TASK_DELAY_SECS	15		// 15 seconds after powerup before starting MQTT task loop


// variables
static esp_mqtt_client_handle_t		mqtt_client;
static esp_mqtt_client_config_t 	mqtt_cfg;
#ifdef MQTT_ALERT_ENABLE
static esp_mqtt_client_handle_t		mqtt_alert_client;
static esp_mqtt_client_config_t 	mqtt_alert_cfg;
#endif

	   float						mqtt_databuf[NOF_MQTT_PARAMS];

// #ifdef MQTT_ALERT_ENABLE
char * MqttNameArry[5] =
{
	"Cpu",
	"Packets_Transmitted",
	"Packets_Received",
	"Packets_Dropped",
	"MACaddress"
};
// #endif

static uint8_t 	mIsMqttEnabled;
static int 		mintCurrTimeMsec, mintStartTimeMsec;	//use for time_span entry to send with MQTT packets (for now)
static int		mintMqttRatems;							// milliseconds between MQTT transmissions

#ifdef MQTT_ENABLE

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
//    esp_mqtt_client_handle_t client = event->client;
 //   int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
			// msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
			// ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        	// msg_id = esp_mqtt_client_publish(client, "mone", "data", 0, 2, 0);	
            // ESP_LOGI(TAG, "Connected, so now publish test data to 'mone', msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_subscribe(client, "mone", 0);			//don't need to subscribe since others may be sending as well?
			// ESP_LOGI(TAG, "subscribed to 'mone', msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");		example of how to unsubscribe
			// ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			// msg_id = esp_mqtt_client_publish(client, "mone", "data", 0, 2, 0);	don't need to test this now
			// ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
			#ifndef OUTPUT_AI_DATA
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			#endif
            break;
        case MQTT_EVENT_DATA:
			if (event->data_len >= 31)
				ESP_LOGI(TAG, "MQTT_EVENT_DATA received, length=%d, %.*s", event->data_len, 31, event->data);	// include first 31 bytes (mac address)
			else
				ESP_LOGI(TAG, "MQTT_EVENT_DATA received, length=%d", event->data_len);
				
            #ifdef OUTPUT_MQTT_DATA_RECD
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
			#endif
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);	// includes MQTT_EVENT_BEFORE_CONNECT (7)
            break;
    }
    return ESP_OK;
}


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static esp_mqtt_client_handle_t mqtt_app_start(esp_mqtt_client_config_t *p2Config)
{
	#if CONFIG_BROKER_URL_FROM_STDIN
		char line[128];

		if (strcmp(config.uri, "FROM_STDIN") == 0) {
			int count = 0;
			printf("Please enter url of mqtt broker\n");
			while (count < 128) {
				int c = fgetc(stdin);
				if (c == '\n') {
					line[count] = '\0';
					break;
				} else if (c > 0 && c < 127) {
					line[count] = c;
					++count;
				}
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
			config.uri = line;
			printf("Broker url: %s\n", line);
		} else {
			ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
			abort();
		}
	#endif /* CONFIG_BROKER_URL_FROM_STDIN */

//printf(" *** Client Init \n");
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(p2Config);
//printf(" *** Register Event \n");
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
//printf(" *** Client Start \n");
    esp_mqtt_client_start(client);
	
	return (client);
}
#endif	// #ifdef MQTT_ENABLE

/*****
	Assemble the MQTT message
	format: "$$mac_address,<addr>|<param2_id>,<param2_value>|....|temp_4,<temp4_value>##"

*****/
#define MAX_SIZEOF_MQTT_BUF		800
static void send_mqtt_msg(ENUM_MQTT_CHANNELS ch_num, char *p2Ch_name, char *p2ClientID, esp_mqtt_client_handle_t clientHandle )
{
   int 				msg_id;
    int 				len;
	char				charBuffer[MAX_SIZEOF_MQTT_BUF];
	
	sprintf( charBuffer, "$$mac_address,%s", p2ClientID );	// p2ClientID = mqtt_cfg.client_id
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|latitude, 32.911123" ); //hard-coded value
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|longitude, -96.877658" ); //hard-coded value
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|unix_time,%d", mintStartTimeMsec/1000 );
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|time_span,%d", (mintCurrTimeMsec-mintStartTimeMsec)/1000 );
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|host_device_id,45|edge_device_id,45" );
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|x_value,45.78|y_value,45.78|z_value,45.78" );
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|normalized_value,%.1f", mqtt_databuf[E_CURRENT_VAL] );		// current value
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|urgent_alarm,%d", (int)mqtt_databuf[E_URGENT_ALARM] );					// is alarm
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|model_1_color,45.78|model_2_color,45.78|model_3_color,45.78");
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|yellow_counter_1,45.78|yellow_counter_2,45.78|yellow_counter_3,45.78");
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|weight1,45.78|weight2,45.78|weight3,45.78");
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|health_level,%.1f", mqtt_databuf[E_HEALTH_SCORE] );					// health score
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|mean_score,45.78|bound_of_signals_value,45.78");
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|red_counter_1,45.78|red_counter_2,45.78|red_counter_3,45.78");
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|normalized_density,45.78");
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|days_to_maintenance,%d", (int)mqtt_databuf[E_DAYS_TO_MAINT]);	// MUST be an integer value
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|signal_color,45.78|machine_usage_time,45");
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|sensor_name,%s", p2Ch_name);
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|sensor_id,%d", ch_num);
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|temp_1,%.6f", mqtt_databuf[E_UPPER_BOUND]);				// upper limit
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|temp_2,%.6f", mqtt_databuf[E_LOWER_BOUND]);				// lower limit
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|temp_3,%d", (int)mqtt_databuf[E_ALARM]);
	len = strlen( charBuffer );
	sprintf( &charBuffer[len], "|temp_4,%.2f##", 245.78);
	len = strlen( charBuffer );

	if (len > MAX_SIZEOF_MQTT_BUF) {
		ESP_LOGE(TAG, " *** Exceeded buffer size *** \n");
		return;
	}
	else {
		//xSemaphoreTake(print_mux, portMAX_DELAY);				// don't need to see this any longer
		//printf("\n Buf size: %d\n", len);
		//xSemaphoreGive(print_mux);	
	}

	// now publish this message (note that the topic is only used when someone is subscribed to the channel)

	// (void)esp_mqtt_client_publish(clientHandle, "mone", charBuffer, 0, 2, 0);	// Length=0 (publish full string), QOS=2

//	(void)esp_mqtt_client_publish(mqtt_client, "mone", charBuffer, 0, 2, 0);	// Length=0 (publish full string), QOS=2
	msg_id = esp_mqtt_client_publish(mqtt_client, "msec", charBuffer, 0, 2, 0);	// Length=0 (publish full string), QOS=2
	ESP_LOGI(TAG, "sent publish to topic 'msec', msg_id=%d", msg_id);
}	

#ifdef MQTT_ALERT_ENABLE
// ------------------------------------------------------------------------
// MQTT alert message
//
//	mac_address		string
//	channel_id		int
//	sensor_name		string
//	current_value	float
//	upper_value		float
//	lower_value		float
//	description		string
//	event_type		string
//
// ------------------------------------------------------------------------
void send_mqtt_alert_msg(ENUM_MQTT_CHANNELS ch_num)
{
//    int 				msg_id;
    int 				len;
	char				charBuffer[MAX_SIZEOF_MQTT_BUF];
	char				*p2Ch_name = MqttNameArry[ch_num];
	
	if (mIsMqttEnabled == 1)
	{
		updateDataBuffer(ch_num, mqtt_databuf); 					//i = channel number
		
		sprintf( charBuffer, "$$mac_address,%s", (char *)mqtt_alert_cfg.client_id );
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "|channel_id,%d", ch_num);
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "|sensor_name,%s", p2Ch_name);
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "|current_value,%.1f", mqtt_databuf[E_CURRENT_VAL] );		// current value
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "|upper_value,%.6f", mqtt_databuf[E_UPPER_BOUND]);
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "|lower_value,%.6f", mqtt_databuf[E_LOWER_BOUND]);
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "|description,%s Anomaly Detected", p2Ch_name);
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "|event_type,%s", "Alert");			// only one type right now
		len = strlen( charBuffer );
		sprintf( &charBuffer[len], "##");
		len = strlen( charBuffer );

		if (len > MAX_SIZEOF_MQTT_BUF) {
			ESP_LOGE(TAG, " *** Exceeded buffer size *** \n");
			return;
		}
		else {
			//xSemaphoreTake(print_mux, portMAX_DELAY);				// don't need to see this any longer
			//printf("\n Buf size: %d\n", len);
			//xSemaphoreGive(print_mux);	
		}

		// For testing the output string...
		//ESP_LOGW(TAG, "Msg: %s", charBuffer); 

		// now publish this message (note that the topic is only used when someone is subscribed to the channel)
		(void)esp_mqtt_client_publish(mqtt_alert_client, "msec", charBuffer, 0, 2, 0);	// Length=0 (publish full string), QOS=2
	}
}	
#endif	// #ifdef MQTT_ALERT_ENABLE


// Send all channels to C2M
static void send_mqtt_msgs()
{
	#ifndef OUTPUT_AI_DATA
	ESP_LOGI(TAG, " *** Sending MQTT channels to C2M");
	#endif
	
	// send CPU PERCENTAGE
	updateDataBuffer(E_MQTT_ID0, mqtt_databuf);
	send_mqtt_msg(E_MQTT_ID0, MqttNameArry[0], (char *)mqtt_cfg.client_id, mqtt_client);
	
	// send PACKETS TRANSMITTED	
	updateDataBuffer(E_MQTT_ID1, mqtt_databuf);
	send_mqtt_msg(E_MQTT_ID1, MqttNameArry[1], (char *)mqtt_cfg.client_id, mqtt_client);
	
	// send PACKETS RECEIBVED
	updateDataBuffer(E_MQTT_ID2, mqtt_databuf);
	send_mqtt_msg(E_MQTT_ID2, MqttNameArry[2], (char *)mqtt_cfg.client_id, mqtt_client);
	
	// send PACKETS DROPPED	
	updateDataBuffer(E_MQTT_ID3, mqtt_databuf);
	send_mqtt_msg(E_MQTT_ID3, MqttNameArry[3], (char *)mqtt_cfg.client_id, mqtt_client);
	
	// send MAC ADDRESS	
	updateDataBuffer(E_MQTT_ID4, mqtt_databuf);
	send_mqtt_msg(E_MQTT_ID4, MqttNameArry[4], (char *)mqtt_cfg.client_id, mqtt_client);
	
}


void mqtt_task(void *arg)
{
	#ifndef OUTPUT_AI_DATA
	#ifdef MQTT_ENABLE
	int					cnt = 0;
	#endif
	#endif

    vTaskDelay((INITIAL_MQTT_TASK_DELAY_SECS * 1000) / portTICK_RATE_MS);	// delay 15 seconds before starting (15000 msec)

    while (1) {
		#ifndef OUTPUT_AI_DATA
		#ifdef MQTT_ENABLE
        ESP_LOGI(TAG, "MQTT TASK loop: %d", cnt++);
		#endif
		#endif		
		
		if (gpio_get_level(GPIO_IS_MQTT_ENABLED) == 1) 
		{
			mintCurrTimeMsec = xTaskGetTickCount();
			
			//ESP_LOGI(TAG, " *** MQTT output ON");
			send_mqtt_msgs();			// send all message with current data
			mIsMqttEnabled = 1;
		}			
		else {
			mIsMqttEnabled = 0;
		}

		vTaskDelay((mintMqttRatems) / portTICK_RATE_MS);	// delay X milliseconds 
    }
    vTaskDelete(NULL);
}

void initMqtt( int rateMsec )
{

	mintMqttRatems = rateMsec;					// update rate in milliseconds	
	mintStartTimeMsec = xTaskGetTickCount();	//xthal_get_ccount();
	mintCurrTimeMsec = xTaskGetTickCount();
	
#ifdef MQTT_ENABLE
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

     // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     // * Read "Establishing Wi-Fi or Ethernet Connection" section in
     // * examples/protocols/README.md for more information about this function.
     
    ESP_ERROR_CHECK(example_connect());

	mqtt_cfg.host = "XXX.XXX.XX.XX";
	mqtt_cfg.port = 0000;
	mqtt_cfg.client_id = "XX:XX:XX:XX:XX:XX";		// C2M
    mqtt_client = mqtt_app_start(&mqtt_cfg); 
	ESP_LOGI(TAG, " *** Client ID: %s", mqtt_cfg.client_id);
	ESP_LOGI(TAG, " *** Client sending to: %s:%d", mqtt_cfg.host, mqtt_cfg.port);
	
	#ifdef MQTT_ALERT_ENABLE
	// when alert channel is setup, uncomment the following and then "publish" to send a message
	mqtt_alert_cfg.host = "XXX.XXX.XX.XX";
	mqtt_alert_cfg.port = 0000;
	mqtt_alert_cfg.client_id = "XX:XX:XX:XX:XX:XX";		// ALERT feed
	mqtt_alert_client = mqtt_app_start(&mqtt_alert_cfg);
	ESP_LOGI(TAG, " *** Alert Client ID: %s", mqtt_alert_cfg.client_id);
	ESP_LOGI(TAG, " *** Alerts sending to: %s:%d", mqtt_alert_cfg.host, mqtt_alert_cfg.port);
	#endif
	
#endif	// #ifdef MQTT_ENABLE
}


#endif	// #ifndef MQTT_C
