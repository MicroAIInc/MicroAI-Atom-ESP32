/* mqtt.h
	
*/

typedef enum {
	E_CURRENT_VAL,			// 0	
	E_UPPER_BOUND,			// 1
	E_LOWER_BOUND,			// 2
	E_URGENT_ALARM,			// 3
	E_HEALTH_SCORE,			// 4
	E_DAYS_TO_MAINT,		// 5
	E_ALARM,				// 6
	
	NOF_MQTT_PARAMS			// 7
} ENUM_MQTT_PARAMS;


// Note: The following enums should match the AI Channels
typedef enum {
	E_MQTT_ID0,				// 0, 
	E_MQTT_ID1,				// 1, 
	E_MQTT_ID2,				// 2, 
	E_MQTT_ID3,				// 3, 
	E_MQTT_ID4,				// 4, 	

	NOF_MQTT_CH_IDS			// 5
} ENUM_MQTT_CHANNELS;


void send_mqtt_alert_msg(ENUM_MQTT_CHANNELS ch_num);
void mqtt_task(void *arg);
void initMqtt( int rateMsec );


