/* esp32ai.h
	Define global variables here
*/

#ifdef EXTERN		// have to undef before re-defining
#undef EXTERN
#endif

#ifdef ESP32AI_C
	#define EXTERN
#else
	#define EXTERN extern
#endif


// ------- GPIO
#define GPIO_IS_MQTT_ENABLED	27	// input, GPIO_27
#define GPIO_LEARNING_DISABLED	26	// input, GPIO_26
#define GPIO_INPUT_PIN_SEL   	((1ULL<<GPIO_IS_MQTT_ENABLED) | (1ULL<<GPIO_LEARNING_DISABLED))


EXTERN bool		gIsReadyForMQTT;


// CONDITIONAL COMPILE OPTIONS:
// The following are enabled for full operation
//
//#define MQTT_ENABLE					// disabled when an MQTT connection is not available
//#define MQTT_ALERT_ENABLE				// enable to show alerts on monitor screen, whether MQTT is enabled or not
//#define ENABLE_SENSORS				// if enabled, we read sensors, else they are all zeros.
#define TEMP_SENSOR_ENABLE				// allow disabling if no temperature sensor is attached

// The following are optional (for debugging/testing)
//
#define OUTPUT_AI_DATA					// output the AI engine data for review (using this disables other outputs as well)
//#define CHECK_FOR_NEW_ALARM			// tests the output of the AI engine to see if the alrm[] flag is set and output that data
//#define CALC_TASK_STATS				// calls RTOS function to calculate task stats, i.e. - "Security Metrics:"
//#define PRINT_TASK_STATS				// print specific task stats (relies upon CALC_TASK_STATS being enabled as well)
//#define OUTPUT_METRICS				// calculate and print out the metric data (stack and heap levels, elapsed sec's)
//#define USE_BUILD_MODEL_JUMPER		// if defined, uses the jumper input (IO27 to determine if build is enabled)
//										// Note: this overrides the flag in the config file
//#define MICROAI_DEBUG					// provide debug output for special debug testing
//#define OUTPUT_MODEL_FIRST_LOOP		// output the model the first loop it runs
//#define TEMP_FIXED_CFG_PARAMETERS		// enables hard-coding some of the parameters from the cfg file
//#define OUTPUT_MQTT_DATA_RECD			// outputs the complete data packet to the monitored
//#define READ_ALL_SENSORS				// read all (not necessary for sensors that are ununsed)

 
// The following delays at the end of the task loops are important to consider the timing of data updates
#define MAIN_TASK_LOOP_DELAY_MS			1000	// caution: setting the value much less than sound loop delay may cause repetitive sound values
#define SOUND_TASK_LOOP_DELAY_MS		500		// just prior to beginning the delay, MQTT is enabled
//												   Increase this value to reduce noise error induced by MQTT transmission
//Note: MQTT_TASK_LOOP_DELAY_MS is defined in the configuration file



void updateDataBuffer(int ch_index, float *databuf);
