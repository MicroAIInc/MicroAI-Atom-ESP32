/* esp32ai.c
*/
#ifndef ESP32AI_C
#define ESP32AI_C        // define allowing the header file to identify the module using it
/*    Update Log:
Date        Version        Changes
02Oct2020    01.00.00    Initial code with integrated static for security implementation
						 Also added a limit to the number of channels and provide a AI engine version number.
*/
#define    version_str    "1.0.0 - Security Monitor"        // display at power on, in the command window
/* --------------------------------------------------------------------------------------------
	SDK Limitations - these maximum limitations are imposed by the distributed SDK
	- max channels		5
	- max runtime		1296000 samples (15 days at 1 sample/sec)
	- max training time	3600 samples (60 minutes at 1 sample/sec)
   --------------------------------------------------------------------------------------------
*/ 
// CONDITIONAL COMPILE OPTIONS (see esp32ai.h)
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <math.h>
// ------- GPIO
#include "driver/gpio.h"
// ------- MQTT
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_log.h"
#include "mqtt_client.h"


#include "AiEngine.h"
#include "mqtt.h"
#include "securitychannels.h"
#include "esp32ai.h"
#include "freertos/task.h"

static TaskHandle_t                 main_task_handle;
# ifdef MQTT_ENABLE
static TaskHandle_t                 mqtt_task_handle;
#endif

static const char *TAG = "MicroAI_ESP32";
// struct for LwIP stats
extern struct stats_ lwip_stats; // found in lwip/stats.h

//***************************
// MicroAI variables
//***************************

// MicroAI input channels
typedef enum {

	E_AI_CH_0,		
	E_AI_CH_1,		
	E_AI_CH_2,	   	
	E_AI_CH_3,		
	E_AI_CH_4,		
	
	NOF_AI_CHANNELS		
} ENUM_AI_CHANNELS_USED; 
#define TOTAL_CHANNELS 		NOF_AI_CHANNELS	

#define maxCSVRowSize         200    // csv file line size
#define TIME_CONSTANT_L       100 // in the unit of the sampleingTime
#define TIME_CONSTANT_S       20
#define TIME_CONSTANT_ALRM    90
#define HEALTH_SCORE_P        90
#define DTNP                  600            // default days to next planned maintenance
#define NORMAL_DTN            40    // 90    // days to normal maintenance

sdmmc_host_t         host         = SDMMC_HOST_DEFAULT();
sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
esp_vfs_fat_sdmmc_mount_config_t mount_config = {
	.format_if_mount_failed = false,
	.max_files = 5,
	.allocation_unit_size = 16 * 1024};
sdmmc_card_t     *card;
esp_err_t         ret;
FILE             *f;
char             *modelFileX = "/sdcard/modelX";
char             *modelFileY = "/sdcard/modelY";
int             isBuildModel;
int             deleteOldModel;
uint32_t        u32CycleCount;
float             **cfg;
int             alrmReport[TOTAL_CHANNELS];
int             alrmReportNew[TOTAL_CHANNELS];    // determine which alarms are new (move out of library)
float             **ratios;
float             signalSource[TOTAL_CHANNELS];
float             signalL[TOTAL_CHANNELS]; // long memory
float             signalS[TOTAL_CHANNELS]; // short memory
float             prevSignalSource[TOTAL_CHANNELS];
float             err[TOTAL_CHANNELS];
float             mean[TOTAL_CHANNELS];
float             std[TOTAL_CHANNELS];
float             U[TOTAL_CHANNELS];
float             L[TOTAL_CHANNELS];
int             alrm[TOTAL_CHANNELS];
int             alrmNew[TOTAL_CHANNELS];    // determine which alarms are new (move out of library)
float             alrmSum;
float             filteredAlrmDensity;
float             healthScore;
float             daysToNextMaint;
int             splitCounter;
float             *predict;
float             *X;
float             *Cx;
float             *Cy;
float            **Cxx;
float            **Cyy;
int             rowCx, rowCy;            // NOTE: The code expects rowCx to be the same as rowCy!
int             colCx, colCy;
int             lenX;
int             lenY;

#define MIN_MQTT_RATE_SECS            1    // used when reading configuration
#define MAX_MQTT_RATE_SECS            20
#define DEFAULT_MQTT_RATE_SECS        10
int                mintMqttRatems;            // milliseconds between MQTT transmissions
SemaphoreHandle_t print_mux = NULL;
SemaphoreHandle_t mqtt_mux = NULL;
uint8_t    mIsLearningEnabled;
int     mintCurrTimeMsec, mintStartTimeMsec;    //use for time_span entry to send with MQTT packets (for now)
int        mintBeginTimeMsec, mintEndTimeMsec;        // to measure time intervals
#ifdef OUTPUT_METRICS
void printHeapSize();
#endif
static void free2D(float **array, int rows);
static float **alloc2D(int m, int n);
static void getRowsCols(char *fileName, int *p2Rows, int *p2Cols, char *delim, int ifPrint);
static void loadMatrix(float **x, char *fileName, char *delim, int ifPrint);
static void saveMatrix(float **x, char *fileName, char *delim, int *shape, int isAppend);
static void set0(float *x, int len) {
	int i;
	for (i = 0; i < len; i++) {
		x[i] = 0.0;
	}
}
static void free2D(float **array, int rows) {
	int ii;
	for (ii=0; ii<rows; ii++) {
		free(array[ii]);
	}
	free(array);
}
static float **alloc2D(int m, int n) {
	/*allocation memory for the 2D array (using **); Initialize them to 0.0*/
	int i, j;
	float **array;
	array = malloc(m * sizeof *array);
	if (array == NULL) {
		fprintf(stderr, "out of memory for 2D array\n");
		#ifdef MICROAI_DEBUG
		printf("\nout of memory for 2D array-1\n");
		#endif
		exit(0);
	}
	for (i = 0; i < m; i++) {
		array[i] = malloc(n * sizeof *array[i]);
		if (array[i] == NULL) {
			fprintf(stderr, "out of memory for 2D array\n");
			#ifdef MICROAI_DEBUG
			printf("\nout of memory for 2D array-2\n");
			#endif
			exit(0);
		}
		for (j = 0; j < n; j++) {
			array[i][j] = 0.0;
			/*          printf("%d ,  %d ,=%f  \n",i,j,array[i][j]);*/
		}
	}
	return array;
}
static void getRowsCols(char *fileName, int *p2Rows, int *p2Cols, char *delim, int ifPrint) {
	/*read number of rows and columns from 2D array from a csv file (or other with different delimiters)*/
	int i = 0;
	int j = 0;
	int nofRows = 0;
	int nofCols = 0;
	*p2Rows = 0;
	*p2Cols = 0;
	FILE *F;
	F = fopen(fileName, "r");
	if (F == NULL) {
		printf("Cannot open the file ");
		printf("%s\n", fileName);
		exit(0);
	}
	char buf[maxCSVRowSize];
	char *line, *record;
	char *temp = "0";
	record = temp;
	line = fgets(buf, sizeof(buf), F);
	while (line != NULL) {
		record = strtok(line, delim);
		//           printf("%s",line);
		while (record != NULL) {
			record = strtok(NULL, ",");
			j = j + 1;
		}
		if (nofCols < j)    // remember the greatest column
			nofCols = j;
		i = i + 1;
		j = 0;
		line = fgets(buf, sizeof(buf), F);
		nofRows++;
	}
	if (ifPrint == 1) {
		printf("Rows: %d, Columns: %d\n", nofRows, nofCols);
	}
	*p2Rows = nofRows;
	*p2Cols = nofCols;
	if (line != NULL) {
		free(line);
	}
	free(record);
	fclose(F);
}
static void loadMatrix(float **x, char *fileName, char *delim, int ifPrint) {
	/*read 2D array from a csv file*/
	int i = 0;
	int j = 0;
	FILE *F;
	F = fopen(fileName, "r");
	if (F == NULL) {
		printf("Cannot open the file ");
		printf("%s", fileName);
		exit(0);
	}
	char buf[maxCSVRowSize];
	char *line, *record = "0";            // initialize to prevent compiler warning
	line = fgets(buf, sizeof(buf), F);
	while (line != NULL) {
		record = strtok(line, delim);
		//           printf("%s",line);
		while (record != NULL) {
			x[i][j] = atof(record);
			record = strtok(NULL, ",");
			if (ifPrint == 1) {
				printf("x[%d][%d] = %f\n", i, j, x[i][j]);
			}
			j = j + 1;
		}
		i = i + 1;
		j = 0;
		line = fgets(buf, sizeof(buf), F);
	}
	free(line);
	free(record);
	fclose(F);
}
static void saveMatrix(float **x, char *fileName, char *delim, int *shape,
				int isAppend) {
	// define a function that transfer 1d array into a csv string which is
	// actually 2d
	FILE *F;
	if (isAppend == 1) {
		F = fopen(fileName, "a");
	} else {
		F = fopen(fileName, "w");
	}
	int i, j;
	char *tmpS = malloc(sizeof *tmpS * maxCSVRowSize);
	for (i = 0; i < shape[0]; i++) {
		for (j = 0; j < shape[1]; j++) {
			if ((j - shape[1]) == -1) {
				free(tmpS);
				tmpS = malloc(sizeof *tmpS * maxCSVRowSize);
				sprintf(tmpS, "%f", x[i][j]);
				fputs(tmpS, F);
				fputs("\n", F);
			} else {
				free(tmpS);
				tmpS = malloc(sizeof *tmpS * maxCSVRowSize);
				sprintf(tmpS, "%f", x[i][j]);
				fputs(tmpS, F);
				fputc(',', F);
			}
		}
	}
	free(tmpS);
	fclose(F);
	// fputs all strings to file
	// add /n at the row end index
	//  return 0;
}
static int f2i(float x) { return (((int)(x + 32768.5)) - 32768); }
void updateDataBuffer(int ch_index, float *databuf)
{
	databuf[E_CURRENT_VAL]         = signalSource[ch_index];
	databuf[E_UPPER_BOUND]         = U[ch_index];
	databuf[E_LOWER_BOUND]         = L[ch_index];
	databuf[E_URGENT_ALARM]     = alrmReport[ch_index];
	databuf[E_HEALTH_SCORE]     = healthScore;
	databuf[E_DAYS_TO_MAINT]     = daysToNextMaint;
	databuf[E_ALARM]             = alrmReport[ch_index];
}

#ifdef OUTPUT_METRICS
void printHeapSize()
{
		size_t        free_heap = esp_get_free_heap_size();    //xPortGetFreeHeapSize();
		size_t        min_free_heap = xPortGetMinimumEverFreeHeapSize();
static     uint32_t    prevFreeHeap = 0;
	xSemaphoreTake(print_mux, portMAX_DELAY);
	printf( "\n *** Free Heap: %d (%d), Min: %d\r\n", (int32_t)free_heap, (int32_t)(free_heap-prevFreeHeap), (int32_t)min_free_heap );
	xSemaphoreGive(print_mux);
	prevFreeHeap = free_heap;
}
#endif
//***************************
// SD card functions
//***************************
// Initialize SD card and read files
//    * init SD card
//        - confirm card is present and writeable
//        - read config files and copy arrays
//    * set up build model, if necessary, including deleting old model
static void initSDcard( void )
{
#ifdef OUTPUT_MODEL_FIRST_LOOP
	static uint8_t         isFirstLoop = true;        // only output on the first loop
#endif
	ESP_LOGI(TAG, "Initializing SD card");
	ESP_LOGI(TAG, "Using SDMMC peripheral");

	// To use 1-line SD mode, uncomment the following line:
	// slot_config.width = 1;
	// GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
	// Internal pull-ups are not sufficient. However, enabling internal pull-ups
	// does make a difference some boards, so we do that here.
	gpio_set_pull_mode(15, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
	gpio_set_pull_mode(2,  GPIO_PULLUP_ONLY); // D0, needed in 4- and 1-line modes
	gpio_set_pull_mode(4,  GPIO_PULLUP_ONLY); // D1, needed in 4-line mode only
	gpio_set_pull_mode(12, GPIO_PULLUP_ONLY); // D2, needed in 4-line mode only
	gpio_set_pull_mode(13, GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes
	// Options for mounting the filesystem.
	// If format_if_mount_failed is set to true, SD card will be partitioned and
	// formatted in case when mounting fails.

	// Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
	// Please check its source code and implement error recovery when developing
	// production applications.
	ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config,
										&mount_config, &card);
	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount filesystem. "
						  "If you want the card to be formatted, set "
						  "format_if_mount_failed = true.");
		} else {
			ESP_LOGE(TAG,
					 "Failed to initialize the card (%s). "
					 "Make sure SD card lines have pull-up resistors in place.",
					 esp_err_to_name(ret));
		}
		return;
	}
	// Card has been initialized, print its properties
	sdmmc_card_print_info(stdout, card);
	// Use POSIX and C standard library functions to work with files.
	// First create a file.
	ESP_LOGI(TAG, "Opening test file");
	f = fopen("/sdcard/hello.txt", "w");
	if (f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for writing");
		return;
	}
	fprintf(f, "Hello %s!\n", card->cid.name);
	fclose(f);
	ESP_LOGI(TAG, "File written");
	lenX = TOTAL_CHANNELS * 3;
	lenY = TOTAL_CHANNELS;
	//// load microAI cfg file;
	//// read cfg
	int i;
	int cols = 1;
	int rows = NOF_CONFIG_FILE_ENTRIES;
	char *srcFile = "/sdcard/cfg";
	getRowsCols(srcFile, &rows, &cols, ",", 1);
	if (rows < NOF_CONFIG_FILE_ENTRIES) {         //rows = NOF_CONFIG_FILE_ENTRIES;                // minimum size to allocate
		ESP_LOGE(TAG, "Incorrect config file (%d out of %d entries)\n", rows, NOF_CONFIG_FILE_ENTRIES);
		while (1);    // force stop here, else it will fail later
	}
	cfg = alloc2D(rows, cols);
	loadMatrix(cfg, srcFile, ",", 0);            // do not print these parameters until we validate them
#ifdef TEMP_FIXED_CFG_PARAMETERS
	printf("\nPrevious parameters:\n");
	for (i = 0; i < NOF_CONFIG_FILE_ENTRIES; i++) {
		printf("cfg[%d][0] = %f\n", i, cfg[i][0]);
	}
	cfg[E_STD_CAL_GAIN][0] = 0.5;
	cfg[E_MEAN_CAL_GAIN][0] = 0.6;
#endif
//===== load sigmaRatios from algPar file
	char *ratioFile = "/sdcard/algPar";
	getRowsCols(ratioFile, &rows, &cols, ",", 1);    // determine how many rows and cols are present
	if (rows < TOTAL_CHANNELS) {                         // minimum size to allocate
		ESP_LOGE(TAG, "Incorrect algPar file (%d out of %d entries)\n", rows, TOTAL_CHANNELS);
		while (1);    // force stop here, else it will fail later - consider testing for columns as well
	}
	ratios = alloc2D(rows, cols);
	loadMatrix(ratios, ratioFile, ",", 0);            // do not print these parameters until we validate them
//=====
	/**SETUP for the main task**/
	isBuildModel = f2i(*cfg[E_IS_BUILD_MODEL]);
	#ifdef USE_BUILD_MODEL_JUMPER
	isBuildModel = mIsLearningEnabled;                    // the hardware jumper overrides this now
	#endif
	deleteOldModel = f2i(*cfg[E_IS_DELETE_OLD]);
	if ((deleteOldModel == 1) && (isBuildModel == 1)) {
		if (remove(modelFileX)) {
			ESP_LOGI(TAG, "File X removed");
		}
		else {
			f = fopen(modelFileX, "w");                           //remove(modelFileX);
			if (f == NULL) {
				ESP_LOGE(TAG, "Failed to open file X for writing");
			}
			else {
				fputc(' ', f);
				fclose(f);
				ESP_LOGI(TAG, "File X erased");
			}
		}
		if (remove(modelFileY)) {
			ESP_LOGI(TAG, "File Y removed");
		}
		else {
			f = fopen(modelFileY, "w");                            //remove(modelFileY);
			if (f == NULL) {
				ESP_LOGE(TAG, "Failed to open file Y for writing");
				return;
			}
			else {
				fputc(' ', f);
				fclose(f);
				ESP_LOGI(TAG, "File Y erased");
			}
		}
	}
	else if (isBuildModel == 0)     // normal mode, so load up the model data here
	{
		//printf("\n*** Before malloc: ");
		//printHeapSize();
		getRowsCols(modelFileX, &rowCx, &colCx, ",", 0);    // determine the memory allocation required
		getRowsCols(modelFileY, &rowCy, &colCy, ",", 0);    // determine the memory allocation required
		if (rowCx < 5)
			rowCx = 5;                        // minimum size to allocate
		if (colCx < lenX)
			colCx = lenX;
		if (rowCy < 5)
			rowCy = 5;                    // minimum size to allocate
		if (colCy < lenY)
			colCy = lenY;
		if (rowCx < rowCy) {
			rowCx = rowCy;                // allocate memory for the greatest size, if different
			ESP_LOGE(TAG, "File X has less rows than File Y");
		}
		if (rowCy < rowCx) {
			rowCy = rowCx;
			ESP_LOGE(TAG, "File Y has less rows than File X");
		}
		Cxx = alloc2D(rowCx, colCx);        // Cxx = alloc2D(rowCxCy, lenX);        lenX = TOTAL_CHANNELS * 3
		Cyy = alloc2D(rowCy, colCy);        // Cyy = alloc2D(rowCxCy, lenY);
		if (Cxx == NULL)
			ESP_LOGE(TAG, "Cxx is NULL (no memory allocated)");
		if (Cyy == NULL)
			ESP_LOGE(TAG, "Cyy is NULL (no memory allocated)");
		#ifdef OUTPUT_MODEL_FIRST_LOOP
			if (isFirstLoop == true) {
				loadMatrix(Cxx, "/sdcard/modelX", ",", 1);        // special printout case
				loadMatrix(Cyy, "/sdcard/modelY", ",", 1);
				isFirstLoop = false;
			}
			else {
				loadMatrix(Cxx, "/sdcard/modelX", ",", 0);        // normal case
				loadMatrix(Cyy, "/sdcard/modelY", ",", 0);
			}
		#else
			loadMatrix(Cxx, "/sdcard/modelX", ",", 0);        // normal case
			loadMatrix(Cyy, "/sdcard/modelY", ",", 0);
		#endif
	}
#ifdef OUTPUT_METRICS
	if (isBuildModel)
		printf(" *** Training Seconds = %d ***\n", (int)cfg[E_TRAINING_SECONDS][0]);
#endif
	// Validate the config file entries
	mintMqttRatems = f2i(cfg[E_MQTT_SAMPLE_RATE_SECS][0]);            // make this conversion one time
	if (mintMqttRatems < MIN_MQTT_RATE_SECS)                        // validate the entry
		mintMqttRatems = DEFAULT_MQTT_RATE_SECS;                    // if invalid, use the default
	else if (mintMqttRatems > MAX_MQTT_RATE_SECS)
		mintMqttRatems = DEFAULT_MQTT_RATE_SECS;
	cfg[E_MQTT_SAMPLE_RATE_SECS][0] = mintMqttRatems;                // put back in the array for the upcoming print function
	mintMqttRatems *= 1000;                                            // convert to milliseconds
	printf("\nConfig parameters:\n");
	for (i = 0; i < NOF_CONFIG_FILE_ENTRIES; i++) {
		printf("cfg[%d][0] = %.3f\n", i, cfg[i][0]);
	}

	// Consider validating the AlgPar entries too...
	printf("\nAlgPar:\n");
	for (i = 0; i < TOTAL_CHANNELS; i++) {
		printf("ratios[%d] = %.1f, %.1f\n", i, ratios[i][0], ratios[i][1]);
	}
}
void initGPIO()
{
	gpio_config_t io_conf;
	// inputs...
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//bit mask of the pins, use GPIO27
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
}

static void main_task(void *arg)
{
	int                 i;
	SecurityChannels_t    security_array;
	int                    intAlarm, intAlarmReport;
	uint32_t            u32RuntimeSeconds;
	int                 shape[2] = {TOTAL_CHANNELS, TOTAL_CHANNELS};
	char                 *modelFileX = "/sdcard/modelX";
	char                 *modelFileY = "/sdcard/modelY";
	#ifdef OUTPUT_METRICS
	TickType_t            xTime1, xTime2;
	uint32_t            ulTime1=0;    //, ulTime2=0;
	uint32_t            ulTemp;
	#endif

	// Initialize variables
	alrmSum = 0.0;
	filteredAlrmDensity = 0.0;
	healthScore = 100.0;
	daysToNextMaint = NORMAL_DTN;
	while (1)
	{
		if (read_network_data(&security_array) == true)         // elapsed time for if(): 29-30 msec
		{
			for (i = 0; i < TOTAL_CHANNELS; i++) {
				prevSignalSource[i] = signalSource[i];                    // before updating the signals, save the previous
			}
			// assign the values to be processed by AI engine
			signalSource[E_AI_CH_0]         = security_array.data[E_CH_0];
			signalSource[E_AI_CH_1]         = security_array.data[E_CH_1];
			signalSource[E_AI_CH_2]         = security_array.data[E_CH_2];
			signalSource[E_AI_CH_3]         = security_array.data[E_CH_3];
			signalSource[E_AI_CH_4]     	= security_array.data[E_CH_4];
	
			u32RuntimeSeconds = xTaskGetTickCount()/1000;        // determine the number of runtime seconds
			microAI_task(0, u32CycleCount, signalSource,        // execution time: 1-2 msec
						Cxx, Cyy, rowCx, lenX, lenY,
						signalL, signalS, cfg, X,
						isBuildModel, Cx, Cy, splitCounter,
						predict, prevSignalSource, err, mean,
						std, U, L, alrm, &filteredAlrmDensity,
						&healthScore, &alrmSum, &daysToNextMaint,
						alrmReport, ratios);
			// test for new alarm condition, then output it via MQTT if true
			intAlarmReport = 0;
			intAlarm = 0;
			for (i = 0; i < TOTAL_CHANNELS; i++) {
				intAlarmReport |= alrmReport[i];
				if ((alrmReport[i] > 0) && (alrmReportNew[i] == 0))        // new alarm condition? (only send a message if it is new)
				{
					alrmReportNew[i] = 1;
					#ifdef MQTT_ALERT_ENABLE
						send_mqtt_alert_msg(i);
					#endif
				}
				else
					alrmReportNew[i] = 0;
				#ifdef CHECK_FOR_NEW_ALARM
				intAlarm |= alrm[i];
				if ((alrm[i] > 0) && (alrmNew[i] == 0))        // new alarm condition? (only send a message if it is new)
				{
					alrmNew[i] = 1;
				}
				#endif
			}
			// check to see if it's time to save training data
			if ((isBuildModel == 1) &&
				(u32CycleCount >= ((int)cfg[E_INITIAL_PERIOD][0])))
			{
				if (u32RuntimeSeconds >= cfg[E_TRAINING_SECONDS][0]) {
					printf("\n *** TRAINING COMPLETE *** \n\n");
					printf("\n Power Off, then train again or disable training mode \n\n");
					vTaskDelay(300000 / portTICK_RATE_MS);		// delay 5 minutes
				}
				else if (u32CycleCount % splitCounter == (splitCounter - 1)) {
					shape[0] = 1;                                // save every "splitCounter" number of cycles
					shape[1] = TOTAL_CHANNELS * 3;
					saveMatrix(&Cx, modelFileX, ",", shape, 1); // append model rows
					shape[1] = TOTAL_CHANNELS;
					saveMatrix(&Cy, modelFileY, ",", shape, 1); // append model rows
					printf("model rows ADDED \n");
				}
			}
			#ifdef OUTPUT_AI_DATA        // for test purposes only (does not output alarm data to the monitor)
			if ((intAlarmReport > 0) && (isBuildModel == 0)) {
				// Use this to visually see when alarms are present (not good if capturing data to be analyzed)
				printf("\n");
				ESP_LOGW(TAG, "Alarm REPORT: %d, %d, %d, %d, %d", alrmReport[0],alrmReport[1],alrmReport[2],alrmReport[3],alrmReport[4]);
				printf("\n Abnormal security behavior");
			}
			else
				printf("\n Normal security behavior");
			if (u32CycleCount == 0)
				//printf("count,U_0,L_0,R_0,Time_0,Alrm_0,U_1,L_1,R_1,Time_1,Alrm_1,U_2,L_2,R_2,Time_2,Alrm_2,U_3,L_3,R_3,Time_3,Alrm_3,U_4,L_4,R_4,Time_4,Alrm_4 \n");        // version 2
			//printf("%3d", u32CycleCount);
			#else    // #ifdef OUTPUT_AI_DATA
			#ifdef MQTT_ALARM_ENABLE
			if ((intAlarmReport > 0) && (isBuildModel == 0)) {
				// Use this to visually see when alarms are present (not good if capturing data to be analyzed)
				ESP_LOGW(TAG, "Alarm: %d, %d, %d, %d, %d", alrmReport[0],alrmReport[1],alrmReport[2],alrmReport[3],alrmReport[4]);
				for (i = 0; i < TOTAL_CHANNELS; i++) {
					printf(" %f ", signalSource[i]);
				}
				printf(" =R=\n");
				for (i = 0; i < TOTAL_CHANNELS; i++) {
					printf(" %f ", U[i]);
				}
				printf(" =U=\n");
				for (i = 0; i < TOTAL_CHANNELS; i++) {
					printf(" %f ", L[i]);
				}
				printf(" =L=\n\n");
			}
			#endif    // #ifdef MQTT_ALARM_ENABLE
			#endif    // #ifdef OUTPUT_AI_DATA
			u32CycleCount++;
		}
		else
		{
			xSemaphoreTake(print_mux, portMAX_DELAY);
			printf("\n*** Read Security Channels FAILED ***\n");
			xSemaphoreGive(print_mux);
		}
		mintCurrTimeMsec = xTaskGetTickCount();
		
		#ifdef OUTPUT_METRICS
			printf("\n *** Elapsed Sec's: %.1f \n", (float)(mintCurrTimeMsec-mintStartTimeMsec)/1000);
			//xTime1 = main_task_handle.ulRunTimeCounter;
			//xTime1 = xTime2;                // remember the previous time    Total loop: 1.2 secs
			//xTime2 = xTaskGetTickCount();    // get the current time
			//ulTime1 = esp_timer_impl_get_time;    //os_cputime_get32();
			// then do something to determine the time consumed and print it out
						ulTemp = uxTaskGetStackHighWaterMark(main_task_handle);
			xSemaphoreTake(print_mux, portMAX_DELAY);
			//printf(" *** Elapsed Ticks: %d, Stack: %d\n", xTime2-xTime1, ulTime1);
			printf(" *** Min Stack level: %d", ulTemp);
			printHeapSize();
		#endif
		vTaskDelay(MAIN_TASK_LOOP_DELAY_MS / portTICK_RATE_MS);        // see esp32ai.h
	} // while (1)
	vSemaphoreDelete(print_mux);
	vSemaphoreDelete(mqtt_mux);
	free2D(Cyy, rowCy);
	free2D(Cxx, rowCx);
	vTaskDelete(NULL);
}
void app_main()
{
	//*****************************************
	// Set up GPIO and show results
	//*****************************************
	initGPIO();                                    // set up to read for enabled MQTT output and MicroAI Learning
	if (gpio_get_level(GPIO_IS_MQTT_ENABLED) == 1) {
		printf("\n *** MQTT output ON\n");
	}
	else {
		printf("\n *** MQTT output Off (remove jumper from IO27 to enable)\n");
	}
	if (gpio_get_level(GPIO_LEARNING_DISABLED) == 0)
	{
		// Test a 2nd time before setting this active
		if (gpio_get_level(GPIO_LEARNING_DISABLED) == 0) {
			printf("\n *** Learn Mode: ON\n\n");
			mIsLearningEnabled = true;
			vTaskDelay(5000);
		}
	}
	else
	{
		printf("\n *** Learn Mode: Off (connect jumper from GND to IO26 to enable, then reboot)\n\n");
		mIsLearningEnabled = false;
	}
	mintStartTimeMsec = xTaskGetTickCount();    //xthal_get_ccount();
	//*****************************************
	// Set up configuration
	//*****************************************
	initSDcard();        // confirm SD card access, read config files (also uses mIsLearningEnabled)
	//*****************************************
	// Set up MicroAI function
	//*****************************************
	if (initAI( TIME_CONSTANT_L, TIME_CONSTANT_S, TIME_CONSTANT_ALRM,
				HEALTH_SCORE_P, DTNP, NORMAL_DTN, TOTAL_CHANNELS) == 0)
	{
		printf("\n\n\n *** AI init failed *** \n\n\n");
		vTaskDelay(5000);
	}
	X = malloc(sizeof *X * (3 * TOTAL_CHANNELS));
	Cx = malloc(sizeof *X * (3 * TOTAL_CHANNELS));
	Cy = malloc(sizeof *X * (TOTAL_CHANNELS));
	splitCounter = f2i(cfg[E_SPLIT_COUNTER][0]);
	predict = malloc(sizeof *predict * (TOTAL_CHANNELS));
	set0(X, TOTAL_CHANNELS * 3);
	set0(Cx, TOTAL_CHANNELS * 3);
	set0(Cy, TOTAL_CHANNELS);
	set0(predict, TOTAL_CHANNELS);
	set0(err, TOTAL_CHANNELS);
	set0(mean, TOTAL_CHANNELS);
	set0(std, TOTAL_CHANNELS);
	set0(U, TOTAL_CHANNELS);
	set0(L, TOTAL_CHANNELS);
	for (int ii = 0; ii < TOTAL_CHANNELS; ii++) {
		std[ii] = 0;
		alrm[ii] = 0;
		alrmReport[ii] = 0;
		alrmReportNew[ii] = 0;
	}
	//*****************************************
	// Set up MQTT
	//*****************************************
	# ifdef MQTT_ENABLE
	initMqtt(mintMqttRatems);
	#endif //MQTT_ENABLE
	//*****************************************
	// Then set up the rest
	//*****************************************
	print_mux = xSemaphoreCreateMutex();        // mutex for sharing printing between 2 tasks
	mqtt_mux = xSemaphoreCreateMutex();
#ifdef OUTPUT_METRICS
	printHeapSize();
#endif
	//*****************************************
	// show current version
	//*****************************************
	printf("\n\n **********************\n");
	printf(    " *\n");
	printf(    " * ESP32\n");
	printf(    " * Version: %s\n", version_str);
	printf(    " *\n");
	printf(    " * MicroAI Engine\n");
	printf(    " * Version: %s\n", GetAiEngLibVersion());
	printf(    " *\n");
	printf(    " * SecurityLibrary\n"); 
	printf(    " * Version: %s\n", GetSecurityLibVersion());

	printf(    " *\n");
	printf(    " ********************** \n\n\n");
	
	# ifndef MQTT_ENABLE
	//***************************************************
	// TCP serer initialization and establish connection
	//***************************************************
	ESP_ERROR_CHECK(nvs_flash_init());
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	/* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
	 * Read "Establishing Wi-Fi or Ethernet Connection" section in
	 * examples/protocols/README.md for more information about this function.
	 */
	ESP_ERROR_CHECK(example_connect());
	
	#endif //MQTT_ENABLE
	//*****************************************
	// Then start the tasks
	//*****************************************
	//xTaskCreate(function,            task name,       stack size, params,   priority (0=lowest), handle);
	xTaskCreate(main_task,             "MainTask",        2048 * 2, (void *)0, 10,                 &main_task_handle);
	# ifdef MQTT_ENABLE
	xTaskCreate(mqtt_task,             "MqttTask",        2048 * 2, (void *)2, 10,                 &mqtt_task_handle);
	#endif //MQTT_ENABLE
}
#endif    // #ifndef MICROAI_C