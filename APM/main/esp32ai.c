/* esp32ai.c
 
*/
#ifndef ESP32AI_C
#define ESP32AI_C		

/*	Update Log:

Date		Version		Changes
----		-------		-------
20Oct2020	01.00.00	Initial SDK release

*/

#define	version_str	"01.00.00"		// display at power on, in the command window

// --------------------------------------------------------------------------------------------
// SDK Limitations - these maximum limitations are imposed by the distributed SDK
//	1. Number of Channels 		= 5 (set during initialization and fails if larger)
//	2. Runtime 					= 15 days (1296000 samples at 1 sample/sec) 
//	3. Training Time 			= 60 minutes (3600 samples at 1 sample/sec)
//			(upon expiration, the AI engine returns 0's)
// --------------------------------------------------------------------------------------------


// for CONDITIONAL COMPILE OPTIONS, see esp32ai.h)

			   
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

#include "driver/i2c.h"
#include "AiEngine.h"
#include "sensors.h"
#include "mqtt.h"
#include "esp32ai.h"

static TaskHandle_t 				main_task_handle;
#ifdef ENABLE_SENSORS
static TaskHandle_t 				sensor_task_handle;
#endif
#ifdef MQTT_ENABLE
static TaskHandle_t 				mqtt_task_handle;
#endif

static const char *TAG = "MicroAI_ESP32";


//***************************
// MicroAI variables 
//***************************
				
// MicroAI input channels
typedef enum {
	E_AI_CH_GYRO_NOW,		// 0
	E_AI_CH_GYRO,			// 1	
	E_AI_CH_TEMPERATURE,	// 2	
	E_AI_CH_HUMIDITY,		// 3
	E_AI_CH_SOUND,			// 4	
	
	NOF_AI_CHANNELS			// 5
} ENUM_AI_CHANNELS_USED; 
#define TOTAL_CHANNELS 		NOF_AI_CHANNELS
				
#define maxCSVRowSize 		200	// csv file line size, max

#define TIME_CONSTANT_L 	100	
#define TIME_CONSTANT_S 	20	
#define TIME_CONSTANT_ALRM	90
#define HEALTH_SCORE_P		90	
#define DTNP 				600	// default days to next planned maintenance
#define NORMAL_DTN 			40	// days to normal maintenance for this device

		 
sdmmc_host_t 		host 		= SDMMC_HOST_DEFAULT();
sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
esp_vfs_fat_sdmmc_mount_config_t mount_config = {
	.format_if_mount_failed = false,
	.max_files = 5,
	.allocation_unit_size = 16 * 1024};
sdmmc_card_t 	*card;
esp_err_t 		ret;
FILE 			*f;

char 			*modelFileX = "/sdcard/modelX";		// model file X stored on SD card
char 			*modelFileY = "/sdcard/modelY";		// model file Y stored on SD card
int 			isBuildModel;
int 			deleteOldModel;

uint32_t		u32CycleCount;					// number of cycles of data through AI engine

float 			**cfg;							// pointer to config array read from SD card
int 			alrmReport[TOTAL_CHANNELS];		// channels that currently are in alarm report condition
int 			alrmReportNew[TOTAL_CHANNELS];	// determine which alarms are new
float 			**ratios;
float 			signalSource[TOTAL_CHANNELS];
float 			signalL[TOTAL_CHANNELS]; // long memory
float 			signalS[TOTAL_CHANNELS]; // short memory
float 			prevSignalSource[TOTAL_CHANNELS];
float 			err[TOTAL_CHANNELS];  
float 			mean[TOTAL_CHANNELS]; 
float 			std[TOTAL_CHANNELS]; 
float 			U[TOTAL_CHANNELS];
float 			L[TOTAL_CHANNELS];
int 			alrm[TOTAL_CHANNELS];
int 			alrmNew[TOTAL_CHANNELS];
float 			alrmSum;
float 			filteredAlrmDensity;
float 			healthScore;
float 			daysToNextMaint;
int 			splitCounter;
float 			*predict;
float 			*X;
float 			*Cx;
float 			*Cy;
float			**Cxx;
float			**Cyy;
int 			rowCx, rowCy;			// NOTE: The code expects rowCx to be the same as rowCy!
int 			colCx, colCy;
int 			lenX;
int 			lenY;


#define MIN_MQTT_RATE_SECS			1	// used for validation when reading configuration
#define MAX_MQTT_RATE_SECS			20
#define DEFAULT_MQTT_RATE_SECS		10
int				mintMqttRatems;			// milliseconds between MQTT transmissions

SemaphoreHandle_t print_mux = NULL;
SemaphoreHandle_t data_mux = NULL;
SemaphoreHandle_t mqtt_mux = NULL;

uint8_t	mIsLearningEnabled;
int 	mintCurrTimeMsec, mintStartTimeMsec;	//use for time_span entry to send with MQTT packets
int		mintBeginTimeMsec, mintEndTimeMsec;		// to measure time intervals

#ifdef OUTPUT_METRICS			// test function
static void printHeapSize();
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
            /*		  printf("%d ,  %d ,=%f  \n",i,j,array[i][j]);*/
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
        //		   printf("%s",line);
        while (record != NULL) {
            record = strtok(NULL, ",");
            j = j + 1;
        }
		if (nofCols < j)	// remember the greatest column
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
    char *line, *record = "0";			// initialize to prevent compiler warning

    line = fgets(buf, sizeof(buf), F);
    while (line != NULL) {
        record = strtok(line, delim);
        //		   printf("%s",line);
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
	databuf[E_CURRENT_VAL] 		= signalSource[ch_index];	
	databuf[E_UPPER_BOUND] 		= U[ch_index];
	databuf[E_LOWER_BOUND] 		= L[ch_index];
	databuf[E_URGENT_ALARM] 	= alrmReport[ch_index];
	databuf[E_HEALTH_SCORE] 	= healthScore;
	databuf[E_DAYS_TO_MAINT] 	= daysToNextMaint;
	databuf[E_ALARM] 			= alrmReport[ch_index];
}

#ifdef OUTPUT_METRICS		
static void printHeapSize()
{
		size_t		free_heap = esp_get_free_heap_size();	//xPortGetFreeHeapSize();
		size_t		min_free_heap = xPortGetMinimumEverFreeHeapSize();
static 	uint32_t	prevFreeHeap = 0;

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
//	* init SD card
//		- confirm card is present and writeable
//		- read config files and copy arrays
//	* set up build model, if necessary, including deleting old model
static void initSDcard( void )
{
#ifdef OUTPUT_MODEL_FIRST_LOOP
	static uint8_t 		isFirstLoop = true;		// only output on the first loop
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
	if (rows < NOF_CONFIG_FILE_ENTRIES) { 		//rows = NOF_CONFIG_FILE_ENTRIES;				// minimum size to allocate
        ESP_LOGE(TAG, "Incorrect config file (%d out of %d entries)\n", rows, NOF_CONFIG_FILE_ENTRIES);
		while (1);	// force stop here, else it will fail later
	}
    cfg = alloc2D(rows, cols);
    loadMatrix(cfg, srcFile, ",", 0);			// 0 = do not print these parameters (done later after we validate them)

	#ifdef TEMP_FIXED_CFG_PARAMETERS
		printf("\nPrevious parameters:\n");
		for (i = 0; i < NOF_CONFIG_FILE_ENTRIES; i++) {
			printf("cfg[%d][0] = %f\n", i, cfg[i][0]);
		}

		// then assign any specific parameters for test purposes
		cfg[E_STD_CAL_GAIN][0] = 0.5;
		cfg[E_MEAN_CAL_GAIN][0] = 0.6;
		cfg[E_SMALL_THRESHOLD][0] = 0.1;
	#endif

//===== load sigmaRatios from algPar file
    char *ratioFile = "/sdcard/algPar";
	getRowsCols(ratioFile, &rows, &cols, ",", 1);	// determine how many rows and cols are present
	if (rows < TOTAL_CHANNELS) { 						// minimum size to allocate
        ESP_LOGE(TAG, "Incorrect algPar file (%d out of %d entries)\n", rows, TOTAL_CHANNELS);
		while (1);	// force stop here, else it will fail later
	}

    ratios = alloc2D(rows, cols);
	loadMatrix(ratios, ratioFile, ",", 0);			// 0 = do not print these parameters (done later after we validate them)

	#ifdef TEMP_FIXED_CFG_PARAMETERS
		//	TEST ONLY: Adjust sigma values (hard code for testing)
		ratios[0][0] = 6.0;		//	E_AI_CH_GYRO_NOW,		// 0
		ratios[1][0] = 6.0;		//	E_AI_CH_GYRO,			// 1
		ratios[2][0] = 4.0;		//	E_AI_CH_TEMPERATURE,	// 2
		ratios[3][0] = 4.0;		//	E_AI_CH_HUMIDITY,		// 3
		ratios[4][0] = 8.0;		//	E_AI_CH_SOUND,			// 4
		ratios[0][1] = 6.0;		//	E_AI_CH_GYRO_NOW,		// 0
		ratios[1][1] = 6.0;		//	E_AI_CH_GYRO,			// 1
		ratios[2][1] = 6.0;		//	E_AI_CH_TEMPERATURE,	// 2
		ratios[3][1] = 6.0;		//	E_AI_CH_HUMIDITY,		// 3
		ratios[4][1] = 6.0;		//	E_AI_CH_SOUND,			// 4
	#endif
//=====

    /**SETUP for the main task**/

    isBuildModel = f2i(*cfg[E_IS_BUILD_MODEL]);
	#ifdef USE_BUILD_MODEL_JUMPER
	isBuildModel = mIsLearningEnabled;					// the hardware jumper overrides this now
	#endif
	
    deleteOldModel = f2i(*cfg[E_IS_DELETE_OLD]);
    if ((deleteOldModel == 1) && (isBuildModel == 1)) {
		if (remove(modelFileX)) {
			ESP_LOGI(TAG, "File X removed");
		}
		else {
			f = fopen(modelFileX, "w");					     
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
			f = fopen(modelFileY, "w");					
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
	else if (isBuildModel == 0) 	// normal mode, so load up the model data here
	{	
		getRowsCols(modelFileX, &rowCx, &colCx, ",", 0);	// determine the memory allocation required
		getRowsCols(modelFileY, &rowCy, &colCy, ",", 0);	// determine the memory allocation required
		if (rowCx < 5) 		
			rowCx = 5;						// minimum size to allocate
		if (colCx < lenX) 	
			colCx = lenX;
		
		if (rowCy < 5) 		
			rowCy = 5;					// minimum size to allocate
		if (colCy < lenY) 	
			colCy = lenY;

		if (rowCx < rowCy) {
			rowCx = rowCy;				// allocate memory for the greatest size, if different
			ESP_LOGE(TAG, "File X has less rows than File Y");
		}
		if (rowCy < rowCx) {
			rowCy = rowCx;
			ESP_LOGE(TAG, "File Y has less rows than File X");
		}		
		
		Cxx = alloc2D(rowCx, colCx);		// Cxx = alloc2D(rowCxCy, lenX);		lenX = TOTAL_CHANNELS * 3
        Cyy = alloc2D(rowCy, colCy);		// Cyy = alloc2D(rowCxCy, lenY);

		if (Cxx == NULL)
			ESP_LOGE(TAG, "Cxx is NULL (no memory allocated)");
		if (Cyy == NULL)
			ESP_LOGE(TAG, "Cyy is NULL (no memory allocated)");

		#ifdef OUTPUT_MODEL_FIRST_LOOP
			if (isFirstLoop == true) {
				loadMatrix(Cxx, "/sdcard/modelX", ",", 1);		// special printout case
				loadMatrix(Cyy, "/sdcard/modelY", ",", 1);		
				isFirstLoop = false;
			}
			else {
				loadMatrix(Cxx, "/sdcard/modelX", ",", 0);		// normal case
				loadMatrix(Cyy, "/sdcard/modelY", ",", 0);		
			}
		#else
			loadMatrix(Cxx, "/sdcard/modelX", ",", 0);		// normal case
			loadMatrix(Cyy, "/sdcard/modelY", ",", 0);
		#endif
	}

#ifdef OUTPUT_METRICS
	if (isBuildModel)
		printf(" *** Training Seconds = %d ***\n", (int)cfg[E_TRAINING_SECONDS][0]);
#endif	
		
	// Validate the config file entries

	mintMqttRatems = f2i(cfg[E_MQTT_SAMPLE_RATE_SECS][0]);			// make this conversion one time

	if (mintMqttRatems < MIN_MQTT_RATE_SECS)						// validate the entry
		mintMqttRatems = DEFAULT_MQTT_RATE_SECS;					// if invalid, use the default
	else if (mintMqttRatems > MAX_MQTT_RATE_SECS)
		mintMqttRatems = DEFAULT_MQTT_RATE_SECS;
	cfg[E_MQTT_SAMPLE_RATE_SECS][0] = mintMqttRatems;				// put back in the array for the upcoming print function
	mintMqttRatems *= 1000;											// convert to milliseconds
	
	printf("\nConfig parameters:\n");
	for (i = 0; i < NOF_CONFIG_FILE_ENTRIES; i++) {
		printf("cfg[%d][0] = %.3f\n", i, cfg[i][0]);
	}
	
	// Validate the AlgPar entries here, if desired
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


#ifdef CALC_TASK_STATS
// This example demonstrates how a human readable table of run time stats
// information is generated from raw data provided by uxTaskGetSystemState().
// The human readable table is written to pcWriteBuffer
void local_vTaskGetRunTimeStats( char *pcWriteBuffer )
{
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	uint32_t ulTotalRunTime;	//, ulStatsAsPercentage;
	float fltStatsAsPercentage;		// use a float to display fractional units
	float fltRunTimeTicks;
	size_t heapMin, heapNow;
	uint32_t ulAccumCounter;
	
	// Make sure the write buffer does not contain a string.
	*pcWriteBuffer = 0x00;

	// Take a snapshot of the number of tasks in case it changes while this
	// function is executing.
	uxArraySize = uxTaskGetNumberOfTasks();

	// Allocate a TaskStatus_t structure for each task.  An array could be
	// allocated statically at compile time.
	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL )
	{
		// Generate raw status information about each task.
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

		fltRunTimeTicks = (float)(ulTotalRunTime)/1000000.0;
		sprintf( pcWriteBuffer, "\nSecurity Metrics:\n Runtime = %.1f\n", fltRunTimeTicks);
		pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );

		// For percentage calculations.
		ulTotalRunTime /= 100UL;

		// Avoid divide by zero errors.
		if( ulTotalRunTime > 0 )
		{
			// For each populated position in the pxTaskStatusArray array,
			// format the raw data as human readable ASCII data
			ulAccumCounter = 0;
			for( x = 0; x < uxArraySize; x++ )
			{
				#ifdef PRINT_TASK_STATS				
					// What percentage of the total run time has the task used?
					// This will always be rounded down to the nearest integer.
					// ulTotalRunTimeDiv100 has already been divided by 100.
					fltStatsAsPercentage = (float)pxTaskStatusArray[ x ].ulRunTimeCounter / (float)ulTotalRunTime;		// see in fractional units

					if( fltStatsAsPercentage > 0.1 )
					{
						sprintf( pcWriteBuffer, "%s\t\t%d\t\t%.2f%%\r\n", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter, fltStatsAsPercentage );
					}
					else
					{
						// If the percentage is zero here then the task has
						// consumed less than 1% of the total run time, so don't bother outputting it
						//sprintf( pcWriteBuffer, "%s\t\t%d\t\t<1%%\r\n", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter );
					}

					pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
				#endif	
				
				// now, let's just calculate the needed metrics
				if ((strcasecmp(pxTaskStatusArray[ x ].pcTaskName, "MainTask") == 0) ||
					(strcasecmp(pxTaskStatusArray[ x ].pcTaskName, "MqttTask") == 0) ||
					(strcasecmp(pxTaskStatusArray[ x ].pcTaskName, "wifi") == 0) ||
					(strcasecmp(pxTaskStatusArray[ x ].pcTaskName, "SoundTask") == 0) )
				{
					ulAccumCounter += pxTaskStatusArray[ x ].ulRunTimeCounter;
				}
			}
			fltStatsAsPercentage = (float)ulAccumCounter / (float)ulTotalRunTime;		// see in fractional units
			sprintf( pcWriteBuffer, " CPU = %.2f\r\n", fltStatsAsPercentage );
			pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
		}	
	}
	
	// The array is no longer needed, free the memory it consumes.
	vPortFree( pxTaskStatusArray );

	//----------------------------------
	heapNow = xPortGetFreeHeapSize();
	heapMin = xPortGetMinimumEverFreeHeapSize();
	sprintf( pcWriteBuffer, " Heap ratio = %.5f\r\n", (float)(heapNow-heapMin)/(float)heapMin);		// current value is X percent higher than minimum value
	pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );

	sprintf( pcWriteBuffer, " Processes = %d\n \n", uxArraySize );
	pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
	//----------------------------------

}
#endif	// #ifdef CALC_TASK_STATS


static void main_task(void *arg)
{
    int 				i;
	SensorChannels_t	sensor_array;
	int					intAlarmReport;
	uint32_t			u32RuntimeSeconds;
    int 				shape[2] = {TOTAL_CHANNELS, TOTAL_CHANNELS};
    char 				*modelFileX = "/sdcard/modelX";
    char 				*modelFileY = "/sdcard/modelY";

	#ifdef CHECK_FOR_NEW_ALARM
	int					intAlarm;
	#endif

	#ifdef OUTPUT_AI_DATA
    int 				intTemp;	
	#endif

	#ifdef OUTPUT_METRICS
	uint32_t			ulTemp;
	//TickType_t		xTime1, xTime2;
	//uint32_t			ulTime1=0;
	#endif
	
	#ifdef CALC_TASK_STATS
	char 				charBuffer[400];		// must be cautious with the size here so we don't overrun the buffer or exceed stack size
	#endif
	
	// Initialize variables
	
	alrmSum = 0.0;
	filteredAlrmDensity = 0.0;
	healthScore = 100.0;
	daysToNextMaint = NORMAL_DTN;

	sensor_array.data[E_ROTATE_Z]		= 0;		
	sensor_array.data[E_ROTATE_Z_DIFF]	= 0;
	sensor_array.data[E_TEMP]			= 0;			
	sensor_array.data[E_HUMID]			= 0;				
	sensor_array.data[E_SOUND]			= 0;			

    while (1) 
	{
		#ifdef ENABLE_SENSORS
		if (read_sensor_data(&sensor_array) == true) 		// elapsed time for if(): 29-30 msec
		#else
		if (1)
		#endif
		{
			for (i = 0; i < TOTAL_CHANNELS; i++) {
				prevSignalSource[i] = signalSource[i];					// before updating the signals, save the previous
			}	
			
			// assign the values to be processed by AI engine 
			signalSource[E_AI_CH_GYRO_NOW] 		= sensor_array.data[E_ROTATE_Z];		
			signalSource[E_AI_CH_GYRO] 			= sensor_array.data[E_ROTATE_Z_DIFF];
			signalSource[E_AI_CH_TEMPERATURE] 	= sensor_array.data[E_TEMP];			
			signalSource[E_AI_CH_HUMIDITY] 		= sensor_array.data[E_HUMID];				
			signalSource[E_AI_CH_SOUND] 		= sensor_array.data[E_SOUND];			
		
			u32RuntimeSeconds = xTaskGetTickCount()/1000;		// determine the number of runtime seconds
			
			microAI_task(0, u32CycleCount, signalSource,		// execution time: 1-2 msec
						Cxx, Cyy, rowCx, lenX, lenY,
						signalL, signalS, cfg, X, 
						isBuildModel, Cx, Cy, splitCounter,
						predict, prevSignalSource, err, mean, 
						std, U, L, alrm, &filteredAlrmDensity, 
						&healthScore, &alrmSum, &daysToNextMaint, 
						alrmReport, ratios);

			// test for new alarm condition, then output it via MQTT if true
			intAlarmReport = 0;
			#ifdef CHECK_FOR_NEW_ALARM
			intAlarm = 0;
			#endif
			for (i = 0; i < TOTAL_CHANNELS; i++) {
				intAlarmReport |= alrmReport[i];
				if ((alrmReport[i] > 0) && (alrmReportNew[i] == 0))		// new alarm condition? (only send a message if it is new)
				{
					alrmReportNew[i] = 1;
					#ifdef MQTT_ENABLE 
					#ifdef MQTT_ALERT_ENABLE
						send_mqtt_alert_msg(i);
					#endif
					#endif
				}
				else
					alrmReportNew[i] = 0;

				#ifdef CHECK_FOR_NEW_ALARM
				intAlarm |= alrm[i];
				if ((alrm[i] > 0) && (alrmNew[i] == 0))	
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
					printf("\n *** TRAINING COMPLETE ***");
					printf("\n Power Off, then train again or disable training mode \n\n");
					vTaskDelay(300000 / portTICK_RATE_MS);		// delay 5 minutes
				}
				else if (u32CycleCount % splitCounter == (splitCounter - 1)) {
					shape[0] = 1;								// save every "splitCounter" number of cycles
					shape[1] = TOTAL_CHANNELS * 3;
					saveMatrix(&Cx, modelFileX, ",", shape, 1); // append model rows
					shape[1] = TOTAL_CHANNELS;
					saveMatrix(&Cy, modelFileY, ",", shape, 1); // append model rows
					printf("model rows ADDED \n");
				}
			}
			
			#ifdef OUTPUT_AI_DATA		// for test purposes only (does not output alarm data to the monitor)	
				if ((intAlarmReport > 0) && (isBuildModel == 0)) {
					// Use this to visually see when alarms are present (not good if capturing data to be analyzed)
					ESP_LOGW(TAG, "Alarm REPORT: %d, %d, %d, %d, %d", alrmReport[0],alrmReport[1],alrmReport[2],alrmReport[3],alrmReport[4]);
				}

				intTemp = xTaskGetTickCount();
				if (u32CycleCount == 0)
					printf("count,U_0,L_0,R_0,Time_0,Alrm_0,U_1,L_1,R_1,Time_1,Alrm_1,U_2,L_2,R_2,Time_2,Alrm_2,U_3,L_3,R_3,Time_3,Alrm_3,U_4,L_4,R_4,Time_4,Alrm_4 \n");		// version 2
				printf("%3d", u32CycleCount);
				for (i=0; i< TOTAL_CHANNELS; i++)	
				{									
					#ifdef CHECK_FOR_NEW_ALARM
					printf(", %8.3f, %8.3f, %8.3f, %7.2f, %d", U[i], L[i], signalSource[i], ((float)intTemp)/1000, alrm[i] );
					#else
					printf(", %8.3f, %8.3f, %8.3f, %7.2f, %d", U[i], L[i], signalSource[i], ((float)intTemp)/1000, alrmReport[i] );
					#endif
				}
				printf("\n");
			#endif	// #ifdef OUTPUT_AI_DATA

			#ifdef MQTT_ALERT_ENABLE
				if ((intAlarmReport > 0) && (isBuildModel == 0)) {
					// Use this to visually see when alarms are present
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
			#endif	// #ifdef MQTT_ALERT_ENABLE

			u32CycleCount++;				
		}
		else
		{
			ESP_LOGE(TAG, "*** Read Sensor FAILED ***");
		}

		#ifdef OUTPUT_METRICS		
			mintCurrTimeMsec = xTaskGetTickCount();
			printf("\n *** Elapsed Sec's: %.1f \n", (float)(mintCurrTimeMsec-mintStartTimeMsec)/1000);

			//xTime1 = main_task_handle.ulRunTimeCounter;			
			//xTime2 = xTaskGetTickCount();				
			//ulTime1 = esp_timer_impl_get_time;	
			//ESP_LOGI(TAG, " *** Elapsed Ticks: %d, Stack: %d\n", xTime2-xTime1, ulTime1);

			ulTemp = uxTaskGetStackHighWaterMark(main_task_handle);
			ESP_LOGI(TAG, " *** Min Stack level: %d", ulTemp);

			printHeapSize();
			
		#endif
		
		#ifdef CALC_TASK_STATS
			local_vTaskGetRunTimeStats(charBuffer);		// CAUTION: buffer size must be sufficient to hold all printable task info
			xSemaphoreTake(print_mux, portMAX_DELAY);			
			printf( charBuffer);
			xSemaphoreGive(print_mux);			
		#endif		
		
		vTaskDelay(MAIN_TASK_LOOP_DELAY_MS / portTICK_RATE_MS);		// see esp32ai.h for the current delay, in msec

    } // while (1)

    vSemaphoreDelete(print_mux);
    vSemaphoreDelete(data_mux);
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

	// set up to look for enabled MQTT output and MicroAI Learning (via jumper wires)
	initGPIO();									
	if (gpio_get_level(GPIO_IS_MQTT_ENABLED) == 1) 
	{
		printf("\n *** MQTT output ON\n\n");		
	}
	else 
	{
		#ifdef MQTT_ENABLE		// no need to show this if we are always disabled
		printf("\n *** MQTT output Off (remove jumper from IO27 to enable)\n\n");		
		#endif
	}
	
	#ifdef USE_BUILD_MODEL_JUMPER		// this overrides the config file flag
		if (gpio_get_level(GPIO_LEARNING_DISABLED) == 0)
		{
			// Test a 2nd time before setting this active
			if (gpio_get_level(GPIO_LEARNING_DISABLED) == 0) {
				printf(" *** Learn Mode: ON\n\n");
				mIsLearningEnabled = true;
				vTaskDelay(5000);
			}
		}
		else
		{
			printf(" *** Learn Mode: Off (connect jumper from GND to IO26 to enable, then reboot)\n\n");
			mIsLearningEnabled = false;
		}
	#endif

	mintStartTimeMsec = xTaskGetTickCount();


	//*****************************************
	// Set up ADC for measuring sound
	//*****************************************
    initADC();

	//*****************************************
	// Set up configuration
	//*****************************************
	initSDcard();		// confirm SD card access, read config files (also uses mIsLearningEnabled)

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
	#ifdef MQTT_ENABLE
	initMqtt(mintMqttRatems);
    #endif //MQTT_ENABLE


	//*****************************************
	// Then set up the rest 
	//*****************************************	
    print_mux = xSemaphoreCreateMutex();		// mutex for sharing printing between 2 tasks
    data_mux = xSemaphoreCreateMutex();	
	mqtt_mux = xSemaphoreCreateMutex();
	
	#ifdef ENABLE_SENSORS
    ESP_ERROR_CHECK(i2c_master_init());			// init the master I2C port
	i2c_init_gyro_accel(I2C_MASTER_NUM);		// init the gyro/accelerometer
	#endif
#ifdef OUTPUT_METRICS
	printHeapSize();
#endif

	//*****************************************
	// show current version 
	//*****************************************	
	printf("\n\n **********************\n");
	printf(    " *\n");
	printf(    " * ESP32 SDK\n");
	printf(    " * Version: %s\n", version_str);
	printf(    " *\n");
	printf(    " * MicroAI Engine\n");
	printf(    " * Version: %s\n", GetAiEngLibVersion());
	printf(    " *\n");
	printf(    " ********************** \n\n\n");

	
	//*****************************************
	// Then start the tasks
	//*****************************************

	//xTaskCreate(function,        	task name,       stack size, params,   priority (0=lowest), handle);
    xTaskCreate(main_task,     		"MainTask", 	   2048 * 2, (void *)0, 10, 				&main_task_handle);

	#ifdef ENABLE_SENSORS
    xTaskCreate(sound_sensor_task, 	"SoundTask", 	   1024 * 2, (void *)1, 10, 				&sensor_task_handle);
	#endif //ENABLE_SENSORS

    #ifdef MQTT_ENABLE
    xTaskCreate(mqtt_task,     		"MqttTask", 	   2048 * 2, (void *)2, 10, 				&mqtt_task_handle);
    #endif //MQTT_ENABLE
}

#endif	// #ifndef MICROAI_C
