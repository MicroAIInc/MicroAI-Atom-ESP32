#pragma once
/* AiEngine.h - header file
	
	SDK version
	- max channels		5
	- max runtime		1296000 samples (15 days at 1 sample/sec)
	- max training time	3600 samples (60 minutes at 1 sample/sec)
	
	Version: "00.01.04 - SDK"
*/

#ifdef EXTERN		// have to undef before re-defining
#undef EXTERN
#endif

#ifdef AI_ENGINE_C
#define EXTERN
#else
#define EXTERN extern
#endif

#ifndef uint32_t
#define uint32_t	unsigned int	// 32 bit unsigned integer
#endif

// General config enumeration is used in the "cfg" config file
typedef enum {
	E_IS_BUILD_MODEL,				// The program can run in two modes: 1 = "Build" will train the machine learning model; 
									//		0 = the statistic inference will be executed by using existing ML model. 
	E_IS_DELETE_OLD,				// If set to be 1, old machine learning model files will be deleted and redo the training; 
									//		Only used when in "build" mode.
	E_MEAN_CAL_GAIN,				// (consult OneTech before changing)
	E_STD_CAL_GAIN,					// (consult OneTech before changing)
	E_INITIAL_PERIOD,				// The time window length of the initial period, in counts (before testing for alarm conditions)
	E_SPLIT_COUNTER,				// The length of sub-window, in counts (number of data points used for each training sample)
	E_TRAINING_SECONDS,				// The length of training period, in seconds
	E_MQTT_SAMPLE_RATE_SECS,	 	// Seconds between MQTT samples transmitted
	E_SMALL_THRESHOLD,	 			// Deadzone for upper and lower bound calculation.
									// (when current value, upper, and lower are all very close, this limits false alarms)
	NOF_CONFIG_FILE_ENTRIES
} ENUM_CONFIG;

// Sigma config enumeration is used in the "algPar" config file
typedef enum {
	E_ALARM_SIGMA,					// The threshold defining the "abnormal" state 
	E_ALARM_REPORT_SIGMA,			// The threshold defining the "abnormal and need report" state
	
	NOF_SIGMA_CONFIG_ENTRIES
} ENUM_SIGMA_CONFIG;

// Returns F if any parameters are out of range
int initAI(
	float timeConstantL,		// (consult OneTech before changing)
	float timeConstantS,		// (consult OneTech before changing)
	float tcAlrm,				// (consult OneTech before changing)
	float healthScore,			// Health score parameter
	float dtnp,					// Days to next maintenance parameter
	float normaldtn,			// Normal Days to Next Maintenance
	int totalChannels);			// Total number of channels

void microAI_task(
	int ifPrint,				// = 0 (reserved for future use)
	uint32_t counter,			// counter incremented with each call (used to determine data point quantity for model samples)
	float *signalSource,		// data array, one element per channel (current data)
	float **Cxx,				// 2-dimensional array used for modelX data
	float **Cyy,				// 2-dimensional array used for modelY data
	int rowCx,					// number of rows in the model data (X and Y)
	int lenX,					// number of columns in the modelX data
	int lenY,					// number of columns in the modelY data
	float *signalL,				// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float *signalS,				// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float **cfg,				// configuration file data
	float *X,					// AI variable, must contain memory allocation = 3 * TOTAL_CHANNELS
	int ifBuildModel,			// 0 = run mode, 1 = model training in progress
	float *Cx,					// AI variable, must contain memory allocation = 3 * TOTAL_CHANNELS
	float *Cy,					// AI variable, must contain memory allocation = TOTAL_CHANNELS
	int splitCounter,			// configuration variable indicating the number of data points per model sample
	float *predict,				// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float *prevSignalSource,	// data array, one element per channel (previous data)
	float *err,					// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float *mean,				// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float *std,					// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float *U,					// Upper limit data array, one per channel
	float *L,					// Lower limit data array, one per channel
	int *alrm,					// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float *filteredAlrmDensity,	// AI variable, must contain memory allocation = TOTAL_CHANNELS
	float *healthScore,			// 0 to 100, the calculated health score of the system
	float *alrmSum,				// AI variable, must contain memory allocation = TOTAL_CHANNELS				
	float *daysToNextMaint,		// calculated number of days until next maintenance
	int *alrmReport,			// 0 = no alarm, 1 = alarm condition (one entry per channel)
	float **ratios);			// configuration array of sigma values

char * GetAiEngLibVersion(void);	// returns null-terminated string pointer
