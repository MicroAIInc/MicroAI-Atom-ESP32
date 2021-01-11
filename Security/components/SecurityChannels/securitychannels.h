/* 
	securitychannels.h
*/
typedef enum {

	E_AI_CH_0,		// 0
	E_AI_CH_1,		// 1	
	E_AI_CH_2,	    // 2	
	E_AI_CH_3,		// 3
	E_AI_CH_4,	// 4	
	
	NOF_AI_CHANNELS			// 5
} ENUM_AI_CHANNELS_USED; 
			
typedef enum {

	E_CH_0,		// 0
	E_CH_1,		// 1	
	E_CH_2,	    // 2	
	E_CH_3,		// 3
	E_CH_4,	// 4	
	
	NOF_SEC_CHANNELS			// 5
} ENUM_SEC_CHANNELS_USED; 

#define TOTAL_CHANNELS 		NOF_AI_CHANNELS

typedef struct {
  float    data[NOF_SEC_CHANNELS];
} SecurityChannels_t;	


bool read_network_data(SecurityChannels_t *security_array);
char * GetSecurityLibVersion(void);	// returns null-terminated string pointer
