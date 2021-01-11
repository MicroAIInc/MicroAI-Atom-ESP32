/* Sensors.c 

	All sensor access code, including Tasks
	
*/
#ifndef SENSORS_C
#define SENSORS_C		// define allowing the header file to identify the module using it

// ------- ADC 
#include "driver/gpio.h"			// added with ADC sample
#include "driver/adc.h"				// added with ADC sample
#include "esp_adc_cal.h"			// added with ADC sample

// ------- I2C
#include "esp_log.h"
#include "driver/i2c.h"
#include "LSM6DS3.h"				// gyro/accel 

#include <math.h>         

#include "esp32ai.h"
#include "sensors.h"


// CONDITIONAL COMPILE OPTIONS (see esp32ai.h)


// External references
extern SemaphoreHandle_t print_mux;
extern SemaphoreHandle_t data_mux;
extern SemaphoreHandle_t mqtt_mux;
 
	
//***************************
// ADC values
//***************************
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NOF_SAMPLES   	500			//0x4000	//64          //Multisampling
static 			esp_adc_cal_characteristics_t 	*adc_chars;
static const 	adc_channel_t 	m_ADC_CHANNEL = ADC_CHANNEL_6;		//GPIO34 if ADC1, GPIO14 if ADC2
//static const 	adc_atten_t 	m_ADC_ATTEN   = ADC_ATTEN_DB_0;		//no attenuation (most accurate for reading full scale between 100 and 950mV)
static const 	adc_atten_t 	m_ADC_ATTEN   = ADC_ATTEN_DB_2_5;		
//static const 	adc_atten_t 	m_ADC_ATTEN   = ADC_ATTEN_DB_6;		
//static const 	adc_atten_t 	m_ADC_ATTEN   = ADC_ATTEN_DB_11;	// reduce the sensitivity by a max amount by applying this attenuation

static const 	adc_unit_t 		m_ADC_UNIT	  = ADC_UNIT_1;

static 			float	 		mfltAdcReading = 0;
//***************************

float CM[2][2] = 		// covariance matrix (for Kalman calculation)
{
	{0.0, 0.0},
	{0.0, 0.0}
};

static float 	mKalmanAngle = 0.0;
static float 	mKalmanBias = 0.0;
static float 	mKalmanRate = 0.0;

static const char *TAG = "ESP32_Sensors";


static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}



void initADC( void )
{
	//*****************************************
	// Set up ADC for measuring sound
	//*****************************************
    
	//Check if Two Point or Vref are burned into eFuse
    check_efuse();
	
    //Configure ADC
    if (m_ADC_UNIT == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(m_ADC_CHANNEL, m_ADC_ATTEN);
    } else {
        adc2_config_channel_atten((adc2_channel_t)m_ADC_CHANNEL, m_ADC_ATTEN);
    }
	
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //esp_adc_cal_value_t val_type = esp_adc_cal_characterize(m_ADC_UNIT, m_ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    (void)esp_adc_cal_characterize(m_ADC_UNIT, m_ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}



/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
/**** disabled along with the Master to Slave test

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
****/

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
/**** disabled along with the Master to Slave test
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
****/



#ifdef TEMP_SENSOR_ENABLE
/**
 * @brief test code to operate on SHT30 sensor (temp and humidity)
 *
 * 1. select register to read (Dummy register in this case, should always read 0x33)
 * _______________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|---------------------|------|
 * 2. wait 5 ms for conversion
 * 3. read data from the registers (6 bytes) - temperature, then humidity
 * _____________________________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read MSB byte + ack | read LSB + ack      | read CRC + ack     |...>
 * --------|---------------------------|---------------------|---------------------|--------------------|
 *                                      _______________________________________________________________________
 *                                     | read MSB byte + ack | read LSB + ack      | read CRC + nack    | stop |  
 *                                     |---------------------|---------------------|--------------------|------|
 */
static esp_err_t i2c_master_sensor_get_temp(i2c_port_t i2c_num, uint16_t *data1, uint16_t *data2)	// gets regnum AND regnum+1 data
{
    int ret;
	uint8_t byteH_1, byteL_1, dummy;
	uint8_t byteH_2, byteL_2;
	i2c_cmd_handle_t cmd;
		
    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(SHT30_READ_TEMP_CMD >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(SHT30_READ_TEMP_CMD & 0x00FF), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error, TempSense Part 1");
        return ret;
    }
    vTaskDelay(50 / portTICK_RATE_MS);			// MUST be more than 10 msec when not clock stretching
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &byteH_1, ACK_VAL);
    i2c_master_read_byte(cmd, &byteL_1, ACK_VAL);	 
    i2c_master_read_byte(cmd, &dummy, ACK_VAL);		// crc here, but not using it
	
    i2c_master_read_byte(cmd, &byteH_2, ACK_VAL);
    i2c_master_read_byte(cmd, &byteL_2, ACK_VAL);	 
    i2c_master_read_byte(cmd, &dummy, NACK_VAL);	// crc here, but not using it	
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	*data1 = (byteH_1 << 8) + byteL_1;
	*data2 = (byteH_2 << 8) + byteL_2;
    if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error, TempSense Part 2");
        return ret;
    }

    return ret;
}
#endif	// #ifdef TEMP_SENSOR_ENABLE



/**
 * @brief test code to operate on SHT30 sensor (temp and humidity)
 *
 * 1. select register to read (Dummy register in this case, should always read 0x33)
 * _________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | write 1 byte + ack  |...>
 * --------|---------------------------|---------------------|---------------------|
 * 2. wait 5 ms (not sure if this is necessary though)
 * 3. read data from the register (1 byte)
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read MSB byte + ack | read LSB + nack    | stop |  (also sends a CRC, but the nack stops that )
 * --------|---------------------------|---------------------|--------------------|------|
 */
/*** not currently used 
static esp_err_t i2c_master_sensor_get_temp_status(i2c_port_t i2c_num, uint16_t u16cmd, uint8_t *data_h, uint8_t *data_l)	// gets regnum AND regnum+1 data
{
    int ret;
	uint8_t crc;
	i2c_cmd_handle_t cmd;
		
    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(u16cmd >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(u16cmd & 0x00FF), ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, 0xF3, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, 0x2D, ACK_CHECK_EN);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error, Status Part 1");
        return ret;
    }
    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SHT30_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, ACK_VAL);	 
    i2c_master_read_byte(cmd, &crc, NACK_VAL);	
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Error, Status Part 2");
        return ret;
    }

    return ret;
}
***/

/**
 * @brief Read an 8 bit register
 *
 * 1. select register to read (Dummy register in this case, should always read 0x33)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait 5 ms (not sure if this is necessary though)
 * 3. read data from the register (1 byte)
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|------|
 */
 /*** not used at the moment, but KEEP IT!
static esp_err_t i2c_master_sensor_get_8bits(i2c_port_t i2c_num, uint8_t sensorAddr, uint8_t sensorRegNum, uint8_t *data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensorAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, sensorRegNum, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensorAddr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
***/

/**
 * @brief Write an 8 bit register
 * _____________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg byte + ack  | write data byte + nack  | stop |
 * --------|---------------------------|-----------------------|-------------------------|------|
 */
static esp_err_t i2c_master_sensor_write_8bits(i2c_port_t i2c_num, uint8_t sensorAddr, uint8_t sensorRegNum, uint8_t byteData)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	
    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensorAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, sensorRegNum, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, byteData, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}



/**
 * @brief Read a 16 bit register
 *
 * 1. select register to read (Dummy register in this case, should always read 0x33)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait 5 ms (not sure if this is necessary though)
 * 3. read data from the register (1 byte)
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_master_sensor_get_2bytes(i2c_port_t i2c_num, uint8_t sensorAddr, uint8_t regnum, uint8_t *data_h, uint8_t *data_l)	// gets regnum AND regnum+1 data
{
    int ret;
	
    vTaskDelay(5 / portTICK_RATE_MS);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensorAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regnum, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensorAddr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);		// low byte comes first on this register
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

// now read the other byte (that works)

    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensorAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regnum+1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(5 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, sensorAddr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    return ret;
}


//---------------------------------------------------------------
// Gyro Control
//---------------------------------------------------------------

static float calcGyro(int16_t input) {
	uint8_t gyroRangeDivisor = 2000 / 125;		//settings.gyroRange / 125;
//	uint8_t gyroRangeDivisor = 245 / 125;		//settings.gyroRange / 125;

    float output = (float)input * 4.375 * (float)gyroRangeDivisor / 1000;

    return output;
}



/**
 * @brief initialize the LSM6DS3 sensor
 *
 *	1. Write to Control Reg #1
 *
 */
void i2c_init_gyro_accel(i2c_port_t i2c_num)
{
    uint8_t dataToWrite;
    int 	ret;
	
    //Setup the accelerometer******************************
    dataToWrite = 0;
    dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;	// 0x02
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;		// 0x0C
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;	// 0x40
	ret = i2c_master_sensor_write_8bits(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
    if (ret != ESP_OK) {
		printf("Problem writing Gyro config register 1\n\n");
    }

    dataToWrite = LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;	// 0x80	// all other bits are 0 at power on
	ret = i2c_master_sensor_write_8bits(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);
    if (ret != ESP_OK) {
		printf("Problem writing Gyro config register 4\n\n");
    }

    dataToWrite = 0;
	// let full scale be the default of 0 = 245 dps
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;		// gyro range, Max deg/s.  Can be: 125, 245, 500, 1000, 2000
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;		// gyro SampleRate, Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	ret = i2c_master_sensor_write_8bits(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);
    if (ret != ESP_OK) {
		printf("Problem writing Gyro config register 2\n\n");
    }

}



/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init( void )
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


/**
 * @brief test function to show buffer
 */
/**** disabled along with the Master to Slave test
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}
****/





#define Q_ANGLE		0.001		// these 3 can also be make into adjustable variables, if needed
#define Q_BIAS		0.003
#define R_MEASURE	0.03

float getKalmanAngle(float newAngle, float newRate, float dt)		// newRate = gyro reading, newAngle = calculated angle (from X & Y accels)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    mKalmanRate = newRate - mKalmanBias;
    mKalmanAngle += dt * mKalmanRate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    CM[0][0] += dt * (dt*CM[1][1] - CM[0][1] - CM[1][0] + Q_ANGLE);
    CM[0][1] -= dt * CM[1][1];
    CM[1][0] -= dt * CM[1][1];
    CM[1][1] += Q_BIAS * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = CM[0][0] + R_MEASURE; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = CM[0][0] / S;
    K[1] = CM[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - mKalmanAngle; // Angle difference
    /* Step 6 */
    mKalmanAngle += K[0] * y;
    mKalmanBias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float CM00_temp = CM[0][0];
    float CM01_temp = CM[0][1];

    CM[0][0] -= K[0] * CM00_temp;
    CM[0][1] -= K[0] * CM01_temp;
    CM[1][0] -= K[1] * CM00_temp;
    CM[1][1] -= K[1] * CM01_temp;

//	ESP_LOGW(TAG, "newAngle=%.3f, newRate=%.3f, mKalmanRate = %.3f, mKalmanAngle = %.3f", newAngle, newRate, mKalmanRate, mKalmanAngle);
	
    return mKalmanAngle;	
}

// gyro_now = kalman.getAngle(sensor.getZRotation(), sensor.readFloatGyroZ(), .05)
#define ADC_VREF		3.3			// is 5.0 for the RPi
#define SENSITIVITY		0.3         // volts/g -> used to calculate the g's from volts (get this from spec sheet?)
//                                      set up for 8g full scale, linear acceleration sensitivity=0.244 mg/LSB
//                                      0.244 mg/LSB 
#define VOLT_CAL_AT_0g	1.0		    // 1.569 on RPi (get this from spec sheet?)
// TODO - read the number of counts being read when at 0g, then use this to subtract from live counts instead of VOLT_CAL_AT_0g
#define MAX_COUNTS      1023.0      // (10 bits)
float getZRotation(float accelX, float accelY)
{
	float xv, yv, angle;
	
	xv = ((accelX / MAX_COUNTS * ADC_VREF) - VOLT_CAL_AT_0g) / SENSITIVITY;	
    //     counts /maxcounts * volts ) - volts@0 ) / 0.3
	yv = ((accelY / MAX_COUNTS * ADC_VREF) - VOLT_CAL_AT_0g) / SENSITIVITY;
	//Calculate the angle between the vectors Y and X. 
	// * 57.2957795 is for conversion from radians to degrees. 
	// +180 is the offset
	angle = (atan2(-yv, -xv) * 57.2957795) + 180;
	
	return angle;
}

									
									
// Read sensor data and return in the structure passed
bool read_sensor_data(SensorChannels_t *p2SensorChannels)
{
	bool		result = true;		
    int 		ret;	//, ii;	
    uint8_t 	sensor_data_h, sensor_data_l;
	int16_t		intSensorData;
	float		fltSensorData;
	float		fltPreviousData;
static TickType_t	xTimePrevious;
    TickType_t  xTimeCurrent;

	// ---------------------------------
	// then read the X-axis accelerometer (execution time: 20 msec, then finally 4 msec)
	// ---------------------------------
	ret = i2c_master_sensor_get_2bytes(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_OUTX_L_XL, &sensor_data_h, &sensor_data_l);
	if (ret == ESP_ERR_TIMEOUT) {
		result = false;			//ESP_LOGE(TAG, "I2C Timeout");
	} else if (ret == ESP_OK) {
		intSensorData = (sensor_data_h << 8 | sensor_data_l);	// ---- --xx xxxx xxxx  10 bit data, but in 16 bit, 2's complement
		intSensorData = intSensorData >> 4;						// 10 bit data, so move it back over (2 less since we have to x4
		fltSensorData = (float)intSensorData;					// convert to float
        
        p2SensorChannels->data[E_ACCEL_X] = fltSensorData;
	} else {
		result = false;			//ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
	}
	
	// ---------------------------------
	// then read the Y-axis accelerometer (execution time: 20 msec, then finally 4 msec)
	// ---------------------------------
	ret = i2c_master_sensor_get_2bytes(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_OUTY_L_XL, &sensor_data_h, &sensor_data_l);
	if (ret == ESP_ERR_TIMEOUT) {
		result = false;			//ESP_LOGE(TAG, "I2C Timeout");
	} else if (ret == ESP_OK) {
		intSensorData = (sensor_data_h << 8 | sensor_data_l);
		intSensorData = intSensorData >> 4;						// 10 bit data, so move it back over (2 less since we have to x4
		fltSensorData = (float)intSensorData;					// convert to float
		p2SensorChannels->data[E_ACCEL_Y] = fltSensorData;
	} else {
		result = false;			//ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
	}

#ifdef READ_ALL_SENSORS			// else this one is not read
	// ---------------------------------
	// then read the Z-axis accelerometer
	// ---------------------------------
	ret = i2c_master_sensor_get_2bytes(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_OUTZ_L_XL, &sensor_data_h, &sensor_data_l);
	if (ret == ESP_ERR_TIMEOUT) {
		result = false;			//ESP_LOGE(TAG, "I2C Timeout");
	} else if (ret == ESP_OK) {
		intSensorData = (sensor_data_h << 8 | sensor_data_l);
		intSensorData = intSensorData >> 4;						// 10 bit data, so move it back over (2 less since we have to x4
		fltSensorData = (float)intSensorData;
		p2SensorChannels->data[E_ACCEL_Z] = fltSensorData;
	} else {
		result = false;			//ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
	}
#endif

#ifdef READ_ALL_SENSORS			// else this one is not read
	// ---------------------------------
	// then read the X-axis gyro
	// ---------------------------------
	ret = i2c_master_sensor_get_2bytes(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_OUTX_L_G, &sensor_data_h, &sensor_data_l);
	if (ret == ESP_ERR_TIMEOUT) {
		result = false;			//ESP_LOGE(TAG, "I2C Timeout");
	} else if (ret == ESP_OK) {
		intSensorData = (sensor_data_h << 8 | sensor_data_l);
		fltSensorData = calcGyro(intSensorData);
		p2SensorChannels->data[E_GYRO_X] = fltSensorData;
	} else {
		result = false;			//ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
	}
#endif

#ifdef READ_ALL_SENSORS			// else this one is not read
	// ---------------------------------
	// then read the Y-axis gyro
	// ---------------------------------
	ret = i2c_master_sensor_get_2bytes(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_OUTY_L_G, &sensor_data_h, &sensor_data_l);
	if (ret == ESP_ERR_TIMEOUT) {
		result = false;			//ESP_LOGE(TAG, "I2C Timeout");
	} else if (ret == ESP_OK) {
		intSensorData = (sensor_data_h << 8 | sensor_data_l);
		fltSensorData = calcGyro(intSensorData);
		p2SensorChannels->data[E_GYRO_Y] = fltSensorData;
	} else {
		result = false;			//ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
	}
#endif
	
	// ---------------------------------
	// then read the Z-axis gyro (execution time: 20 msec, then finally 4 msec)
	// ---------------------------------
	ret = i2c_master_sensor_get_2bytes(I2C_MASTER_NUM, LSM6DS3_SENSOR_ADDR, LSM6DS3_ACC_GYRO_OUTZ_L_G, &sensor_data_h, &sensor_data_l);
	if (ret == ESP_ERR_TIMEOUT) {
		result = false;			//ESP_LOGE(TAG, "I2C Timeout");
	} else if (ret == ESP_OK) {
		intSensorData = (sensor_data_h << 8 | sensor_data_l);
		fltSensorData = calcGyro(intSensorData);
		p2SensorChannels->data[E_GYRO_Z] = fltSensorData;
	} else {
		result = false;			//ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
	}

#ifdef TEMP_SENSOR_ENABLE
	uint16_t	u16SensorData1, u16SensorData2;
	// ---------------------------------
	// then read the temperature from the SHT30 sensor (execution time: 55 msec, then finally 16 msec)
	// ---------------------------------
	ret = i2c_master_sensor_get_temp(I2C_MASTER_NUM, &u16SensorData1, &u16SensorData2);
	if (ret == ESP_ERR_TIMEOUT) {
		result = false;			//ESP_LOGE(TAG, "I2C Timeout - SHT30");
	} else if (ret == ESP_OK) {
		//fltSensorData = ((((float)u16SensorData1) / 65535) * 175 ) -45;		// degrees C
		fltSensorData = ((((float)u16SensorData1) / 65535) * 315 ) -49;			// degrees F
		p2SensorChannels->data[E_TEMP] = fltSensorData;
		fltSensorData = (((float)u16SensorData2) / 65535) * 100;
		p2SensorChannels->data[E_HUMID] = fltSensorData;
	} else {
		result = false;			//ESP_LOGW(TAG, "%s: No ack, sensor SHT30 not connected...skip...", esp_err_to_name(ret));
	}
#else
		p2SensorChannels->data[E_TEMP] = 40.0;
		p2SensorChannels->data[E_HUMID] = 10.0;
#endif

	// elapsed time (this routine up to here): 28 msec (ticks) after streamlining the I2C routines
	
	
	// --------------------------------------------------------------------------------
	// final calculations: "gyro_now and gyro", a.k.a. E_ROTATE_Z and E_ROTATE_Z_DIFF
	// --------------------------------------------------------------------------------

	fltSensorData = getZRotation(	p2SensorChannels->data[E_ACCEL_X],
									p2SensorChannels->data[E_ACCEL_Y]);
								 
    // determine how long it's been since the last calculation (for Kalman filtering)	
	xTimeCurrent = xTaskGetTickCount();

//TODO - for the moment, let's just use the Zrotation calculation (combination of AccelX and AccelY)
// hang on, not just yet...
	fltSensorData = getKalmanAngle(	fltSensorData, 
									p2SensorChannels->data[E_GYRO_Z],
									//1.2);
                                    //1.0);	            // 1.0 because we are filtering 1/sec
                                    //0.05);	        // 50 msec per sample, or 20/sec
									//0.08);            // 80 msec per sample 
                                    (float)(xTimeCurrent-xTimePrevious)/1000);     // pass the value in seconds
	//ESP_LOGW(TAG, "Kalman timing: %5.2f secs", (float)(xTimeCurrent-xTimePrevious)/1000); // use this to view the time between cycles
	xTimePrevious = xTimeCurrent;
                                  
	fltPreviousData = p2SensorChannels->data[E_ROTATE_Z];						// before updating, save the previous
	p2SensorChannels->data[E_ROTATE_Z] = fltSensorData;
	p2SensorChannels->data[E_ROTATE_Z_DIFF] = fltPreviousData - fltSensorData;	// determine difference from last calc		

	// ---------------------------------
	// then read the sound sensor (read from the sound sensor task)
	// ---------------------------------
	xSemaphoreTake(data_mux, portMAX_DELAY);
	p2SensorChannels->data[E_SOUND] = mfltAdcReading;	
	xSemaphoreGive(data_mux);			

	return (result);	
}

// Task to read the sound sensor continuously and update the reading every x number of samples
void sound_sensor_task(void *arg)
{
	uint32_t 	adc_reading;
	uint32_t	ulTemp, ulTempHigh;
	
    while (1) {
        //ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
		// ---------------------------------
		// then update the current sensor value, filtered
		// ---------------------------------
		adc_reading = 0;
		ulTempHigh = 0;

		for (int i = 0; i < 101; i++) 	
		{
			ulTemp = adc1_get_raw((adc1_channel_t)m_ADC_CHANNEL);
			ulTemp = ulTemp >> 2;					// reduce dither by 2 bits
			adc_reading += ulTemp;
			if (ulTemp > ulTempHigh)
				ulTempHigh = ulTemp;				// save the highest value to filter out
		}

		adc_reading -= ulTempHigh;					// remove the highest reading (noise?)
		adc_reading /= 100;							// divide by the number of samples -1	
		// make readings available to other tasks
		xSemaphoreTake(data_mux, portMAX_DELAY);
        mfltAdcReading = (float)adc_reading * 0.1 + mfltAdcReading * 0.9;
		xSemaphoreGive(data_mux);			
			
		xSemaphoreTake(mqtt_mux, portMAX_DELAY);
		gIsReadyForMQTT = 1;					// used to synchronize the MQTT transmissions (because of noise generated during Tx)
		xSemaphoreGive(mqtt_mux);			

		vTaskDelay(SOUND_TASK_LOOP_DELAY_MS / portTICK_RATE_MS);		// see esp32ai.h
    }
    vSemaphoreDelete(print_mux);
    vSemaphoreDelete(data_mux);
    vTaskDelete(NULL);
}


#endif	// #ifndef SENSORS_C
