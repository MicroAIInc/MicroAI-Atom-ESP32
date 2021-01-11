/* sensors.h
	
*/


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// Defines for the LIS3DH accelerometer...
#define LIS3DH_SENSOR_ADDR			0x18			/* I2C address of this board */
#define LIS3DH_REG_DUMMY			0x0F			/* Register definitions */
#define LIS3DH_DUMMY_READING		0x33
#define LIS3DH_REG_CNTL_1			0x20			/* Control Reg 1 - adjust the Data Rate (from 1 Hz to 1344 Hz, in Normal mode) */
#define LIS3DH_REG_ACCEL_X_LOW		0x28
#define LIS3DH_REG_ACCEL_X_HIGH 	0x29
#define LIS3DH_REG_ACCEL_Y_LOW		0x2A
#define LIS3DH_REG_ACCEL_Y_HIGH 	0x2B
#define LIS3DH_REG_ACCEL_Z_LOW		0x2C
#define LIS3DH_REG_ACCEL_Z_HIGH 	0x2D

// Defines for the SHT30 temp/humidity sensor...
#define SHT30_SENSOR_ADDR			0x45			/* I2C address of this board (0x44 optional) */
#define SHT30_READ_STATUS_CMD		0xF32D
#define SHT30_READ_TEMP_CMD			0x2400			// High Repeatability, ClkStretch Disabled

// Defines for the LSM6DS3 gyro/accelerometer...
#define LSM6DS3_SENSOR_ADDR			0x6A			/* I2C address of this board */
#define LSM6DS3_REG_DUMMY			0x0F			/* Register definitions */
#define LSM6DS3_DUMMY_READING		0x69
#define LSM6DS3_REG_ACCEL_X_LOW		0x28
#define LSM6DS3_REG_ACCEL_X_HIGH 	0x29
#define LSM6DS3_REG_ACCEL_Y_LOW		0x2A
#define LSM6DS3_REG_ACCEL_Y_HIGH 	0x2B
#define LSM6DS3_REG_ACCEL_Z_LOW		0x2C
#define LSM6DS3_REG_ACCEL_Z_HIGH 	0x2D

//#define DUMMY_READING			LSM6DS3_DUMMY_READING

typedef enum {
	E_ACCEL_X,
	E_ACCEL_Y,
	E_ACCEL_Z,
	E_GYRO_X,
	E_GYRO_Y,
	E_GYRO_Z,
	E_SOUND,
	E_TEMP,
	E_HUMID,
	
	E_ROTATE_Z,						// combination of Accel X, Y, and Gyro Z  (no Gyro right now)
	E_ROTATE_Z_DIFF,
	
	NOF_SENSOR_CHANNELS
} SensorChannels_enum;

typedef struct {
  float	data[NOF_SENSOR_CHANNELS];
} SensorChannels_t;


esp_err_t i2c_master_init( void );
void initADC( void );
void i2c_init_gyro_accel(i2c_port_t i2c_num);
bool read_sensor_data(SensorChannels_t *p2SensorChannels);
void sound_sensor_task(void *arg);


