#include "cmsis_os.h"
#include "stm32l1xx_hal.h"
#include "stdio.h"
#include "math.h"
#include <stdlib.h>
#include "ssd1306.h"
#include "bmp280.h"
#include "GPS.h"
#include <string.h>

#define    INTERVAL            1                     // sending data interval = 5 minutes
#define    PMS_WK_TIME        80                     // PMS waking-up time (seconds) = 80-30=50s before reading data
#define    PMS_READ_TIME      30                     // time of reading sensors (seconds) before sending
#define    TIM_TICK           1000                   // Timer tick (miliseconds) = 1s
#define    SYS_CNT_MAX        (TIM_TICK*60*INTERVAL) // 1000*(1ms)*60*INTERVAL = number of minutes/sending data
#define    SYS_CNT_WKPMS      ((TIM_TICK*60*INTERVAL)-(TIM_TICK*PMS_WK_TIME))   // time to wake up PMS sensor, at least 30s before sending data
#define    SYS_CNT_READ       ((TIM_TICK*60*INTERVAL)-(TIM_TICK*PMS_READ_TIME)) // time to read sensors, PMS_READ_TIME from sending
#define    DATA_LEN           100
#define    OLED_OFF           5000                   // 5000*1ms=5s
#define	   _ESP8266_USART			huart1
#define	   _PMS_USART			    huart3
#define	   _BME280_I2C		    hi2c1

#define Enable_Pin GPIO_PIN_14
#define Enable_GPIO_Port GPIOC
#define OLED_WK_Pin GPIO_PIN_0
#define OLED_WK_GPIO_Port GPIOA
#define OLED_WK_EXTI_IRQn EXTI0_IRQn
#define NBIOT_WK_Pin GPIO_PIN_1
#define NBIOT_WK_GPIO_Port GPIOA
#define PMS_SET_Pin GPIO_PIN_4
#define PMS_SET_GPIO_Port GPIOA
#define PMS_RESET_Pin GPIO_PIN_5
#define PMS_RESET_GPIO_Port GPIOA
#define S0_Pin GPIO_PIN_8
#define S0_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_11
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_12
#define S2_GPIO_Port GPIOA

#define NO2 2
#define SO2 3
#define O3 4
#define CO 5

// OLED
void oled_disp(void);
// PMS
void PMS_Init(void);
void sleepPMS(void);
void wakeupPMS(void);
bool readPMS(void);
// BME280
bool BME280_Init(void);
bool readBME280(void);
//GPS
//Send data via UART
bool sendData_NBIOT(void);
//MUX
void mux_enable(void);
void mux_disable(void);
void mux_sel(uint8_t channel);
void clear_buffer(char* receivedata);



extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern uint32_t sys_cnt;
extern uint8_t pms_rx_data_firstbyte;
extern uint8_t pms_rx_data_tmp;
extern uint8_t pms_rx_index;
extern uint8_t pms_rx_data[32];
extern uint16_t cs_cal;
extern uint16_t cs_read;
extern uint16_t cnt;
extern uint8_t pms_data_valid; //0:data invalid; 1: data valid
extern uint8_t send_cnt;
extern uint8_t gps_data_valid; //0:data invalid; 1: data valid
extern BMP280_HandleTypedef bmp280;
extern bool bme280p;
extern float pressure, temperature, humidity,altitude;
extern uint8_t Data[256];
extern uint8_t  oled_cnt_test,led_on,led_off,oled_init;
extern uint32_t oled_cnt;
extern char sendingData[DATA_LEN];
extern uint16_t pms_pm1_0_validdata;
extern uint16_t pms_pm2_5_validdata;
extern uint16_t pms_pm10_0_validdata;

extern uint8_t data_len;
extern char CMD1[]; // for testing only

typedef struct {
  int pm25;
	int pm10;
	int pm01;
	float humid;
	float temp;
	float pressure;
	float longtitude ;
	float lattitude;
	
} data;