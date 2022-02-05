#include "cmsis_os.h"
#include "stm32l1xx_hal.h"
#include "stdio.h"
#include "math.h"
#include <stdlib.h>
#include "ssd1306.h"
#include "bmp280.h"
#include "GPS.h"
#include <string.h>
#include "AQI_thread.h"

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
uint32_t sys_cnt=0;
uint8_t pms_rx_data_firstbyte=0;
uint8_t pms_rx_data_tmp=0;
uint8_t pms_rx_index=1;
uint8_t pms_rx_data[32]={0};
uint16_t cs_cal=0;
uint16_t cs_read=0;
uint16_t cnt=0;
uint8_t pms_data_valid=0; //0:data invalid; 1: data valid
uint8_t send_cnt=0;
uint8_t gps_data_valid=0; //0:data invalid; 1: data valid
BMP280_HandleTypedef bmp280;
bool bme280p;
float pressure=0, temperature=0, humidity=0,altitude=0;
uint8_t Data[256];
uint8_t  oled_cnt_test=0,led_on=0,led_off=0,oled_init=1;
uint32_t oled_cnt=0;
char sendingData[DATA_LEN]={0};
uint16_t pms_pm1_0_validdata;
uint16_t pms_pm2_5_validdata;
uint16_t pms_pm10_0_validdata;

uint8_t data_len=0;
char CMD1[] = "%A%B%C%D%E%F%G%H%I%*"; // for testing only

extern data *message_pointer;


void clear_buffer(char* receivedata){
	for(int i=0;i< DATA_LEN ;i++){
		*(&receivedata+i) = 0;
	}
}

// PMS
void PMS_Init(void) {
	HAL_GPIO_WritePin(PMS_RESET_GPIO_Port,PMS_RESET_Pin,GPIO_PIN_SET); // not reset
	HAL_GPIO_WritePin(PMS_SET_GPIO_Port,PMS_SET_Pin,GPIO_PIN_SET);     // working mode
}

void sleepPMS(void) {
	HAL_GPIO_WritePin(PMS_SET_GPIO_Port,PMS_SET_Pin,GPIO_PIN_RESET);   // enter sleep mode
}

void wakeupPMS(void) {
	HAL_GPIO_WritePin(PMS_SET_GPIO_Port,PMS_SET_Pin,GPIO_PIN_SET );    // quit sleep mode
}

bool readPMS(void){
	int start = HAL_GetTick();
	int end = start;
	HAL_UART_Receive(&_PMS_USART,&pms_rx_data_firstbyte,1,1000);
		
	while((pms_rx_data_firstbyte != 0x42)&(end-start<4000)){
		HAL_UART_Receive(&_PMS_USART,&pms_rx_data_firstbyte,1,1000);
		end=HAL_GetTick();
	}
	if(pms_rx_data_firstbyte != 0x42){
		return 0;
	}
	
	if(pms_rx_data_firstbyte==0x42) {
	  pms_rx_data[0]=pms_rx_data_firstbyte;
	}
	
	pms_rx_data_firstbyte=0;
	
	while((pms_rx_index<32)&(end-start<8000))
	{
	   HAL_UART_Receive(&_PMS_USART,&pms_rx_data_tmp,1,2000);
		 if(pms_rx_data_tmp!=0x42) {
	     pms_rx_data[pms_rx_index]=pms_rx_data_tmp;
			 pms_rx_index++;
		 }
		 end=HAL_GetTick();
	}
	if(pms_rx_index<32){
		return 0;
	}
	
	pms_rx_index=1;
	
	cs_read = ((uint16_t)pms_rx_data[30] << 8) | (uint16_t)pms_rx_data[31];
	
	cs_cal=0;
	
	for(int i=0;i<30;i++)
	{
	   cs_cal += pms_rx_data[i];
	}	
	
	if (cs_read==cs_cal) {
	  pms_data_valid=1;
		return 1;
	}
	else {
	  pms_data_valid=0;
		return 0;
	}
	
	
}

//BME280
bool BME280_Init(void) {
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &_BME280_I2C;
	int i =0;
	while (!bmp280_init(&bmp280, &bmp280.params)) {
		osDelay(500);
		i++;
		if(i==6){return 0;}
	}
	
	bme280p = bmp280.id == BME280_CHIP_ID;
	return 1;
}

bool readBME280(void) {
  int i=0;
  osDelay(100);
	while (!bmp280_read_float(&bmp280, &(message_pointer->temp), &(message_pointer->pressure), &(message_pointer->humid))) {		
	  osDelay(500);
		i++;
		if(i==6) return 0;
	}
	if (bme280p) {	
		return true;	
	}
	
	else {
		return false;
	}
			
}

//Send data
bool sendData_NBIOT(void) {
	char receivedata[100];
	send_cnt++;
	uint16_t pm_1_0  = message_pointer->pm01;
	uint16_t pm_2_5  = message_pointer->pm25;
	uint16_t pm_10_0 = message_pointer->pm10;

	
	sprintf(sendingData,"AT+CSOSEND=0,0,\"node16hum%dpres%dtemp%dpm1%dpm25%dpm10%dlong%lflat%lf\"\r",
	                     (int)(message_pointer->humid),(int)message_pointer->pressure,(int)(message_pointer->temp),(int)pm_1_0,(int)pm_2_5,(int)pm_10_0,
												 (float)GPS.GPGGA.LongitudeDecimal,
												 (float)GPS.GPGGA.LatitudeDecimal);
	

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);											 
	osDelay(1000);										 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  osDelay(1000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  osDelay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	osDelay(7000);
	char AT[]      = "AT\r";
	char ATCSOC[]  = "\rAT+CSOC=1,1,1\r";
	char ATCSOCON[] =	"\rAT+CSOCON=0,2795,\"123.21.107.92\"\r";								 
	char ATCSOCL[] =  "\rAT+CSOCL=0\r";
												 
//	HAL_UART_Transmit(&huart2,(uint8_t *)ATCSOCL,sizeof(ATCSOCL),2000);
//	osDelay(2000);										 
	for(int i=0;i< DATA_LEN ;i++){
		receivedata[i] = 0;
	}
	data_len=strlen(sendingData);
	HAL_UART_Transmit(&huart2,(uint8_t *)AT,3,2000);
	HAL_UART_Receive(&huart2,(uint8_t *)receivedata,9,200);
	for(int i=0; i<=3;i++){
		HAL_UART_Transmit(&huart2,(uint8_t *)AT,3,2000);
		HAL_UART_Receive(&huart2,(uint8_t *)receivedata,70,200);
		if(strstr(receivedata,"OK")==NULL){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
			osDelay(500);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
			if(i==3){
				return 0;}
		}else{break;}
	}
	osDelay(1000);
	for(int i=0;i< DATA_LEN ;i++){
		receivedata[i] = 0;
	}

	HAL_UART_Transmit(&huart2,(uint8_t *)ATCSOC,sizeof(ATCSOC),3000);
	HAL_UART_Receive(&huart2,(uint8_t *)receivedata,35,3000);
	if(strstr(receivedata,"OK")==NULL){
			HAL_UART_Transmit(&huart2,(uint8_t *)ATCSOCL,sizeof(ATCSOCL),2000);
			return 0;}
	
	for(int i=0;i< DATA_LEN ;i++){
		receivedata[i] = 0;
	}
	HAL_UART_Transmit(&huart2,(uint8_t *)ATCSOCON,sizeof(ATCSOCON),3000);
	HAL_UART_Receive(&huart2,(uint8_t *)receivedata,39,6000);
	if(strstr(receivedata,"OK")==NULL){
			HAL_UART_Transmit(&huart2,(uint8_t *)ATCSOCL,sizeof(ATCSOCL),2000);
			return 0;}	
	osDelay(1000);	
	for(int i=0;i< DATA_LEN ;i++){
		receivedata[i] = 0;
	}

	HAL_UART_Transmit(&huart2,(uint8_t *)sendingData,sizeof(sendingData),3500);
	HAL_UART_Receive(&huart2,(uint8_t *)receivedata,sizeof(sendingData)+6,6000);
	if(strstr(receivedata,"OK")==NULL){
			HAL_UART_Transmit(&huart2,(uint8_t *)ATCSOCL,sizeof(ATCSOCL),2000);
			return 0;}

	osDelay(1000);
  for(int i=0;i< DATA_LEN ;i++){
		receivedata[i] = 0;
	}
	HAL_UART_Transmit(&huart2,(uint8_t *)ATCSOCL,sizeof(ATCSOCL),2000);
	HAL_UART_Receive(&huart2,(uint8_t *)receivedata,sizeof(ATCSOCL)+6,500);
	if(strstr(receivedata,"OK")==NULL){
	return 0;}
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  osDelay(1100);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);			
	return 1;
	
}

// Mux

//Mux ENANLE
void mux_enable(void) {
	HAL_GPIO_WritePin(Enable_GPIO_Port,Enable_Pin,GPIO_PIN_RESET);
}

//MUX DISABLE
void mux_disable(void) {
	HAL_GPIO_WritePin(Enable_GPIO_Port,Enable_Pin,GPIO_PIN_SET);
}

//MUX SELECT
void mux_sel(uint8_t channel) {   
  switch (channel) {
		// channel 1 
		case 1:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_RESET);  
			break;
		// channel 2
		case 2:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_SET);  
			break;
		// channel 3
		case 3:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_RESET);  
			break;
		// channel 4
		case 4:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_SET);  
			break;
		// channel 5
		case 5:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_RESET);  
			break;
		// channel 6
		case 6:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_SET);  
			break;
		// channel 7
		case 7:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_RESET);  
			break;
		// channel 8
		case 8:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_SET);  
			break;
		// Default -> 1 (GPS)
		default:
			HAL_GPIO_WritePin(S2_GPIO_Port,S2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S1_GPIO_Port,S1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(S0_GPIO_Port,S0_Pin,GPIO_PIN_RESET);
  }
  
}
void oled_disp(void) {
    ssd1306_WriteCommand(0xAF); //--turn on SSD1306 panel		 
		
		ssd1306_Fill(Black);
		
		// Line 1: pm 1.0
		ssd1306_SetCursor(1, 0);
		char buff_pm1[15];
		snprintf(buff_pm1, sizeof(buff_pm1), "PM 1.0: %d", message_pointer->pm01);
		ssd1306_WriteString(buff_pm1,Font_7x10, White);
		
		// Line 2: pm 2.5
		ssd1306_SetCursor(1, 11);
		char buff_pm2_5[15];
		snprintf(buff_pm2_5, sizeof(buff_pm2_5), "PM 2.5: %d", message_pointer->pm25);
		ssd1306_WriteString(buff_pm2_5,Font_7x10, White);
		
		// Line 3: pm 10.0
		ssd1306_SetCursor(1, 22);
		char buff_pm10[15];
		snprintf(buff_pm10, sizeof(buff_pm10), "PM 10.0: %d", message_pointer->pm10);
		ssd1306_WriteString(buff_pm10,Font_7x10, White);
		
		// Line 4: Humidity
		ssd1306_SetCursor(1, 33);
		char buff_hum[15];
		snprintf(buff_hum, sizeof(buff_hum), "Hum(%%): %d", ((uint32_t)message_pointer->humid));
		ssd1306_WriteString(buff_hum,Font_7x10, White);
		
		// Line 5: Temperature
		ssd1306_SetCursor(1, 44);
		char buff_temp[15];
		snprintf(buff_temp, sizeof(buff_temp), "Temp(oC): %d",((uint32_t)message_pointer->temp));
		ssd1306_WriteString(buff_temp,Font_7x10, White);
							
		// Line 6: Pressure
		ssd1306_SetCursor(1, 54);
		char buff_pres[18];
		snprintf(buff_pres, sizeof(buff_pres), "Pressure: %d",((uint32_t)message_pointer->pressure)); //(uint32_t)message_pointer->pressure);
		ssd1306_WriteString(buff_pres,Font_7x10, White);
		
		ssd1306_UpdateScreen();			
		oled_cnt=sys_cnt;
		led_on=0;
		led_off=1;
}

bool gas_read(uint8_t gas){
	  uint16_t start = HAL_GetTick();
	  uint16_t end = start;
	  mux_sel(gas);
		char receive_data[100];
		char c[] ="c";
		char r[] = "r";
		for(int i=0;i< DATA_LEN ;i++){
		receive_data[i] = 0;
		}
		HAL_UART_Transmit(&huart1,(uint8_t *)r,1,2000);
		osDelay(500);
		HAL_UART_Transmit(&huart1,(uint8_t *)c,1,2000);
		
		
		//HAL_UART_Receive(&huart1,receive_data,) 
}