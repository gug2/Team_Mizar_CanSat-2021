/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include "bmi160.h"
#include "sx1276.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FATFS fatfs;
FIL file;

BMP280_HandleTypedef bmp280;
struct bmi160_dev bmi160;
lora_sx1276 lora;

RTC_TimeTypeDef RTC__Time;
RTC_DateTypeDef RTC__Date;

struct bmi160_sensor_data accel, gyro;

uint16_t crc16(uint8_t* bytes);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t SDmessageBW;
uint8_t SDmessage[256];
uint8_t SDmessageWidth;

uint8_t GPSmessage[128];
uint8_t GPSmessageWidth;

uint8_t LoRaMessage[128];
uint8_t LoRaMessageWidth;

uint8_t gpsRxBuffer[128];
uint8_t gpsRx;
uint8_t gpsRxIndex;
uint16_t validCounter, gnrmcCounter, gnvtgCounter;
char gnrmcString[64] = { '0' };

float pressure, reference_pressure, temperature, humidity;
float ax, ay, az, gx, gy, gz, relativeAltitude, absoluteAltitude;
float pitch, roll, yaw;
uint32_t
	photoResistorValue = 0,
	photoResistorThreshold = 500 // 0 - 4095 range
;

//// ======  Debug Statuses  ====== ////
FRESULT
	mount = FR_DISK_ERR,
	open = FR_DISK_ERR,
	seek = FR_DISK_ERR,
	write = FR_DISK_ERR,
	close = FR_DISK_ERR
;
bool bmp280__init_STATUS;
int8_t bmi160__init_STATUS;
uint8_t lora__init_STATUS;
uint8_t lora__send_STATUS;


//// ======  Util variables  ====== ////
uint32_t startProgramTime;
uint32_t taskCounter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t validate(char* nmeastr) {
	char check[3];
	char checkcalcstr[3];
	int i;
	int calculated_check;

	i = 0;
	calculated_check = 0;

	    // check to ensure that the string starts with a $
	if(nmeastr[i] == '$')
	    i++;
	else
	    return 0;

	//No NULL reached, 75 char largest possible NMEA message, no '*' reached
	while(nmeastr[i] != 0 && nmeastr[i] != '*' && i < 75){
	    calculated_check ^= nmeastr[i];// calculate the checksum
	    i++;
	}

	if(i >= 75){
	    return 0;// the string was too long so return an error
	}

	if(nmeastr[i] == '*'){
	    check[0] = nmeastr[i+1];    //put hex chars in check string
	    check[1] = nmeastr[i+2];
	    check[2] = 0;
	} else
	    return 0;// no checksum separator found there for invalid

	sprintf(checkcalcstr,"%02X",calculated_check);
	return (checkcalcstr[0] == check[0] && checkcalcstr[1] == check[1]) ? 1 : 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart4) {
		// TODO: any task
		if(gpsRxIndex < 128) {
			gpsRxBuffer[gpsRxIndex] = gpsRx;
			gpsRxIndex += 1;

			gpsRx = 0;
			HAL_UART_Receive_IT(&huart4, &gpsRx, 1);
		}
	}
}

uint32_t
	echoRepeaterDelayTime = 0,
	echoRepeaterStartTime = 0
;
bool
	markCountDelayTime = false,
	markProcessDelayTime = false
;

void increaseEchoDelayTime(uint32_t increaseValue) {
	// try to put pc4 anything...
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	echoRepeaterDelayTime += increaseValue;

	// log to sd for debug
	memset(SDmessage, 0, SDmessageWidth);
	SDmessageWidth = sprintf(SDmessage, "ECHO increase %d\r\n", echoRepeaterDelayTime);
	writeToSD("Data.txt", SDmessage, SDmessageWidth);
}

void startPreEchoDelayTime() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

void startPostEchoDelayTime() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	// log to sd for debug
	memset(SDmessage, 0, SDmessageWidth);
	SDmessageWidth = sprintf(SDmessage, "ECHO started\r\n");
	writeToSD("Data.txt", SDmessage, SDmessageWidth);
}

void endEchoDelayTime() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);

	echoRepeaterDelayTime = 0;

	// log to sd for debug
	memset(SDmessage, 0, SDmessageWidth);
	SDmessageWidth = sprintf(SDmessage, "ECHO complete\r\n");
	writeToSD("Data.txt", SDmessage, SDmessageWidth);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_0) { // if signal in PB0 falling...

		// log to sd for debug
		memset(SDmessage, 0, SDmessageWidth);
		SDmessageWidth = sprintf(SDmessage, "Echo interrupt, pin state: %d\r\n", HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0));
		writeToSD("Data.txt", SDmessage, SDmessageWidth);

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
			markCountDelayTime = true;
		} else {
			markProcessDelayTime = true;
			startPreEchoDelayTime();
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM1) { //check if the interrupt comes from TIM1
		// task every 250 ms
		// if gps packet has been processed - being receive new packet
		if(gpsRxIndex == 0) HAL_UART_Receive_IT(&huart4, &gpsRx, 1);
		taskCounter += 1;
	}
}


//// ------  Util methods  ------ ////
uint8_t bmi160_I2Cwrite(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, 1, (uint8_t*)data, len, 1000);
	return status;
}

uint8_t bmi160_I2Cread(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t len) {
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, 1, (uint8_t*)data, len, 1000);
	return status;
}

float altitude(float pressure, bool isRelative) {
	float ref = isRelative ? reference_pressure : 101325.0F; // Sea level pressure in [Pa]

	return 44300.0F * (1.0F - powf(pressure / ref, 1.0F / 5.255F));
}

// ВАЖНО! Запись на SD производится ТОЛЬКО ПРИ ОТКЛЮЧЕННОМ ОТ СЕТИ И ОТ STM программаторе
void writeToSD(char* filename, uint8_t* buf, uint8_t width) {
	// возможно из-за этого эхо репитер иногда долго раздупляется
	__disable_irq();
	open = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
	seek = f_lseek(&file, f_size(&file));
	write = f_write(&file, buf, width, &SDmessageBW);
	close = f_close(&file);
	__enable_irq();
}

uint16_t crc16(uint8_t* bytes) {
	uint16_t crc = 0;

	for(uint8_t i = 0; i < sizeof(bytes); i++) {
		crc ^= bytes[i];
		for(uint8_t n = 0; n < 8; n++) {
			if(crc & 0x01) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}
//// ------  ------------  ------ ////


//// ======  Barometer  ====== ////
bool bmp280__init() {
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;
	return bmp280_init(&bmp280, &bmp280.params);
}


//// ======  IMU sensor  ====== ////
int8_t bmi160__init() {
	bmi160.id = BMI160_I2C_ADDR;
	bmi160.interface = BMI160_I2C_INTF;
	bmi160.read = bmi160_I2Cread;
	bmi160.write = bmi160_I2Cwrite;
	bmi160.delay_ms = HAL_Delay;
	int8_t result = BMI160_E_DEV_NOT_FOUND;
	result = bmi160_init(&bmi160);

	bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
	bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
	bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
	result = bmi160_set_sens_conf(&bmi160);

	return result;
}


//// ======  LoRa radio  ====== ////
uint8_t* lora__init() {
	lora.spi = &hspi1;
	lora.nss_port = GPIOA;
	lora.nss_pin = GPIO_PIN_4;
	lora.frequency = 434000000; // 434 Mhz
	lora.pa_mode = LORA_PA_OUTPUT_PA_BOOST;
	lora.rx_base_addr = LORA_DEFAULT_RX_ADDR;
	lora.tx_base_addr = LORA_DEFAULT_TX_ADDR;
	lora.spi_timeout = LORA_DEFAULT_SPI_TIMEOUT;

	// hardware reset
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(250);
	//

	return lora_init(&lora, &hspi1, GPIOA, GPIO_PIN_4, 434000000);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_UART4_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // watchdog init \start
  HAL_IWDG_Init(&hiwdg);
  // watchdog init \end

  bmp280__init_STATUS = bmp280__init();
  bmi160__init_STATUS = bmi160__init();
  lora__init_STATUS = lora__init();

  // mount sd card
  mount = f_mount(&fatfs, "", 1);

  // read reference pressure
  bmp280_read_float(&bmp280, NULL, &reference_pressure, NULL);
  //

  // write header to SD card
  // Data header
  memset(SDmessage, 0, SDmessageWidth);
  SDmessageWidth = sprintf(SDmessage, "T+,Ax,y,z,Gx,y,z,Temp,AltiAbs,AltiRel,P,R,Y|CRC\r\n");
  writeToSD("Data.txt", SDmessage, SDmessageWidth);
  // Gps header
  memset(GPSmessage, 0, GPSmessageWidth);
  GPSmessageWidth = sprintf(GPSmessage, "T+,GNRMC|CRC\r\n");
  writeToSD("Gps.txt", GPSmessage, GPSmessageWidth);
  //

  // start timer 1
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  startProgramTime = HAL_GetTick();
	  //// ==== ////

	  // watchdog \refresh start
	  HAL_IWDG_Refresh(&hiwdg);
	  // watchdog \refresh end

	  if(gpsRxIndex > 0) {
		  char* nmea = (char*)gpsRxBuffer;

		  if(validate(nmea)) {
			  for(uint8_t i = 0; i < strlen(nmea) - 5; i++) {
				  if(nmea[i] == '$'
				  && nmea[i+1] == 'G'
			      && nmea[i+2] == 'N'
			      && nmea[i+3] == 'R'
			      && nmea[i+4] == 'M'
		          && nmea[i+5] == 'C') gnrmcCounter = i;

				  if(nmea[i] == '$'
			      && nmea[i+1] == 'G'
			      && nmea[i+2] == 'N'
			      && nmea[i+3] == 'V'
			      && nmea[i+4] == 'T'
			      && nmea[i+5] == 'G') gnvtgCounter = i;
			  }
		  }

		  if(gnvtgCounter - gnrmcCounter > 12) {
			  for(uint8_t i = gnrmcCounter; i < gnvtgCounter; i++) {
			  	  gnrmcString[i - gnrmcCounter] = nmea[i];
		  	  }
		  }

		  // remove \r\n in gnrmcString
		  for(uint8_t i = 0; i < strlen(gnrmcString); i++) {
			  if(gnrmcString[i] == '\r' || gnrmcString[i] == '\n') gnrmcString[i] = '\0';
		  }
		  //

	  	  memset(GPSmessage, 0, GPSmessageWidth);
	  	  sprintf(GPSmessage, "%d,%s", startProgramTime, (char*)gnrmcString);
	  	  // calculate gps crc16
	  	  uint16_t crc = crc16(GPSmessage);
	  	  //
	  	  GPSmessageWidth = sprintf(GPSmessage, "%s|%04X\r\n", (char*)GPSmessage, crc);
	  	  writeToSD("Gps.txt", GPSmessage, GPSmessageWidth);

	      validCounter += 1;

	      memset(gpsRxBuffer, 0, 128);
	      gpsRxIndex = 0;
	      gpsRx = 0;
	      memset(gnrmcString, 0, 64);
	  }

	  ////HAL_RTC_GetTime(&hrtc, &RTC__Time, RTC_FORMAT_BIN);
	  ////HAL_RTC_GetDate(&hrtc, &RTC__Date, RTC_FORMAT_BIN);

	  bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
	  relativeAltitude = altitude(pressure, true);
	  absoluteAltitude = altitude(pressure, false);

	  bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &accel, &gyro, &bmi160);
	  ax = accel.x / 32768.0F * 2.0F * 9.81F;
	  ay = accel.y / 32768.0F * 2.0F * 9.81F;
	  az = accel.z / 32768.0F * 2.0F * 9.81F;
	  gx = gyro.x / 32768.0F * 250.0F;
	  gy = gyro.y / 32768.0F * 250.0F;
	  gz = gyro.z / 32768.0F * 250.0F;

	  pitch = atan2(ay, sqrt(ax*ax + az*az)) * (180.0F / 3.14F);
	  roll = atan2(-ax, sqrt(ay*ay + az*az)) * (180.0F / 3.14F);
	  yaw = gz;

	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2, 250);
	  photoResistorValue = HAL_ADC_GetValue(&hadc2);
	  HAL_ADC_Stop(&hadc2);
	  if(photoResistorValue >= photoResistorThreshold) {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	  }

	  // sd
	  memset(SDmessage, 0, SDmessageWidth);
	  sprintf(SDmessage,
	      "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%d",
		  startProgramTime, ax, ay, az, gx, gy, gz, temperature, absoluteAltitude, relativeAltitude, pitch, roll, yaw, photoResistorValue
	  );
	  // calculate crc16
	  uint16_t crc = crc16(SDmessage);
	  //
	  SDmessageWidth = sprintf(SDmessage, "%s|%04X\r\n", (char*)SDmessage, crc);
	  writeToSD("Data.txt", SDmessage, SDmessageWidth);

	  /*** ** LoRa ** ***/
	  /*memset(LoRaMessage, 0, LoRaMessageWidth);
	  LoRaMessageWidth = sprintf(LoRaMessage, "Hello World! %d <%s>\r\n", startProgramTime, (char*)gnrmcString);
	  lora__send_STATUS = lora_send_packet_blocking(&lora, LoRaMessage, LoRaMessageWidth, 250);
	  memset(gnrmcString, 0, 64);*/
	  /*** ** ==== ** ***/

	  // start Echo repeater
	  if(markCountDelayTime == true && markProcessDelayTime == true) {
		  if(echoRepeaterStartTime == 0) {
			  startPostEchoDelayTime();

			  echoRepeaterStartTime = HAL_GetTick();

			  markCountDelayTime = false;
		  }
	  }
	  if(markProcessDelayTime == true) {
		  if(HAL_GetTick() - echoRepeaterStartTime >= echoRepeaterDelayTime * 50) {
			  endEchoDelayTime();

			  echoRepeaterStartTime = 0;

			  markProcessDelayTime = false;
		  }
	  }
	  if(markCountDelayTime == true) {
		  increaseEchoDelayTime( HAL_GetTick() - startProgramTime );
		  // repeatable event while mark is true
	  }
	  // end Echo repeater

	  //// ==== ////
	  uint32_t loopTime = HAL_GetTick() - startProgramTime;
	  if(loopTime < 250) HAL_Delay(250 - 1 - loopTime);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
