/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t HAL_GetTick(void)
{
  return uwTick;
}
uint32_t millis;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void dis(int val){
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 2&val);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1&val);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 4&val);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 8&val);
 }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 // mpu6050 function for getting the accelerometer data
void  mpu6050(uint8_t *o){

	uint8_t g[2]= {0x6b,0};  // address of mpu6050 power management to wake from sleep mode
	uint8_t l[2]= {0x1c,0};  // address for acc sensor config
	uint8_t p=0x3b;          // address for acc sensor data

	// Getting accelerometer data from the mpu6050 *refer i2c.h file for i2c function
	  init3(0x68);
      write(0x68, &l, 2);
      write(0x68, &g, 2);
      write(0x68, &p, 1);
	  read(0x68,o,6);
}

// mpu6050 function for getting gyro data
void mpu6050gyro(uint8_t *o){
	uint8_t g[2]= {0x6b,0};
	uint8_t l[2]= {0x1b,0}; // address for gyro config
	uint8_t p=0x43;   // address for gyro data

	// Getting gyro data from the mpu6050
		  init3(0x68);
		  write(0x68, &l, 2);
		  write(0x68, &g, 2);
	      write(0x68, &p, 1);
          read(0x68,o,6);
}

// Global variables for acc and gyro data
float  pitch;
float roll;
float yaw;
float de1;
float de;
float de2;
float gx;
float gy ;
float gz;

// Converting acc data to degrees
void acc(){
	int16_t  x,y,z; // 16 bit variables note : always use signed variables
	uint8_t j[6];
	mpu6050(&j);  // Getting data from mpu6050

	// Setting the sensitivity of x,y,z values
    x= (int16_t) (j[0]<<8|j[1]);
    gx = x/16384.0;

    y= j[2]<<8|j[3];
    gy = y/16384.0;

    z=j[4]<<8|j[5];
    gz = z/16384.0;

    // converting acc to radian
    pitch = atan2(gy,sqrt((gx*gx)+(gz*gz)));
    roll = atan2 (gx,gz);
    yaw = atan2(gx,gy);

    // converting radian to degree
    de = roll*180/3.14;
    de1  = (pitch-0.0010)*180/3.14;
    de2  = (yaw)*180/3.14;


}

float  gyrox,gyroy,gyroz; // gyro variables

void gyro(){
	  uint8_t s[6];
	  mpu6050gyro(&s);

	  // getting gyro values
	  int16_t gyrox1  = (int16_t) (s[0]<<8| s[1]);
		       gyrox = (gyrox1/131.0);
	  int16_t gyroy1  = (int16_t) (s[2]<<8| s[3]);
		       gyroy = gyroy1/131.0;
	  int16_t gyroz1  = (int16_t) (s[4]<<8| s[5]);
		       gyroz = gyroz1/131.0;
}

float dout [2]; // Output of Kalman filters

                     //       Kalman Filter         //

void kalman(float kalman ,float uncertanity, float angleinput,float measurement ,float elapsed){
	 kalman = kalman +((elapsed/1000)*angleinput);
	 uncertanity = uncertanity + ((elapsed/1000)*((elapsed/1000)*16));
	 float kalmangain = uncertanity/(uncertanity+(9));
	 kalman = kalman + kalmangain*(measurement - kalman);
	 uncertanity = (1-kalmangain)*uncertanity;
	 dout[0] = kalman;
	 dout[1] = uncertanity;
}

char p[1];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float gyroxangle,gyroyangle,gyrozangle;
	   HAL_Init();
	   SystemClock_Config();
	   MX_GPIO_Init();
	   MX_USART2_UART_Init();
	   MX_TIM6_Init();
//dis(0);

for(int i=0;i<2000;i++){
	gyro();
	gyroxangle = gyroxangle+gyrox;
	gyroyangle = gyroyangle+gyroy;
	gyrozangle = gyrozangle+gyroz;


}

gyroxangle = gyroxangle/2000;
gyroyangle = gyroyangle/2000;
gyrozangle = gyrozangle/2000;

char q[90];
char q1[90];
char q2[90];
char s[6];
int degree1;
int avg;
float k=0.8;
float kck2;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//int32_t n = HAL_GetTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 uint32_t kck = HAL_GetTickFreq();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_InitTick(10);


 /* USER CODE BEGIN Init */
float gx ;
float kalpitch  = 0;
float kalpitch1 = 0;
float kalpitch2 = 0;
float kalroll =0;
float kalyaw = 0;
int  uncertain = 4;
int uncertainroll = 4;
int uncertainyaw = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t cmp=0;
  float yaw;
  float yawpos;
  float yawneg;
  uint16_t adc[9];
  uint16_t adc1[9];

  // taking 2000 sample  values
  float h[9];

  int high[9];

  float degeer[9];


  for (int i=0;i<2000;i++){

  HAL_ADC_Start_DMA(&hadc1,&adc1, 9);

   high[0] = adc1[0] + high[0];
   high[1] = adc1[1] + high[1];
   high[2] = adc1[2] + high[2];
   high[3] = adc1[3] + high[3];
   high[4] = adc1[4] + high[4];
   high[5] = adc1[5] + high[5];
   high[6] = adc1[6] + high[6];
   high[7] = adc1[7] + high[7];
   high[8] = adc1[8] + high[8];

  h[0] = (adc1[0]/90)+h[0];
  h[1] = (adc1[1]/90)+h[1];
  h[2] = (adc1[2]/90)+h[2];
  h[3] = (adc1[3]/90)+h[3];
  h[4] = (adc1[4]/90)+h[4];
  h[5] = (adc1[5]/90)+h[5];
  h[6] = (adc1[6]/90)+h[6];
  h[7] = (adc1[7]/90)+h[7];
  h[8] = (adc1[8]/90)+h[8];

  }

for(int i=0;i<9;i++){
	high[i] = high[i]/2000;
	h[i] = h[i]/2000;
}

  while (1)
  {

uint32_t n=0;
n = HAL_GetTick();
float elapsed = n- cmp;

float mul[3];
float mulgyro[3];
int prev;
	         for(int i=0;i<1;i++){





if(i==0){

	          	     	     acc();
	          	  	       	 mul[0] = de;
                             mul[1] = de1;
                             mul[2] = de2;

                             gyro();
	          	   	         mulgyro[0] = gyrox-gyroxangle;
	                 	     mulgyro[1] = gyroy-gyroyangle;
	          	             mulgyro[2] = gyroz-gyrozangle;


                  kalman ( kalpitch, uncertain, mulgyro[0], mul[1],elapsed);
                  kalpitch = dout[0];
                  uncertain = dout[1];
                  kalman(kalroll, uncertainroll, mulgyro[1], mul[0], elapsed);
                  kalroll = dout[0];
                  uncertainroll = dout[1];
                  kalman(kalyaw, uncertainyaw, mulgyro[2], mul[2], elapsed);
                  kalyaw = dout[0];
                  uncertainyaw = dout[1];

                 yaw = kalyaw;
                 if (kalyaw >yaw){
                	 yawpos = kalyaw - yaw;
                 }
                 if (kalyaw<yaw){
                	 yawpos = yaw + kalyaw;
                 }

}
	         }




	         HAL_ADC_Start_DMA(&hadc1,&adc1, 9);

	         for (int i=0;i<9;i++){
	         degeer[i] = (adc1[i] - high[i])/h[i];
	         }


//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//sprintf(q,"%d\r\n",a);
for (int i=0;i<9;i++){
	  sprintf(q,"%.3f,",degeer[i]);
  	  HAL_UART_Transmit(&huart2, q, strlen(q), 10);
}
  	     //   HAL_UART_Receive(&huart2, p, 1, 10);


  	 if (p[0]=='1'){

  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
  		  	   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
  	 }
  	 if (p[0]=='0'){
  	 	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  	 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
  	 }

//p[0] = '0';

	/*
}

    //    sprintf(q,"%.3f\r\n",-kalpitch);

	      //  HAL_UART_Transmit(&huart2, q, strlen(q), 10);
	      //  sprintf(q,"%.3f\r\n",-kalpitch2);

	             HAL_UART_Transmit(&huart2, q, strlen(q), 10);
           HAL_Delay(10);
            cmp = n;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA7 PA9
                           PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
