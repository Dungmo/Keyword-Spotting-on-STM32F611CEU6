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
#include "arm_math.h"
#include <stdio.h>
#include <math.h>


#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES                    512        /* 256 real party and 256 imaginary parts */
#define FFT_SIZE                SAMPLES / 2        /* FFT size is always the same size as we have samples, so 256 in our case */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float Input[SAMPLES];
float Sample[8056];
//uint16_t adc_val;
//int j;
int count;
float hamm[] = {0.08, 0.08013854, 0.08055409, 0.08124639, 0.082215026, 0.083459415, 0.084978804, 0.086772285, 0.08883877, 0.09117702, 0.09378562, 0.09666301, 0.09980745, 0.103217036, 0.10688973, 0.11082331, 0.11501542, 0.11946351, 0.124164924, 0.12911682, 0.13431622, 0.13975999, 0.14544484, 0.15136737, 0.15752397, 0.16391098, 0.17052454, 0.17736064, 0.18441519, 0.19168393, 0.19916248, 0.20684634, 0.21473087, 0.22281136, 0.23108289, 0.23954049, 0.2481791, 0.25699347, 0.2659783, 0.27512825, 0.2844377, 0.2939011, 0.30351275, 0.31326684, 0.3231575, 0.3331788, 0.34332466, 0.353589, 0.36396563, 0.37444827, 0.38503066, 0.3957064, 0.40646905, 0.41731215, 0.42822912, 0.43921342, 0.45025846, 0.46135753, 0.472504, 0.4836911, 0.49491212, 0.5061603, 0.5174289, 0.528711, 0.54, 0.55128896, 0.5625711, 0.5738397, 0.5850879, 0.5963089, 0.607496, 0.61864245, 0.62974155, 0.6407866, 0.6517709, 0.6626879, 0.67353094, 0.6842936, 0.69496936, 0.70555174, 0.71603435, 0.726411, 0.7366753, 0.7468212, 0.7568425, 0.76673317, 0.7764873, 0.7860989, 0.7955623, 0.8048718, 0.8140217, 0.8230065, 0.8318209, 0.8404595, 0.8489171, 0.85718864, 0.8652691, 0.8731537, 0.8808375, 0.8883161, 0.8955848, 0.9026393, 0.90947545, 0.916089, 0.922476, 0.9286326, 0.9345552, 0.94024, 0.9456838, 0.95088315, 0.9558351, 0.9605365, 0.9649846, 0.9691767, 0.97311026, 0.976783, 0.98019254, 0.983337, 0.9862144, 0.988823, 0.9911612, 0.9932277, 0.9950212, 0.9965406, 0.997785, 0.9987536, 0.9994459, 0.9998615, 1.0, 0.9998615, 0.9994459, 0.9987536, 0.997785, 0.9965406, 0.9950212, 0.9932277, 0.9911612, 0.988823, 0.9862144, 0.983337, 0.98019254, 0.976783, 0.97311026, 0.9691767, 0.9649846, 0.9605365, 0.9558351, 0.95088315, 0.9456838, 0.94024, 0.9345552, 0.9286326, 0.922476, 0.916089, 0.90947545, 0.9026393, 0.8955848, 0.8883161, 0.8808375, 0.8731537, 0.8652691, 0.85718864, 0.8489171, 0.8404595, 0.8318209, 0.8230065, 0.8140217, 0.8048718, 0.7955623, 0.7860989, 0.7764873, 0.76673317, 0.7568425, 0.7468212, 0.7366753, 0.726411, 0.71603435, 0.70555174, 0.69496936, 0.6842936, 0.67353094, 0.6626879, 0.6517709, 0.6407866, 0.62974155, 0.61864245, 0.607496, 0.5963089, 0.5850879, 0.5738397, 0.5625711, 0.55128896, 0.54, 0.528711, 0.5174289, 0.5061603, 0.49491212, 0.4836911, 0.472504, 0.46135753, 0.45025846, 0.43921342, 0.42822912, 0.41731215, 0.40646905, 0.3957064, 0.38503066, 0.37444827, 0.36396563, 0.353589, 0.34332466, 0.3331788, 0.3231575, 0.31326684, 0.30351275, 0.2939011, 0.2844377, 0.27512825, 0.2659783, 0.25699347, 0.2481791, 0.23954049, 0.23108289, 0.22281136, 0.21473087, 0.20684634, 0.19916248, 0.19168393, 0.18441519, 0.17736064, 0.17052454, 0.16391098, 0.15752397, 0.15136737, 0.14544484, 0.13975999, 0.13431622, 0.12911682, 0.124164924, 0.11946351, 0.11501542, 0.11082331, 0.10688973, 0.103217036, 0.09980745, 0.09666301, 0.09378562, 0.09117702, 0.08883877, 0.086772285, 0.084978804, 0.083459415, 0.082215026, 0.08124639, 0.08055409, 0.08013854};
float Output[FFT_SIZE];
float Output_sam[10240];
float aiOutData[AI_NETWORK_OUT_1_SIZE];
arm_cfft_radix4_instance_f32 S;


ai_handle network;							// Pointer to the model

// The activations buffer. To store intermediate results
ai_buffer * ai_input;						// holds pointers to data and info about the data
ai_buffer * ai_output;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void cout()
{
	int i;
	for(i=0;i<8056;i++)
	{
		char x[10];
		sprintf(x,"%f",Sample[i]);
		HAL_UART_Transmit(&huart1,x,sizeof(x),10);
		HAL_UART_Transmit(&huart1,",",sizeof(","),10);
	}
}
void argmax(float arr[], uint8_t size) {
	int idx = 0;
	float max = arr[0];
	float tmp;

	for (int i = 0; i < size; i++) {
//		char x[7];
//		sprintf(x,"%f",arr[i]);
//		HAL_UART_Transmit(&huart1,"\n",sizeof("\n"),10);
//		HAL_UART_Transmit(&huart1,x,sizeof(x),10);
		tmp = arr[i];
		if (tmp > max) {
			max = tmp;
			idx = i;
		}
	}
//	char x[2];
//	HAL_UART_Transmit(&huart1,"\n===========\n",sizeof("\n===========\n"),10);
//	sprintf(x,"%d",idx);
//	HAL_UART_Transmit(&huart1,"KQ:  ",sizeof("KQ:  "),10);
//	HAL_UART_Transmit(&huart1,x,sizeof(x),10);
//	HAL_UART_Transmit(&huart1,"\n===========\n",sizeof("\n===========\n"),10);
	switch (idx)
	{
	case 0:
		HAL_UART_Transmit(&huart1,"Right\n",sizeof("Right\n"),10);
		break;
	case 1:
		HAL_UART_Transmit(&huart1,"Noise\n",sizeof("Noise\n"),10);
		break;
	case 2:
		HAL_UART_Transmit(&huart1,"Go\n",sizeof("Go\n"),10);
		break;
	}

	return idx;
}


static void AI_Init(void)
{
	ai_error err;
	ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
	/* Create a local array with the addresses of the activations buffers */
	const ai_handle act_addr[] = { activations };
	/* Create an instance of the model */
	err = ai_network_create_and_init(&network, act_addr, NULL);
	if (err.type != AI_ERROR_NONE) {
		HAL_UART_Transmit(&huart1,"ai_network_create error - type=%d code=%d\r\n",sizeof("ai_network_create error - type=%d code=%d\r\n"),10);
//		Error_Handler();
	}
	ai_input = ai_network_inputs_get(network, NULL);
	ai_output = ai_network_outputs_get(network, NULL);
	HAL_UART_Transmit(&huart1,"ok\n",sizeof("ok\n"),10);
}


static void AI_Run(float *pIn, float *pOut)
{
	ai_i32 batch;
	ai_error err;
//	ai_input[0].n_batches = 1;
//	ai_output[0].n_batches = 1;
	/* Update IO handlers with the data payload */
	ai_input[0].data = AI_HANDLE_PTR(pIn);

	ai_output[0].data = AI_HANDLE_PTR(pOut);
	/* Run the network */
//	HAL_UART_Transmit(&huart1,"Done2\n",sizeof("Done2\n"),10);
	batch = ai_network_run(network, ai_input, ai_output);
//	HAL_UART_Transmit(&huart1,"Done3\n",sizeof("Done3\n"),10);
	if (batch != 1) {
		err = ai_network_get_error(network);
		HAL_UART_Transmit(&huart1,"AI ai_network_run error - type=%d code=%d\r\n",sizeof("AI ai_network_run error - type=%d code=%d\r\n"),10);
//		Error_Handler();
	}
//	HAL_UART_Transmit(&huart1,"\n===========\n",sizeof("\n===========\n"),10);

	argmax(aiOutData,AI_NETWORK_OUT_1_SIZE);

}

void stft_cus()
{
	int i,j;
//
	memset(Input, 0, SAMPLES *sizeof(Input[0]));
	for(i=0;i<40;i++)
	{
//		char x[2];
//		sprintf(x,"%d",i);
//		HAL_UART_Transmit(&huart1,"\n",sizeof("\n"),10);
//		HAL_UART_Transmit(&huart1,x,sizeof(x),10);
//		HAL_UART_Transmit(&huart1,"\n",sizeof("\n"),10);
		for(j=0;j<256;j++)
		{
			Input[j*2] = Sample[i*200+j];
			Input[j*2] = Input[j*2] * hamm[j];
		}
//		for(j=0;j<512;j++)
//		{
//			char x[8];
//			sprintf(x,"%f",Input[j]);
//			HAL_UART_Transmit(&huart1,", ",sizeof(", "),10);
//			HAL_UART_Transmit(&huart1,x,sizeof(x),10);
//		}
//		HAL_UART_Transmit(&huart1,"\n",sizeof("\n"),10);
		arm_cfft_radix4_f32(&S, Input);
		arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
//		memcpy(Output_sam[i*256],Output,sizeof(Output));

		for(j=0;j<256;j++)
		{
			Output_sam[i*256+j] = Output[j];
//			char x[10];
//			sprintf(x,"%f",Output[j]);
//			HAL_UART_Transmit(&huart1,", ",sizeof(", "),10);
//			HAL_UART_Transmit(&huart1,x,sizeof(x),10);
		}
//		HAL_UART_Transmit(&huart1,"\n",sizeof("\n"),10);
		memset(Input, 0, SAMPLES *sizeof(Input[0]));
//		sprintf(x,"ok%d\n, ",i);

	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

		Sample[count] = (float)(HAL_ADC_GetValue(hadc))/4095.0;
		count++;
	//	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		if(count == 8056 && count > 8054)
		{
			count = 8058;
//			HAL_UART_Transmit(&huart1,"Done\n",sizeof("Done\n"),10);
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
//			cout();
			stft_cus();
			AI_Run(Output_sam,aiOutData);
//			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
//			HAL_UART_Transmit(&huart1,"\n",sizeof("\n"),10);
//			HAL_UART_Transmit(&huart1,"Recording..........\n",sizeof("Recording..........\n"),10);
//			check=1;
			count = 0;

		}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	float aiInData[AI_NETWORK_IN_1_SIZE];		// NN input buffer
			// NN output buffer
  /* USER CODE END 1 */
//	ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ai_error err;
  	ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
  	/* Create a local array with the addresses of the activations buffers */
  	const ai_handle act_addr[] = { activations };
  	/* Create an instance of the model */
  	err = ai_network_create_and_init(&network, act_addr, NULL);
  	if (err.type != AI_ERROR_NONE) {
  		HAL_UART_Transmit(&huart1,"ai_network_create error - type=%d code=%d\r\n",sizeof("ai_network_create error - type=%d code=%d\r\n"),10);
  //		Error_Handler();
  	}
  	ai_input = ai_network_inputs_get(network, NULL);
  	ai_output = ai_network_outputs_get(network, NULL);
	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_IT(&hadc1);
	HAL_UART_Transmit(&huart1,"YYYYYYYYYY\n",sizeof("YYYYYYYYYY\n"),10);
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  HAL_UART_Transmit(&huart1,"XXXXXXXXXXXXXXX\n",sizeof("XXXXXXXXXXXXXXX\n"),10);
//	    for(uint32_t i = 0; i < AI_NETWORK_IN_1_SIZE; i++)
//	    {
//	    	Output_sam[i] = (float)2.0f;
//	    }
//	    AI_Run(Output_sam, aiOutData);
//	    for(uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++)
//	    {
//	  	  char b[10];
//	  	  sprintf(b,"%f",aiOutData[i]);
//	  	  HAL_UART_Transmit(&huart1,b,sizeof(b),10);
//	  //		  printf(" %f", aiOutData[i]);
//	    }
//	    HAL_UART_Transmit(&huart1,"XXXXXXXXXXXXXXX\n",sizeof("XXXXXXXXXXXXXXX\n"),10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
