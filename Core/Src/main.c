/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t txData[8];
uint8_t rxData[8];
uint8_t trigger1msFlag=0;
uint8_t spiAsk[]={0x05,0x00,0x00,0x00,0x00};
uint8_t spi1Rec[5];
uint8_t spi2Rec[5];
uint8_t spiTxData[8],spiTxDataOld[8],spiTxDataBuff[16];
uint8_t spi1HoldData[3];
uint8_t spi2HoldData[3];
int32_t angle;
uint32_t send_angle;
uint8_t dataInterity;
uint8_t dmaBuff[15]; // for receiving pos feedback
uint8_t FBposBuff[30];
uint8_t torEnFlag,torEnFlagOld;
uint8_t desPos[4];
HAL_StatusTypeDef spi1state;
HAL_StatusTypeDef spi2state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint8_t SAEJ1850_CRC8(uint8_t* data,  uint8_t length);
void int32toBytes(int32_t data, uint8_t* outBytes);
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void unpack_reply(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *data)
{
	//if (pRxHeader->DataLength == FDCAN_DLC_BYTES_8)
	{
		int i;
		uint32_t recID=pRxHeader->Identifier;
		if (recID==0x145)
		{
			for (i=0;i<4;i++)
				txData[i]=spiTxDataBuff[i];

			if (rxData[0]==1)
				torEnFlag=1;
			else
				torEnFlag=0;

			uint8_t returnMsg[13];
			returnMsg[0]=0xff;
			returnMsg[1]=0xff;
			returnMsg[2]=0xfd;
			returnMsg[3]=0; // reserved
			returnMsg[4]=1; // device ID
			returnMsg[5]=6; // low byte of length
			returnMsg[6]=0; // high byte of length
			returnMsg[7]=3; // instruction code
			returnMsg[8]=68; // Low byte reg address
			returnMsg[9]=0x00; // high byte reg address
			returnMsg[10]=1; // para bytes
			returnMsg[11]=0xde; //low byte of CRC
			returnMsg[12]=0; // high byte of CRC
			unsigned short crc16;
			crc16=update_crc(0, returnMsg, 11);
			returnMsg[11]= crc16 & 0x00ff;
			returnMsg[12] = (crc16>>8) & 0x00ff;

			if (rxData[0]==2)
			{
				HAL_UART_Abort(&huart1);
				HAL_UART_DMAStop(&huart1);

				//MX_DMA_Init();
				HAL_Delay(100);
				HAL_UART_Transmit(&huart1,returnMsg,13,2); // disable response data for write cmd
				HAL_Delay(100);
				for (i=0;i<15;i++)
					dmaBuff[i]=0;
				for (i=0;i<30;i++)
					FBposBuff[i]=0;
				HAL_UART_Receive_DMA(&huart1, dmaBuff, 15);
			}

			desPos[0]=rxData[2]; // little-endian
			desPos[1]=rxData[3];
			desPos[2]=rxData[4];
			desPos[3]=rxData[5];

			for (i=0;i<16;i++)
			{if (FBposBuff[i]==0xff && FBposBuff[i+1]==0xff && FBposBuff[i+2]==0xfd)
					break;
			}
			txData[4]=FBposBuff[i+9];
			txData[5]=FBposBuff[i+10];
			txData[6]=FBposBuff[i+11];
			txData[7]=FBposBuff[i+12];
		}
		TxHeader.Identifier=recID;
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData);
	}

}
/*void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, rxData);
	int i;
	if (RxHeader.Identifier==0x145)
		{
		int i;
		for (i=0;i<8;i++)
			txData[i]=spiTxDataBuff[i];
		}
	else if (RxHeader.Identifier==0x146)
	{
		if (rxData[0]==1)
			torEnFlag=1;
		else
			torEnFlag=0;

		for (i=0;i<8;i++)
			txData[i]=0;

		uint8_t returnMsg[13];
		returnMsg[0]=0xff;
		returnMsg[1]=0xff;
		returnMsg[2]=0xfd;
		returnMsg[3]=0; // reserved
		returnMsg[4]=1; // device ID
		returnMsg[5]=6; // low byte of length
		returnMsg[6]=0; // high byte of length
		returnMsg[7]=3; // instruction code
		returnMsg[8]=68; // Low byte reg address
		returnMsg[9]=0x00; // high byte reg address
		returnMsg[10]=1; // para bytes
		returnMsg[11]=0xde; //low byte of CRC
		returnMsg[12]=0; // high byte of CRC
		unsigned short crc16;
		crc16=update_crc(0, returnMsg, 11);
		returnMsg[11]= crc16 & 0x00ff;
		returnMsg[12] = (crc16>>8) & 0x00ff;

		if (rxData[0]==2)
		{
			HAL_UART_DMAStop(&huart1);
			HAL_UART_Transmit(&huart1,returnMsg,13,2); // disable response data for write cmd
			MX_DMA_Init();
			for (i=0;i<15;i++)
				dmaBuff[i]=0;
			for (i=0;i<30;i++)
				FBposBuff[i]=0;
			HAL_UART_Receive_DMA(&huart1, dmaBuff, 15);
		}

		desPos[0]=rxData[2]; // little-endian
		desPos[1]=rxData[3];
		desPos[2]=rxData[4];
		desPos[3]=rxData[5];

		for (i=0;i<16;i++)
			{if (FBposBuff[i]==0xff && FBposBuff[i+1]==0xff && FBposBuff[i+2]==0xfd)
				break;
			}
			txData[0]=FBposBuff[i+9];
			txData[1]=FBposBuff[i+10];
			txData[2]=FBposBuff[i+11];
			txData[3]=FBposBuff[i+12];
	}
	TxHeader.Identifier=RxHeader.Identifier;
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txData);
	//HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	}
}*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim==&htim2)
		trigger1msFlag=1;
}
void delay_us(uint16_t nus)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < nus) ;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
	int i;
/*	for (i=15;i<30;i++)
		FBposBuff[i]=FBposBuff[i-15];
	for (i=0;i<15;i++)
		FBposBuff[i]=dmaBuff[i]; */
	for (i=0;i<15;i++)
			FBposBuff[i]=FBposBuff[i+15];
	for (i=15;i<30;i++)
		FBposBuff[i]=dmaBuff[i-15];
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
  TxHeader.Identifier=0x144;
  TxHeader.IdType=FDCAN_STANDARD_ID;
  TxHeader.TxFrameType=FDCAN_DATA_FRAME;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.DataLength=FDCAN_DLC_BYTES_8;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat=FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker=0;

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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  trigger1msFlag=0;

  HAL_Delay(2000);
  torEnFlag=0;
  torEnFlagOld=0;

  uint8_t torEn[13];
  torEn[0]=0xff;
  torEn[1]=0xff;
  torEn[2]=0xfd;
  torEn[3]=0; // reserved
  torEn[4]=1; // device ID
  torEn[5]=6; // low byte of length
  torEn[6]=0; // high byte of length
  torEn[7]=3; // instruction code
  torEn[8]=0x40; // Low byte reg address
  torEn[9]=0x00; // high byte reg address
  torEn[10]=1; // para bytes
  torEn[11]=0xde; //low byte of CRC
  torEn[12]=0; // high byte of CRC
  unsigned short crc16;
  crc16=update_crc(0, torEn, 11);
  torEn[11]= crc16 & 0x00ff;
  torEn[12] = (crc16>>8) & 0x00ff;
  //HAL_UART_Transmit(&huart1,torEn,13,2); // enable torque output

  uint8_t torDis[13];
  torDis[0]=0xff;
  torDis[1]=0xff;
  torDis[2]=0xfd;
  torDis[3]=0; // reserved
  torDis[4]=1; // device ID
  torDis[5]=6; // low byte of length
  torDis[6]=0; // high byte of length
  torDis[7]=3; // instruction code
  torDis[8]=0x40; // Low byte reg address
  torDis[9]=0x00; // high byte reg address
  torDis[10]=0; // para bytes
  torDis[11]=0xde; //low byte of CRC
  torDis[12]=0; // high byte of CRC
  crc16=update_crc(0, torDis, 11);
  torDis[11]= crc16 & 0x00ff;
  torDis[12] = (crc16>>8) & 0x00ff;

  uint8_t returnMsg[13];
  returnMsg[0]=0xff;
  returnMsg[1]=0xff;
  returnMsg[2]=0xfd;
  returnMsg[3]=0; // reserved
  returnMsg[4]=1; // device ID
  returnMsg[5]=6; // low byte of length
  returnMsg[6]=0; // high byte of length
  returnMsg[7]=3; // instruction code
  returnMsg[8]=68; // Low byte reg address
  returnMsg[9]=0x00; // high byte reg address
  returnMsg[10]=1; // para bytes
  returnMsg[11]=0xde; //low byte of CRC
  returnMsg[12]=0; // high byte of CRC
  crc16=update_crc(0, returnMsg, 11);
  returnMsg[11]= crc16 & 0x00ff;
  returnMsg[12] = (crc16>>8) & 0x00ff;
  HAL_UART_Transmit(&huart1,returnMsg,13,2); // disable response data for write cmd

  HAL_UART_Receive_DMA(&huart1, dmaBuff, 15);

  uint8_t getCurPos[14];
  getCurPos[0]=0xff;
  getCurPos[1]=0xff;
  getCurPos[2]=0xfd;
  getCurPos[3]=0;
  getCurPos[4]=1;
  getCurPos[5]=7;
  getCurPos[6]=0;
  getCurPos[7]=0x02;
  getCurPos[8]=0x84;
  getCurPos[9]=0x00;
  getCurPos[10]=0x04;
  getCurPos[11]=0x00;
  crc16=update_crc(0, getCurPos, 12);
  getCurPos[12]=crc16 & 0x00ff;
  getCurPos[13]=(crc16>>8) & 0x00ff;
  HAL_UART_Transmit(&huart1,getCurPos,14,2); // get the current position

  uint8_t setDesPos[16];
  setDesPos[0]=0xff;
  setDesPos[1]=0xff;
  setDesPos[2]=0xfd;
  setDesPos[3]=0;
  setDesPos[4]=1;
  setDesPos[5]=9;
  setDesPos[6]=0;
  setDesPos[7]=0x03;
  setDesPos[8]=0x74;
  setDesPos[9]=0x00;
  setDesPos[10]=0x04; // des pos
  setDesPos[11]=0x00; // des pos
  setDesPos[12]=0x1d; // des pos
  setDesPos[13]=0x15; // des pos
  setDesPos[14]=0;
  setDesPos[15]=0;

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim4);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int gpioCount=0;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (trigger1msFlag==1)
	  {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		spi1state=HAL_SPI_TransmitReceive(&hspi1, spiAsk, spi1Rec, 5,1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		spi2state=HAL_SPI_TransmitReceive(&hspi1, spiAsk, spi2Rec, 5,1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		int i;
		for (i=0;i<8;i++)
			spiTxData[i]=0;

		uint8_t dataTmp[4];
		uint8_t crc8;
		dataTmp[0]=spi1Rec[1];
		dataTmp[1]=spi1Rec[2];
		dataTmp[2]=spi1Rec[3];
		dataTmp[3]=spi1Rec[4];
		crc8=SAEJ1850_CRC8(dataTmp,3);
		if (crc8==dataTmp[3] && spi1state==HAL_OK && (dataTmp[0]+dataTmp[1])!=0)
		{
			spiTxData[1]=dataTmp[0];
			spiTxData[0]=dataTmp[1];
		}
		else
		{
			spiTxData[1]=spiTxDataOld[1];
			spiTxData[0]=spiTxDataOld[0];
			spiTxData[4]=0x0e;
		}

		dataTmp[0]=spi2Rec[1];
		dataTmp[1]=spi2Rec[2];
		dataTmp[2]=spi2Rec[3];
		dataTmp[3]=spi2Rec[4];
		crc8=SAEJ1850_CRC8(dataTmp,3);
		if (crc8==dataTmp[3] && spi2state==HAL_OK && (dataTmp[0]+dataTmp[1])!=0)
		{
			spiTxData[3]=dataTmp[0];
			spiTxData[2]=dataTmp[1];
		}
		else
		{
			spiTxData[3]=spiTxDataOld[3];
			spiTxData[2]=spiTxDataOld[2];
			spiTxData[5]=0x0e;
		}

		for (i=0;i<8;i++)
			spiTxDataOld[i]=spiTxData[i];

		for (i=0;i<8;i++)
			spiTxDataBuff[i]=spiTxDataBuff[i+8];

		for (i=0;i<8;i++)
			spiTxDataBuff[i+8]=spiTxData[i];



		if (torEnFlag!=torEnFlagOld)
		{
			if (torEnFlag==1)
				HAL_UART_Transmit(&huart1,torEn,13,2);
			else
				HAL_UART_Transmit(&huart1,torDis,13,2);
			torEnFlagOld=torEnFlag;
			//HAL_UART_Receive_DMA(&huart1, dmaBuff, 15);
		}

		if (torEnFlag==1)
		{
			setDesPos[10]=desPos[0]; // des pos
			setDesPos[11]=desPos[1]; // des pos
			setDesPos[12]=desPos[2]; // des pos
			setDesPos[13]=desPos[3]; // des pos
			crc16=update_crc(0, setDesPos, 14);
			setDesPos[14]= crc16 & 0x00ff;
			setDesPos[15] = (crc16>>8) & 0x00ff;
			HAL_UART_Transmit(&huart1,setDesPos,16,2);
		}

		HAL_UART_Transmit(&huart1,getCurPos,14,2);
		trigger1msFlag=0;
	  }
	  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) >= 1)
	  {
		  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, rxData) == HAL_OK)
	  	  {
			  unpack_reply(&RxHeader, rxData);
	  	  }
	  }
	 /* if(gpioCount==2)
	  	  {gpioCount=0;HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);}
	  else
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  gpioCount++;*/
	  delay_us(10);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 129;
  hfdcan1.Init.NominalTimeSeg2 = 20;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x145;
  sFilterConfig.FilterID2 = 0x146;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_Start(&hfdcan1);

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 14;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 149;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t SAEJ1850_CRC8(uint8_t* data,  uint8_t length)
{
	uint8_t i,j;
	uint8_t crc,ploy;

	crc = 0xff;
	ploy = 0x1d;

	for(i = 0; i< length; i++)
	{
		crc^= data[i];
		for(j =0; j< 8; j++)
		{
			if(crc& 0x80)
				crc = (crc<<1)^ploy;
			else
				crc <<=1;
		}
	}
	crc^=0xff;
	return crc;
}

void int32toBytes(int32_t data, uint8_t* outBytes)
{
	outBytes[0]=(uint8_t)(data & 0xff);
	outBytes[1]=(uint8_t)((data & 0xff00)>>8);
	outBytes[2]=(uint8_t)((data & 0xff0000)>>16);
	outBytes[3]=(uint8_t)((data>>24) & 0xff);
}

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
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
