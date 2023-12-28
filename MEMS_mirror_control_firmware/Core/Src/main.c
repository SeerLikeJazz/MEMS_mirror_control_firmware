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
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" 
#include "ADS1299_Library.h"
#include "lwrb.h"
#include "Debug.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t Direction;
uint32_t CaptureNumber;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//const uint16_t Sine12bit[100]={
//0x0800,0x0881,0x0901,0x0980,0x09FD,0x0A79,0x0AF2,0x0B68,0x0BDA,0x0C49,
//0x0CB3,0x0D19,0x0D79,0x0DD4,0x0E29,0x0E78,0x0EC0,0x0F02,0x0F3C,0x0F6F,
//0x0F9B,0x0FBF,0x0FDB,0x0FEF,0x0FFB,0x0FFF,0x0FFB,0x0FEF,0x0FDB,0x0FBF,
//0x0F9B,0x0F6F,0x0F3C,0x0F02,0x0EC0,0x0E78,0x0E29,0x0DD4,0x0D79,0x0D19,
//0x0CB3,0x0C49,0x0BDA,0x0B68,0x0AF2,0x0A79,0x09FD,0x0980,0x0901,0x0881,
//0x0800,0x077F,0x06FF,0x0680,0x0603,0x0587,0x050E,0x0498,0x0426,0x03B7,
//0x034D,0x02E7,0x0287,0x022C,0x01D7,0x0188,0x0140,0x00FE,0x00C4,0x0091,
//0x0065,0x0041,0x0025,0x0011,0x0005,0x0001,0x0005,0x0011,0x0025,0x0041,
//0x0065,0x0091,0x00C4,0x00FE,0x0140,0x0188,0x01D7,0x022C,0x0287,0x02E7,
//0x034D,0x03B7,0x0426,0x0498,0x050E,0x0587,0x0603,0x0680,0x06FF,0x077F,
//};
uint16_t DualSine12bit[100] __attribute__ ((at(0x24020000)));
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 固件版本*/
uint8_t version32[10]={"27.12.23"};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/** 单次数据发送长度 */
#define TEST_SEND_DATA_SIZE   1024
uint8_t  snd_data[TEST_SEND_DATA_SIZE];
/*--USB CDC资源部分--*/
		static size_t full_len;
		/** Ringbuf */
		lwrb_t lwrb_buff;
		uint8_t src_buff_data[100*1024] __attribute__ ((at(0x24000000)));//AXI SRAM
    /*存储每次DRDY采集的数据*/
    extern uint8_t eCon_Message[150];
    /*采样率选项，同时连接*/
    uint8_t SampleRate500[8]={0x55,0x66,0x52,0x41,0x54,0x45,0x01,0x0A};//0x52 0x41 0x54 0x45 = RATE
    uint8_t SampleRate1000[8]={0x55,0x66,0x52,0x41,0x54,0x45,0x02,0x0A};
    uint8_t SampleRate2000[8]={0x55,0x66,0x52,0x41,0x54,0x45,0x03,0x0A};
		uint8_t SampleRate4000[8]={0x55,0x66,0x52,0x41,0x54,0x45,0x04,0x0A};
		uint8_t SampleRate8000[8]={0x55,0x66,0x52,0x41,0x54,0x45,0x05,0x0A};
		uint8_t SampleRate16000[8]={0x55,0x66,0x52,0x41,0x54,0x45,0x06,0x0A};
    /*工作模式选项*/
    uint8_t Mode_impedance[8]={0x55,0x66,0x4D,0x4F,0x44,0x45,0x5A,0x0A};//0x4D 0x4F 0x44 0x45 = MODE
    uint8_t Mode_wave[8]={0x55,0x66,0x4D,0x4F,0x44,0x45,0x57,0x0A};
    uint8_t Mode_internalshort[8]={0x55,0x66,0x4D,0x4F,0x44,0x45,0x53,0x0A};
    uint8_t Mode_testsignal[8]={0x55,0x66,0x4D,0x4F,0x44,0x45,0x54,0x0A};
    uint8_t Mode_stop[8]={0x55,0x66,0x4D,0x4F,0x44,0x45,0x52,0x0A};
        /**当前工作状态*/
        uint8_t FT232_EEGstate;//"Z","W","S","T","R"
    /*断开连接*/
    uint8_t FT232_disconnect[8]={0x55,0x66,0x44,0x49,0x53,0x43,0x01,0x0A};//0x44 0x49 0x53 0x43 = DISC
        /**当前断开连接标志位*/
        uint8_t FT232_DISCflag;
    /*传输方式状态定义*/
    #define M8266Wireless  0
    #define FT232Wire      1
    /*数据传输方式*/
    uint8_t Transmission_form = M8266Wireless;
    /*采样率寄存器*/
    uint8_t SampleRateReg = SAMPLE_RATE_500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t usb_rx_data[20];
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t CDC_RX_flag;
void rx_commd()
{
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if((CDC_RX_flag == 1) && (hcdc->TxState == 0)){
		CDC_RX_flag = 0;
        /**采样率msg*/
        if((memcmp(SampleRate500,usb_rx_data,8) == 0 ) || \
            (memcmp(SampleRate1000,usb_rx_data,8) == 0 ) || \
						(memcmp(SampleRate2000,usb_rx_data,8) == 0 ) || \
						(memcmp(SampleRate4000,usb_rx_data,8) == 0 ) || \
						(memcmp(SampleRate8000,usb_rx_data,8) == 0 ) || \
            (memcmp(SampleRate16000,usb_rx_data,8) == 0 ))    {
            CDC_Transmit_FS(usb_rx_data, 8);//Echo
            switch (usb_rx_data[6]) {
                case 1:
                    SampleRateReg = SAMPLE_RATE_500;
                    break;
                case 2:
                    SampleRateReg = SAMPLE_RATE_1000;
                    break;
                case 3:
                    SampleRateReg = SAMPLE_RATE_2000;
										break;
                case 4:
                    SampleRateReg = SAMPLE_RATE_4000;
										break;
                case 5:
                    SampleRateReg = SAMPLE_RATE_8000;
                    break;
                case 6:
                    SampleRateReg = SAMPLE_RATE_16000;
                    break;
                default:;
            }
            Transmission_form = FT232Wire;
            FT232_EEGstate = 'R';//连接之后ADS1299停止运行
        }

        if(Transmission_form == FT232Wire){
            /**运行模式msg*/
            if((memcmp(Mode_wave,usb_rx_data,8) == 0 ) || \
                (memcmp(Mode_impedance,usb_rx_data,8) == 0 ) || \
                (memcmp(Mode_internalshort,usb_rx_data,8) == 0 ) || \
                (memcmp(Mode_testsignal,usb_rx_data,8) == 0 ) || \
                (memcmp(Mode_stop,usb_rx_data,8) == 0 ) )
            {
                CDC_Transmit_FS(usb_rx_data, 8);//Echo
                switch (usb_rx_data[6]) {
                    case 'W':
                        FT232_EEGstate = 'W';
                        break;
                    case 'Z':
                        FT232_EEGstate = 'Z';
                        break;
                    case 'S':
                        FT232_EEGstate = 'S';
                        break;
                    case 'T':
                        FT232_EEGstate = 'T';
                        break;
                    case 'R':
                        FT232_EEGstate = 'R';
                        break;
                    default:;
                }//end of switch
            }
            /**断开连接msg*/
            if(memcmp(FT232_disconnect,usb_rx_data,8) == 0 ){
                FT232_DISCflag = 1;
                CDC_Transmit_FS(usb_rx_data, 8);//Echo
            }
        }//end of if(Transmission_form == FT232Wire)

        memset(usb_rx_data,0,sizeof(usb_rx_data));//清空接收数组
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
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
		__HAL_GPIO_EXTI_CLEAR_IT(KEY_Pin);
		NVIC_ClearPendingIRQ(EXTI4_IRQn);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		/*lwrb初始化*/
		lwrb_init(&lwrb_buff, src_buff_data, sizeof(src_buff_data));
		/*Ads1299中断配置及初始化*/		
		__HAL_GPIO_EXTI_CLEAR_IT(ADS_DRDY_Pin);
		NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		ADS_state_choose(STOP, SAMPLE_RATE_500);//进入低功耗模式	
		
		_MSG_DBG("App start\n");
		/*开启DAC*/
		for (uint8_t Idx = 0; Idx <50; Idx++)  
		{  
			DualSine12bit[Idx] =50*sin(2*3.14159265358979f/50*Idx)+250;
		}
		HAL_TIM_Base_Start(&htim6);
		HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,(uint32_t *)DualSine12bit,50,DAC_ALIGN_12B_R);
		
HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		rx_commd();
    /*有线传输*/
    if(Transmission_form == FT232Wire){
        /*数据发送*/
				full_len = lwrb_get_full(&lwrb_buff);//buffer里面缓存了多少数据
				if ((full_len >= Frame_len)) {//打包数据
					if(lwrb_read(&lwrb_buff,snd_data,Frame_len) != Frame_len) {
						_MSG_DBG("lwrb empty\n");
					}
					else
					CDC_Transmit_FS(snd_data, Frame_len);
				}
        /*指令接收*/
				/**运行模式msg*/
				if(FT232_EEGstate){
						ADS_state_choose(FT232_EEGstate,SampleRateReg);
						FT232_EEGstate = 0;//清空
				}
				/**断开连接msg*/
				if(FT232_DISCflag){
						FT232_DISCflag = 0;//清空
						ADS_state_choose(STOP,SAMPLE_RATE_500);//进入低功耗模式										
						HAL_Delay(100);															
				}
    }	
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * 外部引脚中断
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    /*ADS1299中断*/
    if(GPIO_Pin == ADS_DRDY_Pin) {
       updateBoardData();
    }
		if(GPIO_Pin == KEY_Pin) {
			HAL_DAC_Stop(&hdac1,DAC_CHANNEL_1);
			HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
		}
		
}

/**
*systick毫秒级中断
*/
uint8_t count_ms = 0;
uint8_t old_Direction=0;
uint32_t new_CaptureNumber=8648;

//顺时针：0 ; 逆时针：1
void HAL_SYSTICK_Callback(void)
{
	count_ms++;
	if(count_ms == 10){
		count_ms = 0;

    Direction=__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2); //读取编码器旋转方向
    CaptureNumber=__HAL_TIM_GET_COUNTER(&htim2);   //读取脉冲计数值	
		if((Direction == 0) && (CaptureNumber != 0)) {
			new_CaptureNumber = new_CaptureNumber + CaptureNumber;
			__HAL_TIM_SET_AUTORELOAD(&htim6,new_CaptureNumber);
		}
		__HAL_TIM_SET_COUNTER(&htim2,0);		
	}

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
