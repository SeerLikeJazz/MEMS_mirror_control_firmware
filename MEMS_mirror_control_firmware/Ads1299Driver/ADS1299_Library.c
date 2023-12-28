#include "ADS1299_Library.h"
#include "spi.h"
#include "lwrb.h"
//#include "usart.h"
#include "Debug.h"
#include "arm_math.h"


#define Dummy_Byte                      0xFF
#define csLow()                      HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define csHigh()                     HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

/**
  * @brief  SPI传输读取一字节
  * @param  byte 要传输的字节
  * @retval 读取的字节
  */

uint8_t    ADS_xfer(uint8_t byte)
{
    uint8_t d_read,d_send=byte;

    if(HAL_SPI_TransmitReceive(&hspi2,&d_send,&d_read,1,0xFFFFFF)!=HAL_OK)
        d_read=Dummy_Byte;

    return d_read;
}

uint8_t    regData[24]; // array is used to mirror register data
/**
  * @brief  写寄存器
  * @param  _address  寄存器地址
	* @param  _value    要写入的值
	* @param  target_SS 选择对应的ADS1299
  * @retval None
  */
void ADS_WREG(uint8_t _address, uint8_t _value)
{                                 //  Write ONE register at _address
    uint8_t opcode1 = _address + 0x40; //  WREG expects 010rrrrr where rrrrr = _address
    csLow();               //  open SPI
    ADS_xfer(opcode1);                  //  Send WREG command & address
    ADS_xfer(0x00);                     //  Send number of registers to read -1
    ADS_xfer(_value);                   //  Write the value to the register
    csHigh();              //  close SPI
    regData[_address] = _value;     //  update the mirror array

}
/**
  * @brief  读寄存器
  * @param  _address  寄存器地址
	* @param  target_SS 选择对应的ADS1299
  * @retval 读取的值
  */
uint8_t ADS_RREG(uint8_t _address)
{                                 //  reads ONE register at _address
    uint8_t opcode1 = _address + 0x20; //  RREG expects 001rrrrr where rrrrr = _address
    csLow();                //  open SPI
    ADS_xfer(opcode1);                  //  opcode1
    ADS_xfer(0x00);                     //  opcode2
    regData[_address] = ADS_xfer(0x00); //  update mirror location with returned byte
    csHigh();               //  close SPI

    return regData[_address]; // return requested register value
}
/**
  * @brief  低功耗模式
	* @param  target_SS 选择对应的ADS1299
  * @retval None
  */
void ADS_STANDBY()
{
    csLow();
    ADS_xfer(_STANDBY);
    HAL_Delay(1); //must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
    csHigh();
}

/**
  * @brief  从Standby模式唤醒
	* @param  target_SS 选择对应的ADS1299
  * @retval None
  */
void ADS_WAKEUP()
{ // reset all the registers to default settings
    csLow();
    ADS_xfer(_WAKEUP);
    csHigh();
    HAL_Delay(1); //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}
/**
  * @brief  复位
	* @param  target_SS 选择对应的ADS1299
  * @retval None
  */
void ADS_RESET()
{ // reset all the registers to default settings
    csLow();
    ADS_xfer(_RESET);
    HAL_Delay(1); //must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
    csHigh();
}
/**
  * @brief  停止数据连续读取模式
	* @param  target_SS 选择对应的ADS1299
  * @retval None
  */
void ADS_SDATAC()
{
    csLow();
    ADS_xfer(_SDATAC);
    csHigh();
    HAL_Delay(1); //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
}
/**
  * @brief  读取ADS1299的ID
	* @param  target_SS 选择对应的ADS1299
  * @retval ID值
  */
uint8_t ADS_getDeviceID()
{ // simple hello world com check
    uint8_t data = ADS_RREG(ID_REG);
    return data;
}
/**
  * @brief  ADS1299开始转换
	* @param  target_SS 选择对应的ADS1299
  * @retval None
  */
void  ADS_START()
{ //start data conversion
    csLow();
    ADS_xfer(_START); // KEEP ON-BOARD AND ON-DAISY IN SYNC
    csHigh();
}
/**
  * @brief  数据连续读取模式
	* @param  target_SS 选择对应的ADS1299
  * @retval None
  */
void  ADS_RDATAC()
{
    csLow();
    ADS_xfer(_RDATAC); // read data continuous
    csHigh();
    HAL_Delay(1);
}
/**
  * @brief  初始化Ads1299
	* @param  None
  * @retval None
  */

void initialize_ads(SAMPLE_RATE sr)
{
    HAL_Delay(200);// recommended power up sequence requiers >Tpor (2的18次Tclk) pdf.70

    ADS_RESET();
    HAL_Delay(10);// recommended to wait 18 Tclk before using device

    ADS_SDATAC();
    HAL_Delay(10);

    uint8_t output_data_rate;
    switch(sr) {
        case SAMPLE_RATE_250:
            output_data_rate= 0x06;
            break;
        case SAMPLE_RATE_500:
            output_data_rate= 0x05;
            break;
        case SAMPLE_RATE_1000:
            output_data_rate= 0x04;
            break;
        case SAMPLE_RATE_2000:
						output_data_rate= 0x03;
            break;
        case SAMPLE_RATE_4000:
						output_data_rate= 0x02;
            break;
        case SAMPLE_RATE_8000:
						output_data_rate= 0x01;
            break;
        case SAMPLE_RATE_16000:
						output_data_rate= 0x00;
            break;
        default:;
    }
    ADS_WREG(CONFIG1,0x90|output_data_rate);//tell on-board ADS to output its clk, set the data rate to xxxSPS
    HAL_Delay(200);
    ADS_WREG(CONFIG3,0xEC);//BIASREF signal \ BIAS buffer power
    HAL_Delay(10);

    if(ADS_getDeviceID() != ADS_ID) {
        Error_Handler();
    };
    HAL_Delay(10);
}


/**
  * @brief  设置Ads1299模式（1-Impedance、2-Normal、3-Short noise、4-Test wave）
	* @param  None
  * @retval None
  */
void ADS_ModeSelect(uint8_t mode)
{
    switch(mode) {
        case Impedance:
        {
            ADS_WREG(LOFF,0x02);
            HAL_Delay(10);

            ADS_WREG(BIAS_SENSP,0xFF);
            HAL_Delay(10);
            ADS_WREG(BIAS_SENSN,0xFF);
            HAL_Delay(10);
            ADS_WREG(LOFF_SENSP,0xFF);
            HAL_Delay(10);
            ADS_WREG(LOFF_SENSN,0x00);
            HAL_Delay(10);
            ADS_WREG(LOFF_FLIP,0x00);
            HAL_Delay(10);
            ADS_WREG(MISC1,0x20);//设置SRB1
            HAL_Delay(10);

            ADS_WREG(CH1SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH2SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH3SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH4SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH5SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH6SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH7SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH8SET,0x60);
            HAL_Delay(10);

            ADS_START();
            HAL_Delay(40);
            ADS_RDATAC();
            HAL_Delay(13);
        }
            break;
        case Normal:
        {
            ADS_WREG(BIAS_SENSP,0x00);
            HAL_Delay(10);
            ADS_WREG(BIAS_SENSN,0x00);
            HAL_Delay(10);
            ADS_WREG(MISC1,0x10);//设置SRB1
            HAL_Delay(10);

            ADS_WREG(CH1SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH2SET,0x60);
            HAL_Delay(10);
            ADS_WREG(CH3SET,0x81);
            HAL_Delay(10);
            ADS_WREG(CH4SET,0x81);
            HAL_Delay(10);
            ADS_WREG(CH5SET,0x81);
            HAL_Delay(10);
            ADS_WREG(CH6SET,0x81);
            HAL_Delay(10);
            ADS_WREG(CH7SET,0x81);
            HAL_Delay(10);
            ADS_WREG(CH8SET,0x81);
            HAL_Delay(10);

            ADS_START();
            HAL_Delay(40);
            ADS_RDATAC();
            HAL_Delay(13);
            /*使用DMA前，CS拉低*/
            //	csLow(BOTH_ADS);
        }
            break;
        case InternalShort:
        {
            ADS_WREG(CONFIG2,0xC0);
            HAL_Delay(10);
            ADS_WREG(CH1SET,0x01);
            HAL_Delay(10);
            ADS_WREG(CH2SET,0x01);
            HAL_Delay(10);
            ADS_WREG(CH3SET,0x01);
            HAL_Delay(10);
            ADS_WREG(CH4SET,0x01);
            HAL_Delay(10);
            ADS_WREG(CH5SET,0x01);
            HAL_Delay(10);
            ADS_WREG(CH6SET,0x01);
            HAL_Delay(10);
            ADS_WREG(CH7SET,0x01);
            HAL_Delay(10);
            ADS_WREG(CH8SET,0x01);
            HAL_Delay(10);

            ADS_START();
            HAL_Delay(40);
            ADS_RDATAC();
            HAL_Delay(13);
            //	csLow(BOTH_ADS);
        }
            break;
        case TestSignal:
        {
            ADS_WREG(CONFIG2,0xD0);
            HAL_Delay(10);
            ADS_WREG(CH1SET,0x05);
            HAL_Delay(10);
            ADS_WREG(CH2SET,0x05);
            HAL_Delay(10);
            ADS_WREG(CH3SET,0x05);
            HAL_Delay(10);
            ADS_WREG(CH4SET,0x05);
            HAL_Delay(10);
            ADS_WREG(CH5SET,0x05);
            HAL_Delay(10);
            ADS_WREG(CH6SET,0x05);
            HAL_Delay(10);
            ADS_WREG(CH7SET,0x05);
            HAL_Delay(10);
            ADS_WREG(CH8SET,0x05);
            HAL_Delay(10);

            ADS_START();
            HAL_Delay(40);
            ADS_RDATAC();
            HAL_Delay(13);
        }
            break;
        default:;
    }
}



extern lwrb_t lwrb_buff;
extern uint8_t src_buff_data[500*1024];// __attribute__ ((at(0x240000F0)));//AXI SRAM
/**
 * ADS运行模式选择
 * */
uint8_t Is_standby = 1;

void ADS_state_choose(uint8_t EEG_State,SAMPLE_RATE sr)
{
    if(Is_standby){
        Is_standby = 0;
        ADS_WAKEUP();
    }

    switch(EEG_State) {
        case IMPEDANCING:
            initialize_ads(sr);
            ADS_ModeSelect(Impedance);
						HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            break;

        case WAVE:
            initialize_ads(sr);
            ADS_ModeSelect(Normal);
						HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            break;

        case INTERNALSHORT:
            initialize_ads(sr);
            ADS_ModeSelect(InternalShort);
						HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            break;

        case TESTSIGAL:
            initialize_ads(sr);
            ADS_ModeSelect(TestSignal);
						HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
            break;

        case STOP:
            ADS_STANDBY();//进入低功耗模式
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
            lwrb_reset(&lwrb_buff);//清空缓存数据
//            lwrb_init(&lwrb_buff, src_buff_data, sizeof(src_buff_data));//清空缓存数据
            Is_standby = 1;
            break;

        default:;
    }
}



int  boardStat;
/*数据临时存储*/
uint8_t demoIDX=0;
uint8_t eCon_Message[150] = {0xBB,0xAA};

uint8_t eCon_Checksum(uint8_t *content,uint8_t len);
/**
  * @brief  更新两片ADS1299的数据
  * @retval None
  */
#define LENGTH_PhaseArray   50
int boardChannelDataInt[2];
float32_t TPhaseArray[2][LENGTH_PhaseArray];
uint8_t Index_PhaseArray = 0;
float32_t angle_rad;

void updateBoardData(void)
{
		int ChannelDataInt;
    uint8_t inByte;
    int byteCounter = 2;//前面2字节为包头
    csLow();

		for (int i = 0; i < 3; i++) {
				inByte = ADS_xfer(0x00); //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
				boardStat = (boardStat << 8) | inByte;
		}
		for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 3; j++)
				{ //  read 24 bits of channel data in 8 3 byte chunks
						inByte = ADS_xfer(0x00);
						/*放入数据包*/
						eCon_Message[byteCounter++] = inByte;
						/*组成4字节*/
						boardChannelDataInt[i] = (boardChannelDataInt[i] << 8) | inByte; // int data goes here
				}
		}

    csHigh(); // close SPI
    // need to convert 24bit to 32bit if using the filter
    for (int i = 0; i < 2; i++)
    { // convert 3 byte 2's compliment to 4 byte 2's compliment
        if ((boardChannelDataInt[i] & 0x00800000) == 0x00800000)
        {
            boardChannelDataInt[i] |= 0xFF000000;
        }
        else
        {
            boardChannelDataInt[i] &= 0x00FFFFFF;
        }
				TPhaseArray[i][Index_PhaseArray] = (float32_t)boardChannelDataInt[i];

    }
		Index_PhaseArray++;
		if(Index_PhaseArray == LENGTH_PhaseArray) {
			Index_PhaseArray = 0;
//			TestPhaseCal(); 
			
		}		
    /**eConScan格式*/
    eCon_Message[8] = demoIDX++;
    if(lwrb_write(&lwrb_buff, eCon_Message, 9) != 9)
		{
			_MSG_DBG("lwrb full\n");
		}
}
/*数据包格式：BBAA + 8ch*3bytes*4chips + 校验位 + 标签位 + 电池电量 + 包序号 = 102*/

/*校验位函数*/
uint8_t eCon_Checksum(uint8_t *content,uint8_t len)
{
    uint8_t result = 0;
    for(int i=0;i<len;i++) {
        result += content[i];
    }
    return ~result;
}




//float32_t TPhaseArray[2][40] = 
//{
//{0,	0.360866623520662,	0.709887598281048,	1.03560611556705,	1.32733028286492,
//1.57548409121326,	1.77192175305340,	1.91019509226214,	1.98576520913963,	1.99615147233081,
//1.94101294693709,	1.82215958589509,	1.64349281725918,	1.41087747764335,	1.13194929567902,
//0.815864244951819,	0.472997994047449,	0.114605319426921,	-0.247549338905888,	-0.601578051727556,
//-0.935859628521147,	-1.23942108955499,	-1.50229786007989,	-1.71586086306872,	-1.87309977349638,
//-1.96885313617978,	-1.99997779343119,	-1.96545206096741,	-1.86640926526780,	-1.70610054149872,
//-1.48978811318324,	-1.22457255677065,	-0.919159721242975,	-0.583574953752702,	-0.228834011995030,
//0.133418534199903,	0.491291542384855,	0.833037630897356,	1.14743879323345,	1.42417463538573},

//{12.5000000000000,	12.6480534418411,	12.7748341669787,	12.8761805271978,	12.9487657715384,
//12.9902072487559,	12.9991446191813,	12.9752845086313,	12.9194101385806,	12.8333556164780,
//12.7199457301445,	12.5829032225324,	12.4267265906059,	12.2565424196622,	12.0779371003014,
//11.8967734520307,	11.7189982729348,	11.5504471327022,	11.3966528167866,	11.2626637096297,
//11.1528780786179,	11.0708996984953,	11.0194195554426,	11.0001275139496,	11.0136568460690,
//11.0595634439066,	11.1363403977116,	11.2414674610317,	11.3714937792127,	11.5221511656329,
//11.6884942073159,	11.8650626008781,	12.0460603900409,	12.2255462211376,	12.3976283713710,
//12.5566581479089,	12.6974153093867,	12.8152794232491,	12.9063815340339,	12.9677311639984}
//};
float32_t normPhaseArray1[LENGTH_PhaseArray],normPhaseArray2[LENGTH_PhaseArray];
float32_t degreeValue = 0.0f;
void TestPhaseCal() //执行时间45 uS
{
		// 对信号进行归一化处理，按向量返回 A 中数据的 z 值（中心为 0、标准差为 1）
    // 使用 DSP 库函数计算均值
		float32_t meanValue = 0.0f, stdValue= 0.0f;
    arm_mean_f32(&(TPhaseArray[0]), LENGTH_PhaseArray, &meanValue);	
		arm_std_f32(&(TPhaseArray[0]), LENGTH_PhaseArray, &stdValue);
		// 对数据进行 Z-score 标准化
		for (int i = 0; i < LENGTH_PhaseArray; i++) {
				TPhaseArray[0][i] = (TPhaseArray[0][i] - meanValue) / stdValue;
		}
		
    arm_mean_f32(&(TPhaseArray[1]), LENGTH_PhaseArray, &meanValue);	
		arm_std_f32(&(TPhaseArray[1]), LENGTH_PhaseArray, &stdValue);
		for (int i = 0; i < LENGTH_PhaseArray; i++) {
				TPhaseArray[1][i] = (TPhaseArray[1][i] - meanValue) / stdValue;
		}
	
		// 计算两个向量的点积
		float32_t dotProduct;
		arm_dot_prod_f32(&(TPhaseArray[0]), &(TPhaseArray[1]), LENGTH_PhaseArray, &dotProduct);		
		
		// 计算信号的平方和			
		float32_t squareSum1, squareSum2;
		arm_power_f32(&(TPhaseArray[0]), LENGTH_PhaseArray, &squareSum1); 
		arm_power_f32(&(TPhaseArray[1]), LENGTH_PhaseArray, &squareSum2);
		
		float32_t mul_result;
		arm_mult_f32(&squareSum1, &squareSum2, &mul_result, 1);
		
		float32_t sqrt_result;
		arm_sqrt_f32(mul_result, &sqrt_result);
		// 计算弧度				
		angle_rad = acos(dotProduct/sqrt_result);   
		// 将弧度转换为度数
//		float32_t degreeValue = 0.0f;
		degreeValue = angle_rad * (180.0f / PI);

}




