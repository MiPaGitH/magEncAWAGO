#include "u.h"
#include "main.h"

extern UART_HandleTypeDef huart3;

// Define a structure for menu items
typedef struct {
    char *optionName;
    char *helpText;
    void (*action)(void);
} MenuItem;

#define NB_OF_CLK_EDGES 71

enum
{
	eProgOTP,
	eReadEncVal
};

//values for activating the ABI interface
uint8_t otpData[2][2][NB_OF_CLK_EDGES+2] =
 {
  /*read*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1, 1 },
   /*DO*/  {1,1,          0,0, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,        1, 1, 1, 0, 0 }},
   /*write*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1, 1 },
   /*DO*/  {1,1,          1,1, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1, 0,0,0,0,0,0,0,0, 0,0,1,1,1,1,1,1,        1, 1, 0, 0, 0 }},


 };


#define nbOfElems 12

uint8_t OTPData[32]={0,}; //default values: all bits 0
uint8_t OTPDataBitPos[nbOfElems]={0,3,5,7,8,10,11,12,13,14,15,16};
uint8_t menuOTPDescr[]=
"0 1 2  3 4   56  7  8 9   10 11 12 13 14 15  16          ..                31  \r\n\
0 0 0  0 0   ..  0  0 0   0  0  0  0  0  0   0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0	\r\n\
|      |     |   |  |     |  |  |  |  |  |   | 									\r\n\
|      |     |   |  |     |  |  |  |  |  |   [16..31] Zero.Position = 0			\r\n\
|      |     |   |  |     |  |  |  |  |  [15] Mode_HI.DIR = 0					\r\n\
|      |     |   |  |     |  |  |  |  [14] Mode_HI.SpeedMode = 0				\r\n\
|      |     |   |  |     |  |  |  [13] Mode_HI.PWM Enable = 0					\r\n\
|      |     |   |  |     |  |  [12] Mode_HI.PWMPeriodSel = 0					\r\n\
|      |     |   |  |     |  [11] Mode_HI.IncrementalEn = 0						\r\n\
|      |     |   |  |     [10] Mode_HI.IncrementalSelect = 0  					\r\n\
|      |     |   |  [8,9] Mode_HI.AbsoluteResolution = 0						\r\n\
|      |     |   [7] modeLo.IndexOutputLevel = 0								\r\n\
|      |     [5,6] modeLo.unused bits = 0										\r\n\
|      [3,4] modeLo.IndexPulseWidth = 0											\r\n\
[0..2] modeLo.IncResolution = 0													\r\n";


void setConditionsFor(uint8_t modeOtpOrReadEnc);
void getEncVal( void );
void setOTPField( void );
void getOTPVal( void );
void programOTPData( void);

// Define menu items
MenuItem menu[] = {
    {"ge", "g - get encoder value", getEncVal},
	{"go", "o - get current OTP data", getOTPVal},
    {"so,", "s,<pos>,<0xval> - set OTP field; replace <pos> with the field bit position and <0xval> with the desired field value in hex", setOTPField},
    {"po", "p - program the OTP data", programOTPData}
};

uint8_t uartRxBuf[10];


uint8_t encResolution[]={10u,12u,14u,16u};

uint8_t menuSize;

uint8_t mPrompt[]="cmd:";

uint8_t uartMenu[1024];

typedef enum
{
	eInit = 0,
	eWaitRx,
	eWaitChoiceActions,
	eWaitTxEnd
}uStates;

uint8_t uState;

uint8_t fLedMagHiToggled;
uint8_t fLedMagLoToggled;


uint8_t ssiMode;

uint8_t *cOtpClk;
uint8_t *cOtpData;

uint8_t ssiClkEdgesNb;


uint8_t fUARTRx;
uint8_t fWaitSSITransfer;

uint32_t tim1Tick;

uint32_t cntWaitOTPWriteStatusFlags;

extern TIM_HandleTypeDef htim1;

void setConditionsFor(uint8_t modeOtpOrReadEnc)
{
	if ( eProgOTP == modeOtpOrReadEnc )
	{
		//TODO - enable 7V output to VPP pin; ALIGN, PWRDOWN – Set Low, PROG – Set High
	}
	else
	{
		//TODO - disable 7V output to VPP pin; ALIGN, PWRDOWN – Set Low, PROG – Set Low
	}
}

void getEncVal( void )
{
	uint8_t encResolutionOTPVal = (OTPData[9]<<1) + OTPData[8]; //bits 9 and 8 contain the resolution configuration
	setConditionsFor(eReadEncVal);
	ssiMode = read;

	cOtpClk = &otpData[ssiMode][0][0];
	cOtpData = &otpData[ssiMode][1][0];
	ssiClkEdgesNb = encResolution[encResolutionOTPVal]*2u;

	fWaitSSITransfer  = 1u;

	HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);

}

void getOTPVal( void )
{
	setConditionsFor(eReadEncVal);
	ssiMode = read;

	cOtpClk = &otpData[ssiMode][0][0];
	cOtpData = &otpData[ssiMode][1][0];
	ssiClkEdgesNb = NB_OF_CLK_EDGES;

	fWaitSSITransfer  = 1u;

	HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);
}

void setOTPField( void )
{
	//TODO - check the value passed after option name: so and update the otpData[1] buffer and menuOTPDescr with the values

}

void programOTPData( void)
{
	setConditionsFor(eProgOTP);
	ssiMode = write;

	cOtpClk = &otpData[ssiMode][0][0];
	cOtpData = &otpData[ssiMode][1][0];
	ssiClkEdgesNb = NB_OF_CLK_EDGES;

	fWaitSSITransfer  = 1u;
	cntWaitOTPWriteStatusFlags = 1000u;

	HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);
}

uint16_t prepareMenu( void ) {
	uint16_t lIdx = 0u;
	uint16_t lIdxuMenu = 0u;
	uint8_t lMenuItem = 0u;

	//TODO copy Encoder value data

	//TODO update actual OTP data according to the received data from device via SSI interface
	//TODO update the preset OTP data according to the values set via UART interface

	//copy OTP description
	for (/*lIdx already initialized above*/;lIdxuMenu<sizeof(menuOTPDescr); lIdxuMenu++)
	{
		uartMenu[lIdxuMenu] = menuOTPDescr[lIdxuMenu];
	}

	//copy Menu items
    for (/*lMenuItem initialized above*/;lMenuItem<  menuSize; lMenuItem++) {
    	for (lIdx = 0u; lIdx<sizeof(menu[lMenuItem].helpText); lIdx++)
    	{
    		uartMenu[lIdxuMenu] = menu[lMenuItem].helpText[lIdx];
    		lIdxuMenu++;
    	}
    }

	for (lIdx=0u; lIdx<sizeof(mPrompt); lIdx++)
	{
		uartMenu[lIdxuMenu] = mPrompt[lIdx];
		lIdxuMenu++;
	}

	return lIdxuMenu;

}

void processChoice(uint8_t choice[]) {
	uint8_t lIdx = 0u;
	uint8_t lMenuItem = 0u;
	uint8_t lFound = 0u;

	for (/*lMenuItem initialized above*/;lMenuItem<  menuSize; lMenuItem++)
	{
		lFound = 1u;
		for (lIdx = 0u; lIdx<sizeof(menu[lMenuItem].optionName); lIdx++)
		{
			if (choice[lIdx] != menu[lMenuItem].optionName[lIdx])
			{
				lFound = 0u;
			}
		}

		if ( 0u != lFound )
		{
			menu[lMenuItem].action();
		}
	}
}

void uInit( void )
{
	menuSize = 4u;
	uState = eInit;

	fWaitSSITransfer = 0u;
	ssiMode = read;
	ssiClkEdgesNb = 2u*32u;
	fUARTRx = 0u;
	cntWaitOTPWriteStatusFlags = 0u;

	cOtpClk = &otpData[0][0][0];
	cOtpData = &otpData[0][1][0];
}

void uTask( void ) {
	uint16_t uartDataSize = 0u;

	switch (uState)
	{
	case eInit:
		HAL_UART_Receive_DMA(&huart3, uartRxBuf, sizeof(uartRxBuf));
		uState=eWaitRx;

		break;
	case eWaitRx:
		if ( 0u != fUARTRx )
		{
			fUARTRx = 0u;
			processChoice(uartRxBuf);

			uState=eWaitChoiceActions;
		}
		break;
	case eWaitChoiceActions:
		if ( 0u == fWaitSSITransfer )
		{
			//TODO MiPa process SSI data if any - and prepare the menu accordingly
			uartDataSize = prepareMenu();
			HAL_UART_Transmit_DMA(&huart3, uartMenu, uartDataSize);

			uState=eWaitTxEnd;
		}
		break;
	case eWaitTxEnd:
		//TODO if ( )
		break;
	default:
		uState = eInit;
		break;
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_WritePin(GPIOA, ssiClk_Pin, cOtpClk[tim1Tick]);
	if (write == ssiMode)
	{
		HAL_GPIO_WritePin(GPIOA, ssiDO_Pin, cOtpData[tim1Tick]);
	}
	else
	{
		cOtpData[tim1Tick] = HAL_GPIO_ReadPin(GPIOA, ssiDO_Pin);
	}
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); //led shows the TX status

	if (tim1Tick< ssiClkEdgesNb)
	{
		tim1Tick++;
	}
	else
	{//TX finished
		if ( ( write == ssiMode ) && (cntWaitOTPWriteStatusFlags > 0u ) )
		{//MAG_HI and MAG_LO perform the functionality of OTP_ERR and OPT_PROG_STAT respectively
			if ( ( 0u == fLedMagHiToggled ) && ( 0u == fLedMagLoToggled ) )
			{
				cntWaitOTPWriteStatusFlags--;
			}
			else
			{
				cntWaitOTPWriteStatusFlags = 0u;
			}
		}
		else
		{
			tim1Tick=0u;
			HAL_TIM_Base_Stop_IT(&htim1);
			HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET); //led shows the TX status finished
			fWaitSSITransfer = 0u;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( USART3 == huart->Instance)
	{
		fUARTRx = 1u;
	}
}
