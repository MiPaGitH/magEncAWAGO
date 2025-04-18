#include "u.h"
#include "main.h"


static void setConditionsFor(uint8_t modeOtpOrReadEnc);
static void getEncVal( void );
static void setOTPField( void );
static void getOTPVal( void );
static void programOTPData( void);


#define NB_OF_CLK_EDGES 71
#define NB_OF_OTP_ELEMS 12


typedef struct {
    char *optionName;
    char *helpText;
    void (*action)(void);
} MenuItem;

enum
{
	eProgOTP,
	eReadEncVal,
	eReadOTPVal
};

typedef enum
{
	eInit = 0,
	eWaitRx,
	eWaitChoiceActions,
	eWaitSSITransfer,
	eWaitTxEnd
}uStates;


extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;


static uint8_t uartRxBuf[10];
static uint8_t encResolution[]={10u,12u,14u,16u};
static uint8_t menuSize;
static uint8_t mPrompt[]="cmd:";
static uint8_t uartMenu[1024];
static uint8_t uState;
static uint8_t fLedMagHiToggled;
static uint8_t fLedMagLoToggled;
static uint8_t ssiMode;
static uint8_t *cOtpClk;
static uint8_t *cOtpData;
static uint8_t ssiClkEdgesNb;
static uint8_t fUARTRx;
static uint8_t fWaitSSITransfer;
static uint8_t fParseSSIRxData;

static uint16_t encVal;

static uint32_t tim1Tick;
static uint32_t cntWaitOTPWriteStatusFlags;

//values for activating the ABI interface
static uint8_t otpData[2][2][NB_OF_CLK_EDGES+2] =
 {
  /*read*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1, 1 },
   /*DO*/  {1,1,          0,0, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,        1, 1, 1, 0, 0 }},
   /*write*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1, 1 },
   /*DO*/  {1,1,          1,1, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1, 0,0,0,0,0,0,0,0, 0,0,1,1,1,1,1,1,        1, 1, 0, 0, 0 }},


 };

static uint8_t OTPData[32]={0,}; //default values: all bits 0
static uint8_t OTPDataBitPos[NB_OF_OTP_ELEMS]={0,3,5,7,8,10,11,12,13,14,15,16};
static uint8_t menuOTPDescr[]=
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

// Define menu items
static MenuItem menu[] = {
    {"ge", "g - get encoder value", getEncVal},
	{"go", "o - get current OTP data", getOTPVal},
    {"so,", "s,<pos>,<0xval> - set OTP field; replace <pos> with the field bit position and <0xval> with the desired field value in hex", setOTPField},
    {"po", "p - program the OTP data", programOTPData}
};

static void setConditionsFor(uint8_t modeOtpOrReadEnc)
{
	if ( eProgOTP == modeOtpOrReadEnc )
	{
		//TODO - enable 6.5V output to VPP pin; ALIGN, PWRDOWN – Set Low, PROG – Set High
	}
	else
	{
		//TODO - disable 6.5V output to VPP pin; ALIGN, PWRDOWN – Set Low, PROG – Set Low
	}
}

static void getEncVal( void )
{
	uint8_t encResolutionOTPVal = (OTPData[9]<<1) + OTPData[8]; //bits 9 and 8 contain the resolution configuration
	setConditionsFor(eReadEncVal);
	ssiMode = read;

	cOtpClk = &otpData[ssiMode][0][0];
	cOtpData = &otpData[ssiMode][1][0];
	ssiClkEdgesNb = encResolution[encResolutionOTPVal]*2u;

	fWaitSSITransfer  = 1u;
	fParseSSIRxData = 1u;

	HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);

}

static void getOTPVal( void )
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

static void setOTPField( void )
{
	//TODO - check the value passed after option name: so and update the otpData[1] buffer and menuOTPDescr with the values

}

static void programOTPData( void)
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

static uint16_t prepareMenu( void ) {
	uint16_t lIdx = 0u;
	uint16_t lIdxuMenu = 0u;
	uint8_t lMenuItem = 0u;
	uint8_t encValDescr[] = "Encoder value: ";
	uint8_t encValTextRev[] = "65535";
	uint16_t lencVal = encVal;

	//copy encoder value
	for (/*lIdx already initialized above*/;lIdxuMenu<(sizeof(encValDescr)-1u); lIdxuMenu++)
	{
		uartMenu[lIdxuMenu] = encValDescr[lIdxuMenu];
	}
	//get digits
	encValTextRev[0] = encVal % 10u;
	lIdx = 1u;
	while (lencVal != 0u)
	{
		encValTextRev[lIdx++] = (lencVal / 10u) % 10u ;
	}
	//put encoder value digits in the buffer (in correct order from most significant digit to less significant one)
	while ( lIdx > 0u )
	{
		uartMenu[lIdxuMenu++] = encValTextRev[lIdx--];
	}
	uartMenu[lIdxuMenu++] = '\r';
	uartMenu[lIdxuMenu++] = '\n';

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

static void processChoice(uint8_t choice[]) {
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

static void processSSIRxData( void )
{
	uint8_t lIdx = 0u;
	uint8_t bitNb = 0u;
	uint16_t lencVal = 0u;

	for (lIdx=0u;lIdx<ssiClkEdgesNb;lIdx+=2u)
	{
		lencVal |= cOtpData[lIdx]<<bitNb; //cOtpData has only 0's and 1's
		bitNb++;
	}
	encVal = lencVal;
}



void uInit( void )
{
	menuSize = 4u;
	uState = eInit;

	fWaitSSITransfer = 0u;
	fParseSSIRxData = 0u;

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
			if ( 0u == fWaitSSITransfer )
			{//command does not start SSI transfer
				uState=eWaitChoiceActions;
			}
			else
			{//SSI transfer started
				uState = eWaitSSITransfer;
			}
		}
		break;
	case eWaitSSITransfer:
		if ( 0u == fWaitSSITransfer )
		{
			if ( 0u != fParseSSIRxData )
			{
				processSSIRxData();
			}
			uState=eWaitChoiceActions;
		}//else: wait some more
		break;
	case eWaitChoiceActions:

		uartDataSize = prepareMenu();
		HAL_UART_Transmit_DMA(&huart3, uartMenu, uartDataSize);

		uState=eWaitTxEnd;
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
		cOtpData[tim1Tick] = HAL_GPIO_ReadPin(GPIOA, ssiDO_Pin); //read on every edge but later use only the data sampled on falling edge
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
