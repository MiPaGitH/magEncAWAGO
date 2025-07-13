#include "u.h"
#include "main.h"


static void setConditionsFor(uint8_t reqAction);
static void getEncVal( void );
static void setOTPField( void );
static void getOTPVal( void );
static void programOTPData( void);

#define NB_OF_OTP_RDWR_BITS 1
#define NB_OF_OTP_DATA_BITS 32
#define NB_OF_OTP_PAR_BITS	1
#define NB_OF_OTP_BITS	(NB_OF_OTP_RDWR_BITS + NB_OF_OTP_DATA_BITS + NB_OF_OTP_PAR_BITS)


#define NB_OF_CLK_EDGES (2/*1 then 0 first falling edge*/ + (2*NB_OF_OTP_BITS) + 2/*HIGH bits at the end*/)
#define NB_OF_OTP_ELEMS 12


typedef struct {
    char optionName[2];
    char helpText[100];
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

uint8_t iCnt;


static uint8_t uartRxBuf[12]; //max command is so16,65535\r\n
static uint8_t encResolution[]={10u,12u,14u,16u};
static uint8_t menuSize;
static uint8_t mErrOk[]="command executed successfully\r\n";
static uint8_t mErrNOk[]="invalid command\r\n";
static uint8_t mPrompt[]="cmd:";
static uint8_t uartMenu[2048];
static uStates uState;
static uint8_t fLedMagHiToggled;
static uint8_t fLedMagLoToggled;
static uint8_t ssiMode;
static uint8_t *cOtpClk;
static uint8_t *cOtpData;
static uint8_t ssiClkEdgesNb;
static uint8_t fUARTRx;
static uint8_t timUARTRx;
static uint8_t fUARTTx;
static uint8_t fWaitSSITransfer;
static uint8_t fParseSSIRxData;
static uint8_t fInvalidCommand;

static uint16_t encVal;

static uint32_t tim1Tick;
static uint32_t cntWaitOTPWriteStatusFlags;

static uint8_t encData[2][35]=
{
  /*CLK*/ {1,0, /*data*/1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1},
  /*DI*/  {1,1, /*data*/1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1},
};

static uint8_t otpData[2][2][NB_OF_CLK_EDGES] =
{
  /*read*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, /*data*/1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1 },
   /*DO*/  {1,1,          0,0, /*data*/1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,        1, 1, 1, 0 }},
   /*write*/
  {/*CLK*/ {1,0, /*RD/WR*/1,0, /*data*/1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, 1,0,1,0,1,0,1,0, /*par*/1, 0, 1, 1 },
   /*DI*/  {1,1,          1,1, /*data*/0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1, 0,0,0,0,0,0,0,0, 0,0,1,1,1,1,1,1,        1, 1, 0, 0 }},
};
static uint16_t OTPDataRanges[]={7,0,0,3,0,0,0,1,3,0,1,1,1,1,1,1,65535,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint16_t OTPDataBitNo[]=	{3,0,0,2,0,0,0,1,2,0,1,1,1,1,1,1,16	  ,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint16_t OTPDataElementsFormating[]={2,2,3,2,3,2,3,3,2,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,0};


//static uint8_t OTPDataBitPos[NB_OF_OTP_ELEMS]={0,3,5,7,8,10,11,12,13,14,15,16};
static uint8_t menuOTPDescrHeader[]=
"bit nb. \r\n0 1 2  3 4  5 6  7  8 9  10 11 12 13 14 15 16          ..                31  \r\n\
actual \r\n";
static uint8_t menuOTPDescrActualData[]=
"0 0 0  0 0  . .  0  1 1  0  0  0  0  0  0  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0	\r\n\
desired \r\n";
static uint8_t menuOTPDescrDesiredData[]=
"0 0 0  0 0  . .  0  0 0  0  0  0  0  0  0  0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0	\r\n\
|      |    |    |  |    |  |  |  |  |  |  | 								\r\n\
|      |    |    |  |    |  |  |  |  |  |  [16..31] Zero.Position = 0		\r\n\
|      |    |    |  |    |  |  |  |  |  [15] Mode_HI.DIR = 0				\r\n\
|      |    |    |  |    |  |  |  |  [14] Mode_HI.SpeedMode = 0				\r\n\
|      |    |    |  |    |  |  |  [13] Mode_HI.PWM Enable = 0				\r\n\
|      |    |    |  |    |  |  [12] Mode_HI.PWMPeriodSel = 0				\r\n\
|      |    |    |  |    |  [11] Mode_HI.IncrementalEn = 0					\r\n\
|      |    |    |  |    [10] Mode_HI.IncrementalSelect = 0  				\r\n\
|      |    |    |  [8,9] Mode_HI.AbsoluteResolution = 0					\r\n\
|      |    |    [7] modeLo.IndexOutputLevel = 0							\r\n\
|      |    [5,6] modeLo.unused bits = 0									\r\n\
|      [3,4] modeLo.IndexPulseWidth = 0										\r\n\
[0..2] modeLo.IncResolution = 0												\r\n";


//TODO MiPa test so command; ge, go and po look OK in logic analyzer; TODO test them with real encoder attached;

// Define menu items
static MenuItem menu[] = {
    {"ge", "ge - get encoder value\r\n", getEncVal},
	{"go", "go - get current OTP data\r\n", getOTPVal},
    {"so","so<pos>,<val> - set OTP field; <pos> bit position\r\n", setOTPField},
    {"po", "po - program the OTP data\r\n", programOTPData}
};

static char dbgChoice[10];

uint32_t myAtoUi(const uint8_t *str, uint8_t *status) {
    uint32_t i = 0u;
    uint32_t result = 0u;

    *status = mOK;

    // Convert characters to integer
    while (str[i] >= '0' && str[i] <= '9') {
        int digit = str[i] - '0';

        result = result * 10 + digit;
        i++;
    }

    if (str[i] != '\0' && str[i] != '\r' )
    {
    	*status = mNOK;
    	result = 0xFFFFFFFFu;
    }

    return result;
}

static void setConditionsFor(uint8_t reqAction)
{//Note: make sure ALIGN and PWRDOWN are set to Low
	if ( eProgOTP == reqAction )
	{
		HAL_GPIO_WritePin(pPROG_GPIO_Port, pPROG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(pVPP_GPIO_Port, pVPP_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(pPROG_GPIO_Port, pPROG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(pVPP_GPIO_Port, pVPP_Pin, GPIO_PIN_RESET);
	}
}

static void getEncVal( void ) //TODO MiPa AICI add nbOfSamples parameter and print all the sample values before showing again the menu (use fixed sample time of 200 milliseconds)
{
	uint8_t encResolutionOTPVal = (otpData[read][dat][4+8*2]<<1) + otpData[read][dat][4+9*2]; //bits 9 and 8 contain the resolution configuration
	setConditionsFor(eReadEncVal);
	ssiMode = read;

	cOtpClk = &encData[clk][0];
	cOtpData = &encData[dat][0];
	ssiClkEdgesNb = 2u/*first falling edge*/ + encResolution[encResolutionOTPVal]*2u + 1u/*rising edge at the end*/;

	fWaitSSITransfer  = 1u;
	fParseSSIRxData = 1u;

	HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);

}

static void getOTPVal( void )
{
	setConditionsFor(eReadEncVal);
	ssiMode = read;

	cOtpClk = &otpData[read][clk][0];
	cOtpData = &otpData[read][dat][0];
	ssiClkEdgesNb = NB_OF_CLK_EDGES;

	fWaitSSITransfer  = 1u;

	HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);
}

static void setOTPField( void )
{
	uint8_t lIdx = 0u;
	uint8_t bpVal = 0u;
	uint32_t lOTPval = 0u;
	uint8_t lOTPvalBits[16] = {0,};
	uint8_t conversionStatus;

	//command format so<bitNb>,<hexVal> (max 4 characters for value - Zero.poasition has 16 bits)
	lIdx = 2u;
	while ( (lIdx < 7u) && (uartRxBuf[lIdx] != ',') )
	{
		bpVal *= 10;
		bpVal += uartRxBuf[lIdx]-'0';
		lIdx++;
	}

	if ( bpVal <= 16u )
	{//bpVal in valid range
		lIdx++;
		//convert value from hex string to INT
		lOTPval = myAtoUi(&uartRxBuf[lIdx],&conversionStatus);
		if ( mOK == conversionStatus )
		{//Conversion successful
			if ( (OTPDataRanges[bpVal] != 0) && (OTPDataRanges[bpVal] >= lOTPval) )
			{//value is in range
				//update the otpData buffer used for writing
				lIdx = 0u;
				while (0u != lOTPval)
				{
					lOTPvalBits[lIdx] = lOTPval % 2;
					lIdx++;
					lOTPval >>= 1;
				}
				lIdx = OTPDataBitNo[bpVal];
				while (lIdx > 0)
				{
					lIdx--;
					otpData[write][dat][ 4 + (2 * (bpVal+lIdx)) 	] = lOTPvalBits[lIdx];
					otpData[write][dat][ 4 + (2 * (bpVal+lIdx)) +1 	] = lOTPvalBits[lIdx];
				}
			}
			else
			{//conversion error
				fInvalidCommand = 1u;
			}
		}
		else
		{//conversion error
			fInvalidCommand = 1u;
		}
	}
	else
	{//invalid bit number
		fInvalidCommand = 1u;
	}
}

static void programOTPData( void)
{
	setConditionsFor(eProgOTP);
	ssiMode = write;

	cOtpClk = &otpData[write][clk][0];
	cOtpData = &otpData[write][dat][0];
	ssiClkEdgesNb = NB_OF_CLK_EDGES;

	fWaitSSITransfer  = 1u;
	cntWaitOTPWriteStatusFlags = 1000u;

	HAL_GPIO_WritePin(GPIOA, ssiNCS_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim1);
}

static uint16_t prepareMenu( void ) {
	uint16_t lIdx = 0u;
	uint16_t lIdxuMenu = 0u;
	uint16_t lIdxOtpMenu = 0u;
	uint8_t lMenuItem = 0u;
	uint8_t encValDescr[] = "\r\nEncoder value: ";
	uint8_t encValTextRev[] = "65535";
	uint16_t lencVal = encVal;

	//copy encoder value
	for (/*lIdx already initialized above*/;lIdxuMenu<(sizeof(encValDescr)-1u); lIdxuMenu++)
	{
		uartMenu[lIdxuMenu] = encValDescr[lIdxuMenu];
	}
	//get digits
	encValTextRev[0] = lencVal % 10u;
	lIdx = 1u;
	while (lencVal != 0u)
	{
		lencVal /= 10u;
		encValTextRev[lIdx++] = lencVal % 10u ;
	}
	//put encoder value digits in the buffer (in correct order from most significant digit to less significant one)
	while ( lIdx > 0u )
	{
		uartMenu[lIdxuMenu++] = encValTextRev[lIdx--]+'0';
	}
	uartMenu[lIdxuMenu++] = '\r';
	uartMenu[lIdxuMenu++] = '\n';

	//update actual OTP data according to the received data from device via SSI interface
	for ( lIdx = 0u; lIdx < 32u; lIdx++)
	{
		menuOTPDescrActualData[lIdxOtpMenu]= '0' + otpData[read][dat][4+lIdx*2];
		lIdxOtpMenu += OTPDataElementsFormating[lIdx];
	}
	lIdxOtpMenu=0u;
	//update desired OTP data according to the received data from device via SSI interface
	for ( lIdx = 0u; lIdx < 32u; lIdx++)
	{
		menuOTPDescrDesiredData[lIdxOtpMenu]= '0' + otpData[write][dat][4+lIdx*2];
		lIdxOtpMenu += OTPDataElementsFormating[lIdx];
	}

	//copy OTP description
	for (lIdx = 0u; lIdx < sizeof(menuOTPDescrHeader); lIdx++)
	{
		uartMenu[lIdxuMenu] = menuOTPDescrHeader[lIdx];
		lIdxuMenu++;
	}
	for (lIdx = 0u; lIdx < sizeof(menuOTPDescrActualData); lIdx++)
	{
		uartMenu[lIdxuMenu] = menuOTPDescrActualData[lIdx];
		lIdxuMenu++;
	}
	for (lIdx = 0u; lIdx < sizeof(menuOTPDescrDesiredData); lIdx++ )
	{
		uartMenu[lIdxuMenu] = menuOTPDescrDesiredData[lIdx];
		lIdxuMenu++;
	}

	//copy Menu items
    for (/*lMenuItem initialized above*/;lMenuItem<  menuSize; lMenuItem++) {
    	for (lIdx = 0u; lIdx<sizeof(menu[lMenuItem].helpText); lIdx++)
    	{
    		uartMenu[lIdxuMenu] = menu[lMenuItem].helpText[lIdx];
    		lIdxuMenu++;
    	}
    }
    if ( 1u == fInvalidCommand )
    {//command NOK
    	fInvalidCommand = 0u;
    	for (lIdx=0u; lIdx<sizeof(mErrNOk); lIdx++)
    	{
    		uartMenu[lIdxuMenu] = mErrNOk[lIdx];
    		lIdxuMenu++;
    	}
    }
    else if ( 0u == fInvalidCommand)
    {//command OK
    	for (lIdx=0u; lIdx<sizeof(mErrOk); lIdx++)
    	{
    		uartMenu[lIdxuMenu] = mErrOk[lIdx];
    		lIdxuMenu++;
    	}
    }else{/*no command was executed - nothing to print*/}

	for (lIdx=0u; lIdx<sizeof(mPrompt); lIdx++)
	{
		uartMenu[lIdxuMenu] = mPrompt[lIdx];
		lIdxuMenu++;
	}

	return lIdxuMenu;

}

static void processChoice(uint8_t *choice) {
	uint8_t lIdx = 0u;
	uint8_t lMenuItem = 0u;
	uint8_t lFound = 0u;

	while ( (lMenuItem<menuSize) && (lFound != 1u) )
	{
		lFound = 1u;
		for (lIdx = 0u; lIdx<sizeof(menu[lMenuItem].optionName); lIdx++)
		{
			if (choice[lIdx] != menu[lMenuItem].optionName[lIdx])
			{
				dbgChoice[lIdx] = choice[lIdx];
				lFound = 0u;
			}
		}

		if ( 0u != lFound )
		{
			menu[lMenuItem].action();
		}

		lMenuItem++;
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

	fInvalidCommand = 0xFFu;

	cOtpClk = &otpData[read][clk][0];
	cOtpData = &otpData[read][dat][0];

	iCnt = 0u;
}

void uTask( void ) {
	uint16_t uartDataSize = 0u;

	if ( timUARTRx > 0u )
	{
		timUARTRx--;
		if ( 0u == timUARTRx )
		{//time just expired
			fUARTRx = 1u;
		}
	}

	switch (uState)
	{
	case eInit:
		if ( iCnt > 200u )
		{
			uState = eWaitChoiceActions; //in this state the menu will be sent over UART to the PC terminal application
		}
		else iCnt++;

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
		if ( 0u != fUARTTx )
		{
			fUARTTx = 0u;

			HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uartRxBuf, sizeof(uartRxBuf));
			uState=eWaitRx;
		}
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
		HAL_GPIO_WritePin(GPIOA, ssiDIO_Pin, cOtpData[tim1Tick]);
	}
	else
	{
		cOtpData[tim1Tick] = HAL_GPIO_ReadPin(GPIOA, ssiDIO_Pin); //read on every edge but later use only the data sampled on falling edge
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
		timUARTRx = 0u;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) //called when idle event happens after receiving a number of bytes less than SIZEOF(uartRxBuf) (the 3'RD parameter of HAL_UARTEx_ReceiveToIdle_DMA function
{
	if ( USART3 == huart->Instance)
	{
		timUARTRx = 200u; //wait time until command is completely received
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( USART3 == huart->Instance)
	{
		fUARTTx = 1u;
	}
}
