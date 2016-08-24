// STM32 ADC IQ Sample @ 200 KHz (PC.0, PC.1) STM32F4 Discovery - sourcer32@gmail.com

#include <application.h>
#include "arm_math.h"
#include "audio.h"


////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION 
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////

int LEDS_ENABLED = 1;                  // Control if the LED's should display the spectrum or not.  1 is true, 0 is false.
                                       // Useful for turning the LED display on and off with commands from the serial port.

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEO_PIXEL_COUNT, NEO_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
//char commandBuffer[MAX_CHARS];
float frequencyWindow[NEO_PIXEL_COUNT+1];
float hues[NEO_PIXEL_COUNT];


/**************************************************************************************/
 
void RCC_Configuration(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /* ADC Channel 15 -> PC5 */
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
 
/**************************************************************************************/
 
void ADC_Configuration(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  ADC_InitTypeDef ADC_InitStructure;
 
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; // 2 half-words one by one, 1 then 2
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
 
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
 
  /* ADC1 pin A0 on Photon configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_15Cycles); // PC0
 
  /* Enable DMA request */

  ADC_DMACmd(ADC1, ENABLE);

  /* Enable DMA request after last transfer (Independent-ADC mode)  */

  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
 
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
}
 
/**************************************************************************************/

__IO int16_t ADCConvertedValues[BUFFERSIZE]; // treat this as 2 buffers, we get 2 ints, 1/2 and full

// to let the program know we need some variables
/* so here is how I think it can work. these variables when the normal program is waiting (reading) for the buffer to go full, 
 * the only thing that will write to the variable is the interrupt, so this is safe. once the interrupt has written to the variable
 * we have until the next interrupt before we would have set it to FALSE in the program. We can have the interrupt check and count or stop 
 * if it detects that the buffer was full before it sets it. That is what we will do and then we will know if we ever failed to process
 */

int firstBufferFull = FALSE;
int secondBufferFull = FALSE;

// keep track of if we are not processing fast enough

int overRunFirst = 0;
int overRunSecond = 0;

static void DMA_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues;
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR; // ADC1 data register address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFFERSIZE; // Count of 16-bit words
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
 
  /* Enable DMA Stream Half / Transfer Complete interrupt */
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);
 
  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA2_Stream0, ENABLE);
}
 
/**************************************************************************************/
 
void TIM3_Configuration(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = ((SystemCoreClock / 2) / SAMPLE_RATE_HZ) - 1; // XXX KHz, from 60 MHz TIM3CLK (ie APB1 = HCLK/4, TIM3CLK = HCLK/2)
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
  /* TIM3 TRGO selection */
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T3_TRGO
 
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}
 
/**************************************************************************************/

void DMA2_Stream0_IRQHandler(void) // Called every 256 samples at 20KHz it is every 12.8ms
{
	/* Test on DMA Stream Half Transfer interrupt */
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
	{
		/* Clear DMA Stream Half Transfer interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
 
		// we are not going to process this here, come on we are in an interrupt
		// so tell the program that the first half is full, if it was full count it 

		if (firstBufferFull)
		{
			overRunFirst++;
		}

		firstBufferFull = TRUE;
	}
 
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		// stop this for a test

		// we are not going to process this here, come on we are in an interrupt
		// so tell the program that the second half is full, if it was full count it 

		if (secondBufferFull)
		{
			overRunSecond++;
		}

		secondBufferFull = TRUE;
	}
}
 
/**************************************************************************************/
 
void NVIC_Audio_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
 
  /* Enable the DMA Stream IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  // here is how to attach to an interrupt the hacker way. */

  // get the vector base address

  char *vectors = (char *) SCB->VTOR;

  // the ISRs start at location 16, DMA2_Stream0 is 56 all this time 4 = 0x120 */

  vectors += (16 + DMA2_Stream0_IRQn) * sizeof(uint32_t);

  // put our function in place

  *(uint32_t *) vectors = (uint32_t) (DMA2_Stream0_IRQHandler);

  // enable the ingterrupt

   NVIC_Init(&NVIC_InitStructure);
}
 
/**************************************************************************************/

arm_cfft_radix4_instance_f32 fft_inst;

void setup()
{
	RCC_Configuration();
 
    GPIO_Configuration();
	
	NVIC_Audio_Configuration();
 
	TIM3_Configuration();
 
	DMA_Configuration();
 
	ADC_Configuration();

	pinMode(A4, OUTPUT);
	pinMode(A5, OUTPUT);

	digitalWrite(A4, LOW);
	digitalWrite(A5, LOW);

	arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);

	spectrumSetup();

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);
}

void squareWave(int16_t *buffer, int freq, int16_t max)
{
	int x;
	int count = 256 / (freq + 1);
	int value = max;

	if (count < 2)
	{
		count = 2;
	}

	for(x = 0 ; x < FFT_SIZE ; x++)
	{
		buffer[x] = value;

		if ((x % count) == (count - 1))
		{
			value = -value;
		}
	}
}

void ADCSquareWave(int freq, int16_t max)
{
	squareWave((int16_t *) ADCConvertedValues, freq, max);
}

int which = 0;

void sineWave(int16_t *buffer)
{
	int x;
	static int16_t data2[256] = {
		0x800,0x864,0x8c8,0x92c,0x98f,0x9f1,0xa52,0xab1,
		0xb0f,0xb6b,0xbc5,0xc1c,0xc71,0xcc3,0xd12,0xd5f,
		0xda7,0xded,0xe2e,0xe6c,0xea6,0xedc,0xf0d,0xf3a,
		0xf63,0xf87,0xfa7,0xfc2,0xfd8,0xfe9,0xff5,0xffd,
		0xfff,0xffd,0xff5,0xfe9,0xfd8,0xfc2,0xfa7,0xf87,
		0xf63,0xf3a,0xf0d,0xedc,0xea6,0xe6c,0xe2e,0xded,
		0xda7,0xd5f,0xd12,0xcc3,0xc71,0xc1c,0xbc5,0xb6b,
		0xb0f,0xab1,0xa52,0x9f1,0x98f,0x92c,0x8c8,0x864,
		0x800,0x79b,0x737,0x6d3,0x670,0x60e,0x5ad,0x54e,
		0x4f0,0x494,0x43a,0x3e3,0x38e,0x33c,0x2ed,0x2a0,
		0x258,0x212,0x1d1,0x193,0x159,0x123,0xf2,0xc5,
		0x9c,0x78,0x58,0x3d,0x27,0x16,0xa,0x2,
		0x0,0x2,0xa,0x16,0x27,0x3d,0x58,0x78,
		0x9c,0xc5,0xf2,0x123,0x159,0x193,0x1d1,0x212,
		0x258,0x2a0,0x2ed,0x33c,0x38e,0x3e3,0x43a,0x494,
		0x4f0,0x54e,0x5ad,0x60e,0x670,0x6d3,0x737,0x79b,

		0x800,0x864,0x8c8,0x92c,0x98f,0x9f1,0xa52,0xab1,
		0xb0f,0xb6b,0xbc5,0xc1c,0xc71,0xcc3,0xd12,0xd5f,
		0xda7,0xded,0xe2e,0xe6c,0xea6,0xedc,0xf0d,0xf3a,
		0xf63,0xf87,0xfa7,0xfc2,0xfd8,0xfe9,0xff5,0xffd,
		0xfff,0xffd,0xff5,0xfe9,0xfd8,0xfc2,0xfa7,0xf87,
		0xf63,0xf3a,0xf0d,0xedc,0xea6,0xe6c,0xe2e,0xded,
		0xda7,0xd5f,0xd12,0xcc3,0xc71,0xc1c,0xbc5,0xb6b,
		0xb0f,0xab1,0xa52,0x9f1,0x98f,0x92c,0x8c8,0x864,
		0x800,0x79b,0x737,0x6d3,0x670,0x60e,0x5ad,0x54e,
		0x4f0,0x494,0x43a,0x3e3,0x38e,0x33c,0x2ed,0x2a0,
		0x258,0x212,0x1d1,0x193,0x159,0x123,0xf2,0xc5,
		0x9c,0x78,0x58,0x3d,0x27,0x16,0xa,0x2,
		0x0,0x2,0xa,0x16,0x27,0x3d,0x58,0x78,
		0x9c,0xc5,0xf2,0x123,0x159,0x193,0x1d1,0x212,
		0x258,0x2a0,0x2ed,0x33c,0x38e,0x3e3,0x43a,0x494,
		0x4f0,0x54e,0x5ad,0x60e,0x670,0x6d3,0x737,0x79b
	};

	static int16_t data1[256] = {
		0x800,0x832,0x864,0x896,0x8c8,0x8fa,0x92c,0x95e,
		0x98f,0x9c0,0x9f1,0xa22,0xa52,0xa82,0xab1,0xae0,
		0xb0f,0xb3d,0xb6b,0xb98,0xbc5,0xbf1,0xc1c,0xc47,
		0xc71,0xc9a,0xcc3,0xceb,0xd12,0xd39,0xd5f,0xd83,
		0xda7,0xdca,0xded,0xe0e,0xe2e,0xe4e,0xe6c,0xe8a,
		0xea6,0xec1,0xedc,0xef5,0xf0d,0xf24,0xf3a,0xf4f,
		0xf63,0xf76,0xf87,0xf98,0xfa7,0xfb5,0xfc2,0xfcd,
		0xfd8,0xfe1,0xfe9,0xff0,0xff5,0xff9,0xffd,0xffe,
		0xfff,0xffe,0xffd,0xff9,0xff5,0xff0,0xfe9,0xfe1,
		0xfd8,0xfcd,0xfc2,0xfb5,0xfa7,0xf98,0xf87,0xf76,
		0xf63,0xf4f,0xf3a,0xf24,0xf0d,0xef5,0xedc,0xec1,
		0xea6,0xe8a,0xe6c,0xe4e,0xe2e,0xe0e,0xded,0xdca,
		0xda7,0xd83,0xd5f,0xd39,0xd12,0xceb,0xcc3,0xc9a,
		0xc71,0xc47,0xc1c,0xbf1,0xbc5,0xb98,0xb6b,0xb3d,
		0xb0f,0xae0,0xab1,0xa82,0xa52,0xa22,0x9f1,0x9c0,
		0x98f,0x95e,0x92c,0x8fa,0x8c8,0x896,0x864,0x832,
		0x800,0x7cd,0x79b,0x769,0x737,0x705,0x6d3,0x6a1,
		0x670,0x63f,0x60e,0x5dd,0x5ad,0x57d,0x54e,0x51f,
		0x4f0,0x4c2,0x494,0x467,0x43a,0x40e,0x3e3,0x3b8,
		0x38e,0x365,0x33c,0x314,0x2ed,0x2c6,0x2a0,0x27c,
		0x258,0x235,0x212,0x1f1,0x1d1,0x1b1,0x193,0x175,
		0x159,0x13e,0x123,0x10a,0xf2,0xdb,0xc5,0xb0,
		0x9c,0x89,0x78,0x67,0x58,0x4a,0x3d,0x32,
		0x27,0x1e,0x16,0xf,0xa,0x6,0x2,0x1,
		0x0,0x1,0x2,0x6,0xa,0xf,0x16,0x1e,
		0x27,0x32,0x3d,0x4a,0x58,0x67,0x78,0x89,
		0x9c,0xb0,0xc5,0xdb,0xf2,0x10a,0x123,0x13e,
		0x159,0x175,0x193,0x1b1,0x1d1,0x1f1,0x212,0x235,
		0x258,0x27c,0x2a0,0x2c6,0x2ed,0x314,0x33c,0x365,
		0x38e,0x3b8,0x3e3,0x40e,0x43a,0x467,0x494,0x4c2,
		0x4f0,0x51f,0x54e,0x57d,0x5ad,0x5dd,0x60e,0x63f,
		0x670,0x6a1,0x6d3,0x705,0x737,0x769,0x79b,0x7cd
	};

	for( x = 0 ; x < 256 ; x++ )
	{
		if (which == 1)
		{
			buffer[x] = data1[x];
		}
		else
		{
			buffer[x] = data2[x];
		}
	}
}

void ADCSineWave()
{
	sineWave( (int16_t *) ADCConvertedValues );
}

int graph = 0;
int fakeValue = 0;

float samples[FFT_SIZE * 2];
float magnitudes[FFT_SIZE];

void loop()
{
	int offset;
	int x;

	// We are pretending to have 2 buffers here, so we need to look to see if the first buffer is full 

	if (firstBufferFull)
	{
		offset = 0;

		digitalWrite(A5, HIGH);
	
		for (x = 0 ; x < FFT_SIZE ; x++ )
		{
	        samples[2 * x] = (float) ADCConvertedValues[offset + x];
			samples[(2 * x) + 1] = 0.0;
		}

		arm_cfft_radix4_f32(&fft_inst, samples);

		// Calculate magnitude of complex numbers output by the FFT.

		arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);


		// let the interrupt know it is safe

		firstBufferFull = FALSE;
 
		if (LEDS_ENABLED == 1)
		{
			spectrumLoop();
		}

		digitalWrite(A5, LOW);
	}

	if (secondBufferFull)
	{
		offset = FFT_SIZE;

		digitalWrite(A4, HIGH);

		for (x = 0 ; x < FFT_SIZE ; x++ )
		{
	        samples[2 * x] = (float) ADCConvertedValues[offset + x];
			samples[(2 * x) + 1] = 0.0;
		}

		arm_cfft_radix4_f32(&fft_inst, samples);

		// Calculate magnitude of complex numbers output by the FFT.

		arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

		// let the interrupt know it is safe

		secondBufferFull = FALSE;

		if (LEDS_ENABLED == 1)
		{
			spectrumLoop();
		}

		digitalWrite(A4, LOW);

	}

	if (fakeValue)
	{
		System.dfu();
		ADCSineWave();
		ADCSquareWave(1, 2048);
	}
}
