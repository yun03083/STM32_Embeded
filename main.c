#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "core_cm3.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"
#include "stm32f10x_dma.h"
#include "STM32vldiscovery.h"
#include <time.h>
//#include "stm32f10x_dma.c"

#define CPU_FREQ	72000000
#define PRESCALER 	72

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

const uint32_t marioMelody[] = {
NOTE_E7, NOTE_E7, 0, NOTE_E7, 0, NOTE_C7, NOTE_E7, 0,
NOTE_G7, 0, 0, 0,
NOTE_G6, 0, 0, 0,

NOTE_C7, 0, 0, NOTE_G6, 0, 0, NOTE_E6, 0, 0, NOTE_A6, 0, NOTE_B6, 0, NOTE_AS6,
		NOTE_A6, 0,

		NOTE_G6, NOTE_E7, NOTE_G7,
		NOTE_A7, 0, NOTE_F7, NOTE_G7, 0, NOTE_E7, 0, NOTE_C7,
		NOTE_D7, NOTE_B6, 0, 0,

		NOTE_C7, 0, 0, NOTE_G6, 0, 0, NOTE_E6, 0, 0, NOTE_A6, 0, NOTE_B6, 0,
		NOTE_AS6, NOTE_A6, 0,

		NOTE_G6, NOTE_E7, NOTE_G7,
		NOTE_A7, 0, NOTE_F7, NOTE_G7, 0, NOTE_E7, 0, NOTE_C7,
		NOTE_D7, NOTE_B6, 0, 0 };

const uint32_t secondMelody[] = {
NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_A4,
NOTE_G4, NOTE_C5, NOTE_AS4, NOTE_A4,
NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_FS4, NOTE_DS4, NOTE_D4,
NOTE_C4, NOTE_D4, 0,

NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_A4,
NOTE_G4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_AS4, NOTE_C5, NOTE_AS4,
		NOTE_A4,      //29               //8
		NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_FS4, NOTE_DS4, NOTE_D4,
		NOTE_C4, NOTE_D4, 0,

		NOTE_D4, NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_DS5, NOTE_D5,
		NOTE_C5, NOTE_AS4, NOTE_A4, NOTE_C5,
		NOTE_C4, NOTE_D4, NOTE_DS4, NOTE_FS4, NOTE_D5, NOTE_C5,
		NOTE_AS4, NOTE_A4, NOTE_C5,
		NOTE_AS4,             //58

		NOTE_D4, NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_DS5, NOTE_D5,
		NOTE_C5, NOTE_D5, NOTE_C5, NOTE_AS4, NOTE_C5, NOTE_AS4, NOTE_A4,
		NOTE_C5, NOTE_G4,
		NOTE_A4, 0, NOTE_AS4, NOTE_A4, 0, NOTE_G4,
		NOTE_G4, NOTE_A4, NOTE_G4, NOTE_FS4, 0,

		NOTE_C4, NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_DS4,
		NOTE_C4, NOTE_D4, 0,
		NOTE_C4, NOTE_D4, NOTE_G4, NOTE_FS4, NOTE_DS4,
		NOTE_C4, NOTE_D4 };

const uint32_t marioDuration[] = { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
		12, 12, 12, 12, 12,

		12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,

		9, 9, 9, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,

		12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,

		9, 9, 9, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, };

const uint32_t secondDuration[] = { 8, 4, 8, 4, 4, 4, 4, 12, 4, 4, 4, 4, 4, 4,
		4, 16, 4,

		8, 4, 8, 4, 4, 2, 1, 1, 2, 1, 1, 12, 4, 4, 4, 4, 4, 4, 4, 16, 4,

		4, 4, 4, 4, 4, 4, 4, 4, 4, 12, 4, 4, 4, 4, 4, 4, 4, 4, 4, 12,

		4, 4, 4, 4, 4, 4, 2, 1, 1, 2, 1, 1, 4, 8, 4, 2, 6, 4, 2, 6, 4, 2, 1, 1,
		16, 4,

		4, 8, 4, 4, 4, 4, 16, 4, 4, 8, 4, 4, 4, 4, 20, };

const uint32_t* melody[] = { marioMelody, secondMelody };
const uint32_t* noteDurations[] = { marioDuration, secondDuration };
const uint16_t melodySlowfactor[] = { 15, 30 };

const uint32_t melodySizes[] = { sizeof(marioMelody) / sizeof(uint32_t),
		sizeof(secondDuration) / sizeof(uint32_t) };

void buzzerSetNewFrequency(uint32_t newFreq) {
	uint64_t tempFreq = newFreq;
	uint64_t tempNewValue;
	if (newFreq == 0)
		tempFreq = 1;

	tempNewValue = (uint64_t) CPU_FREQ / PRESCALER / tempFreq;

	// setting new value
	TIM2->ARR = (uint32_t) tempNewValue;
	TIM2->CCR4 = (uint32_t) tempNewValue / 2;

}

int ADC_Value = 1;

int isLedOff = 1;
int isOn = 1;
int color[12] = { WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN,
		BRRED, GRAY };

/*
 * 낮은옥타브
 C131 - 282*1949
 C#139 - 126*4111
 D147 - 15*32653
 D#156 - 162*2849
 E165 - 59*3698
 F175 - 292*1409
 F#185 - 30*12973
 G196 - 266*1381
 G#208 - 34*10181
 A220 - 88*3719
 A#233 - 132*2341
 B247 - 114*2557
 */

int melodyNodes[8][2] = { { 282, 1949 }, { 15, 32653 }, { 59, 3698 }, { 292,
		1409 }, { 266, 1381 }, { 88, 3719 }, { 114, 2557 }, { 141, 1949 } };

/*
 * 중간옥타브
 C262 - 141*1949
 C#277 - 8*32491
 D294 - 104*349
 D#311 - 7*33073
 E330 - 59*3698
 F349 - 96*2149
 F#370 - 298 * 653
 G392 - 133*1381
 G#415 - 389*446
 A440 - 44*3719
 A#466 - 66*2341
 B494 - 57*2557
 */

int melody_mid[8][2] = { { 141, 1949 }, // 도
		{ 9, 27211 }, // 레
		{ 59, 3698 }, // 미
		{ 96, 2149 }, // 파
		{ 133, 1381 }, // 솔
		{ 44, 3719 }, // 라
		{ 57, 2557 }, // 시
		{ 327, 421 } // 도
};

/*
 * 높은옥타브
 C523 - 327*421
 C#554 - 4*32491
 D587 - 6*20443
 D#622 - 172*673
 E659 - 56*1951
 F698 - 307*336
 F#740 - 149*653
 G784 - 36*2551
 G#831 - 2*43321
 A880 - 22*3719
 A#932 - 33*2341
 B988 - 166*439
 */

int melody_high[8][2] = { { 327, 421 }, // 도
		{ 6, 20443 }, // 레
		{ 20, 10909 }, // 미
		{ 38, 5429 }, // 파
		{ 36, 2551 }, // 솔
		{ 336, 487 }, // 라
		{ 332, 439 }, // 시
		{ 224, 307 } // 도

};

__IO uint32_t ADC_DualConvertedValueTab[12];
//flash load "C:\Users\Team08\Desktop\team66\team6_2\Debug\flashclear.axf"
//flash load "C:\Users\Team08\Desktop\team66\team6_2\Debug\team6_2.axf"

void RCC_Configure() {
	GPIO_InitTypeDef gpio1;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//new
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	gpio1.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio1.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
	gpio1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpio1);
}

void configureSpecficGPIOPin() {

}

void GPIO_Configure() {

	// GPIOA
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure3;
	int GPIOPins[10] = { GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3,
			GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6, GPIO_Pin_7 };
	int i = 0;
	// A
	for (i = 0; i < 8; i++) {
		GPIO_InitStructure.GPIO_Pin = GPIOPins[i];
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}

	// B
	for (i = 0; i < 2; i++) {
		GPIO_InitStructure.GPIO_Pin = GPIOPins[i];
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}

	// C
	for (i = 0; i < 6; i++) {
		GPIO_InitStructure.GPIO_Pin = GPIOPins[i];
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	}

	// boozer

	GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure3);

}

void ADC_Configure() {
	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	int i = 0;
	ADC_InitTypeDef ADC_InitStructure;
	int adc_channels[12] = { ADC_Channel_1, ADC_Channel_5, ADC_Channel_6,
			ADC_Channel_7,
			ADC_Channel_8, ADC_Channel_9, ADC_Channel_10, ADC_Channel_11,
			ADC_Channel_12, ADC_Channel_14, ADC_Channel_15 };
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 12;

	for (i = 1; i < 13; i++) {
		ADC_RegularChannelConfig(ADC1, adc_channels[i - 1], i,
				ADC_SampleTime_55Cycles5);
	}
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_55Cycles5);
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1))
		;
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1))
		;
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t tim_period = 170;
uint16_t prescalar = 1000;

void TIM_Configure() {
	TIM_TimeBaseInitTypeDef tim;
	TIM_OCInitTypeDef OutputChannel;
	tim.TIM_ClockDivision = 0;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = tim_period;
	tim.TIM_Prescaler = prescalar;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &tim);

	TIM_ARRPreloadConfig(TIM2, ENABLE);

	/* TIM2 PWM Initialize */
	OutputChannel.TIM_OCMode = TIM_OCMode_PWM1;
	OutputChannel.TIM_OutputState = TIM_OutputState_Enable;
	//OutputChannel.TIM_OutputNState=TIM_OutputNState_Enable;
	OutputChannel.TIM_Pulse = 100; // 50% duty ratio
	OutputChannel.TIM_OCPolarity = TIM_OCPolarity_Low;
	//OutputChannel.TIM_OCNPolarity=TIM_OCNPolarity_High;
	//OutputChannel.TIM_OCIdleState=TIM_OCIdleState_Set;
	//OutputChannel.TIM_OCNIdleState=TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM2, &OutputChannel);

	//TIM_Cmd(TIM2, ENABLE);
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/* TIM2 Enale */
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, ENABLE); // interrupt enable

	//TIM2->ARR = tim_period;
	//TIM2->PSC = prescalar;
	//TIM2->CR1  |= TIM_CR1_CEN;

}

void NVIC_Configure() {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure2;

	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure2.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure2.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure2.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure2);
}

/*void ADC1_2_IRQHandler() {
 uint16_t input;
 LCD_ShowString(1, 1, "Tue_TEAM06", BLACK, WHITE);
 if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {

 input = ADC_GetConversionValue(ADC1);
 LCD_ShowNum(1, 1, (u16)input, 10, BLACK, WHITE);
 }

 ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

 }
 */
int abc = 0, def = 0;

void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the interrupt flag
		GPIOC->BRR = GPIO_Pin_3;  // PB0 OFF
	}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		GPIOC->BSRR = GPIO_Pin_3;  // PB0 ON
		tim_period += 1;
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void DMA_init() {
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ADC_DualConvertedValueTab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 12; // 12로 바꿈
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void delay() {
	int i = 0;
	for (i = 0; i < 100000; i++) {
	}
}
int notes[10] = { NOTE_C7, NOTE_D7, NOTE_E7, NOTE_F7, NOTE_G7, NOTE_A7, NOTE_B7,
		NOTE_C8 };
uint32_t fsr_values[12];
int pulse_comp[8];

int getMax(int a, int b) {
	if (a > b) {
		return 1;
	} else {
		return 0;
	}
}

//flash load "C:\Users\Team08\Desktop\team66\team6_2\Debug\flashclear.axf"
//flash load "C:\Users\Team08\Desktop\team66\team6_2\Debug\team6_2.axf"

//flash load "C:\Users\Team08\Desktop\team66\team6_2\Debug\flashclear.axf"
//flash load "C:\Users\Team08\Desktop\team66\team6_2\Debug\team6_2.axf"

int main() {
	//uint32_t x = 777;
	//uint32_t y = 0;
	int cnt_melody = 0;
	int heuristic_value = 1600;

	//int preDefinedIndex[8] = { 3, 2, 1, 0, 4, 5, 6, 7 };
	int preDefADCIdx[8] = { 7, 0, 1, 3, 6, 2, 9, 4 };

	int minPulseIdx = -1;
	int minPulseValue = 1500;
	int i = 0;
	SystemInit();
	RCC_Configure();


	GPIO_Configure();
	DMA_init();
	ADC_Configure();

	GPIOD->CRL = (GPIO_CRL_MODE2_0 | GPIO_CRL_MODE3_0 | GPIO_CRL_MODE4_0| GPIO_CRL_MODE7_0);


	NVIC_Configure();
	TIM_Configure();
	TIM2->CR1 = 0;


	//buzzerSetNewFrequency(0);
	//LCD_Init();
	//Touch_Configuration();
	//Touch_Adjust();
	//LCD_Clear(WHITE);

	//LCD_ShowString(1, 1, "Tue_TEAM06", BLACK, WHITE);

	//GPIO_SetBits(GPIOD,GPIO_Pin_2);
	//GPIO_SetBits(GPIOD, GPIO_Pin_3);
	//GPIO_SetBits(GPIOD,GPIO_Pin_4);
	//GPIO_SetBits(GPIOD, GPIO_Pin_7);

	GPIO_SetBits(GPIOD,GPIO_Pin_2);
	GPIO_SetBits(GPIOD, GPIO_Pin_3);
	GPIO_SetBits(GPIOD,GPIO_Pin_4);
	GPIO_SetBits(GPIOD, GPIO_Pin_7);
	while (1) {
		for (i = 0; i < 12; i++) {
			//char strs[12][4] = {"PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PB0", "PB1", "PC0", "PC1"};
			fsr_values[i] = ADC_DualConvertedValueTab[i];
			//LCD_ShowString(i, i * 20, strs[i], 10, BLACK, WHITE);
			//LCD_ShowNum(1, i * 20, (u32) fsr_values[i], 10, BLACK, WHITE);
		}
		minPulseIdx = -1;
		minPulseValue = 1500;
		for (i = 0; i < 8; i++) {
			int v = ADC_DualConvertedValueTab[preDefADCIdx[i]];
			if (v > 1500)
				continue;
			if (minPulseValue > v) {
				minPulseValue = v;
				minPulseIdx = i;
			}
		}
		if (minPulseIdx != -1) {
			TIM2->CR1 = 1;
			buzzerSetNewFrequency(notes[minPulseIdx]);
		} else if (minPulseIdx == -1) {
			TIM2->CR1 = 0;
			//buzzerSetNewFrequency(0);
		}


		//delay();

	}

}
