#include <stdio.h>
#include "core_cm3.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"
#include "misc.h"

int x=0;
TIM_OCInitTypeDef TIM_OC_CH3, TIM_OC_CH4;

__IO uint32_t ADC_Val[4]; //조도 센서  
char passive_cntrl_data = NULL; //블루투스로 모터 조정
int mode = 1; //0:자동모드 1:수동모드  

void SysInit(void) {
    /* Set HSION bit */
    /* Internal Clock Enable */
    RCC->CR |= (uint32_t)0x00000001; //HSION
    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    RCC->CFGR &= (uint32_t)0xF0FF0000;
    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= (uint32_t)0xFEF6FFFF;
    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t)0xFFFBFFFF;
    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    RCC->CFGR &= (uint32_t)0xFF80FFFF;
    /* Reset PLL2ON and PLL3ON bits */
    RCC->CR &= (uint32_t)0xEBFFFFFF;
    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x00FF0000;
    /* Reset CFGR2 register */
    RCC->CFGR2 = 0x00000000;
}

void SetSysClock(void) {
    volatile uint32_t StartUpCounter = 0, HSEStatus = 0;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
    /* Enable HSE */
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);
    /* Wait till HSE is ready and if Time out is reached exit */
    do {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    }
    while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
        HSEStatus = (uint32_t)0x01;
    }
    else {
        HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01) {
        /* Enable Prefetch Buffer */
        FLASH->ACR |= FLASH_ACR_PRFTBE;
        /* Flash 0 wait state */
        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;
        /* HCLK = SYSCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
        /* PCLK2 = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
        /* PCLK1 = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

        /* Configure PLLs ------------------------------------------------------*/

        // 30MHz
        RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
        RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 |
        RCC_CFGR_PLLMULL6);

        RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
        RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
        RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
        RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV8);


        /* Enable PLL2 */
        RCC->CR |= RCC_CR_PLL2ON;
        /* Wait till PLL2 is ready */
        while ((RCC->CR & RCC_CR_PLL2RDY) == 0);
        /* Enable PLL */
        RCC->CR |= RCC_CR_PLLON;
        /* Wait till PLL is ready */
        while ((RCC->CR & RCC_CR_PLLRDY) == 0);
        /* Select PLL as system clock source */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

        /* Wait till PLL is used as system clock source */
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08);
        /* Select System Clock as output of MCO */
        RCC->CFGR &= ~(uint32_t)RCC_CFGR_MCO;
        RCC->CFGR |= (uint32_t)RCC_CFGR_MCO_SYSCLK;
    }
    else { /* If HSE fails to start-up, the application will have wrong clock
    configuration. User can add here some code to deal with this error */
    }
}


void delay(long long t){
	long long i = 0;
	for (i=0; i<t; i++);
}


void RCC_Configuration(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}

void GPIO_Configuration(){
	GPIO_InitTypeDef gpio;

	//LED
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin  = (GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7);
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpio);

	//TIM4 CH3 PB8, CH4 PB9 for PWM 
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin  = (GPIO_Pin_8 | GPIO_Pin_9);
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

	//ADC12_IN10 PC0
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &gpio);

	//ADC12_IN11 PC1
	gpio.GPIO_Pin = GPIO_Pin_1;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &gpio);

	//ADC12_IN12 PC2
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &gpio);

	//ADC12_IN13 PC3
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &gpio);

	//USART1 : Putty
	//usart1 tx
	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
	//usart1 rx
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &gpio);

	//USART2 : Bluetooth
	//USART2 TX
	gpio.GPIO_Pin = (GPIO_Pin_2);
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, & gpio);

	//USART2 RX
	gpio.GPIO_Pin = (GPIO_Pin_3);
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, & gpio);

}

/* 서보모터 관련 */
void TIMx_Configuration(){
	TIM_TimeBaseInitTypeDef TIMx;
	/* 30MHz */
	TIMx.TIM_ClockDivision = TIM_CKD_DIV1;
	TIMx.TIM_CounterMode = TIM_CounterMode_Down;
	TIMx.TIM_Period = 20000 - 1;
	TIMx.TIM_Prescaler = 30 - 1;

	//TIM3
	TIM_TimeBaseInit(TIM3, &TIMx);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	//TIM4
	TIM_TimeBaseInit(TIM4, &TIMx);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

void TIMx_PWM_Configuration(){
	//TIM_OC_CH3
	TIM_OC_CH3.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_CH3.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC_CH3.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_CH3.TIM_Pulse = 1500;

	TIM_OC3Init(TIM4, &TIM_OC_CH3);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	//TIM_OC_CH4
	TIM_OC_CH4.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_CH4.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC_CH4.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_CH4.TIM_Pulse = 1500;

	TIM_OC4Init(TIM4, &TIM_OC_CH4);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
}

void change_PWM_CH3 (int p){
	int pwm_pulse;
	pwm_pulse = p * 2000 / 100;
	TIM_OC_CH3.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_CH3.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC_CH3.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_CH3.TIM_Pulse = pwm_pulse;
	TIM_OC3Init(TIM4, &TIM_OC_CH3);
}

void change_PWM_CH4 (int p){
	int pwm_pulse;
	pwm_pulse = p * 2000 / 100;
	TIM_OC_CH4.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_CH4.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC_CH4.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_CH4.TIM_Pulse = pwm_pulse;
	TIM_OC4Init(TIM4, &TIM_OC_CH4);
}


/* 조도 센서 관련 */
void ADC_Configure(){
	ADC_InitTypeDef adc;
	ADC_DeInit(ADC1);
	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = 4;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_55Cycles5);
	ADC_Init(ADC1, &adc);
	ADC_Cmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	ADC_GetResetCalibrationStatus(ADC1);
	ADC_StartCalibration(ADC1);
	ADC_GetCalibrationStatus(ADC1);
}

void DMA_Configuration(){
	DMA_InitTypeDef dma;
	DMA_DeInit(DMA1_Channel1);
	dma.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
	dma.DMA_MemoryBaseAddr = (uint32_t) ADC_Val;
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_BufferSize = 4;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &dma);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

/* USART 통신 */
void Usart_Set(){
	USART_InitTypeDef usart1; // bluetooth TX, RX
	USART_InitTypeDef usart2;

	usart1.USART_BaudRate = 9600;
	usart1.USART_Mode = (USART_Mode_Tx | USART_Mode_Rx);
	usart1.USART_WordLength = USART_WordLength_8b;
	usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart1.USART_Parity = USART_Parity_No;
	usart1.USART_StopBits = USART_StopBits_1;

	USART_Init(USART1, &usart1);
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	usart2.USART_BaudRate = 9600;
	usart2.USART_Mode = (USART_Mode_Tx | USART_Mode_Rx);
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart2.USART_Parity = USART_Parity_No;
	usart2.USART_StopBits = USART_StopBits_1;

	USART_Init(USART2, &usart2);
	USART_Cmd(USART2, ENABLE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}


void NVIC_Configuration(){
	NVIC_InitTypeDef nvic;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// USART1
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x01;
	nvic.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Init(&nvic);


	// USART2
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&nvic);
}

void USART1_IRQHandler(){ //from putty
	char data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		data = USART_ReceiveData(USART1);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, data);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		USART_SendData(USART2, data);
	}
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

void USART2_IRQHandler(){ //from Bluetooth
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
		char data = USART_ReceiveData(USART2);
		if (data == 'T')
			mode = 0;
		else if ( ('a' <= data && data <= 'a'+3) || data == 's' ){
			mode = 1;
			passive_cntrl_data = data;
		}
		else
			mode = 1;
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData(USART1, data);
	}
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
}


/* 동작 알고리즘 */
void controlMotorByBluetooth(int* angle1, int* angle2){ // Bluetooth Control
	if(passive_cntrl_data == 'a'){ //PWM_1 angle ++
		if(*angle1 < 112)
			change_PWM_CH3(*angle1 += 3);
		else
			passive_cntrl_data = NULL;
		delay(3000000);
	}
	else if (passive_cntrl_data == 'b'){ //PWM_1 angle --
		if(*angle1 > 38)
			change_PWM_CH3(*angle1 -= 3);
		else
			passive_cntrl_data = NULL;
		delay(3000000);
	}
	else if (passive_cntrl_data == 'c'){ //PWM_2 angle ++
		if(*angle2 < 112)
			change_PWM_CH4(*angle2 += 3);
		else
			passive_cntrl_data = NULL;
		delay(3000000);
	}
	else if (passive_cntrl_data == 'd'){ //PWM_2 angle --
		if(*angle2 > 38)
			change_PWM_CH4(*angle2 -= 3);
		else
			passive_cntrl_data = NULL;
		delay(3000000);
	}
	else if (passive_cntrl_data == 's'){
		passive_cntrl_data = NULL;
	}
}

double getADCVal(){
	int i=0;
	double avg=0.0;
	for(i=0; i<4; i++)
		avg += ADC_Val[i];
	return avg/4.0;
}

//최적의 위치 찾아가기  
void findMaxLux(int* angle1, int* angle2){
	int angleAzi=35;
	int angelAlt=35;
	int finalAngleAzi=35;
	int finalAngleAlt=35;

	double avg=0.0;
	double max=0.0;

	for(angleAzi = 35; angleAzi<=115; angleAzi += 3){ // 방위각 35 ~ 115 돌려가면서 조도값
		change_PWM_CH3(angleAzi);
		delay(1500000);
		avg = ADC_Val[0];
		if(max < avg){// 현재의 조도 측정값이 기존의 max값 보다 크다면
			max = avg;
			finalAngleAzi = angleAzi;
		}
	}
	change_PWM_CH3(finalAngleAzi);
	delay(3000000);
	avg = 0.0;
	max = 0.0;
	for(angelAlt = 35; angelAlt<=115; angelAlt += 3){ // 방위각 35 ~ 115 돌려가면서 조도값
		change_PWM_CH4(angelAlt);
		delay(1500000);
		avg = ADC_Val[0];
		if(max < avg){ // 현재의 조도 측정값이 기존의 max값 보다 크다면
			max = avg;
			finalAngleAlt = angelAlt;
		}
	}
	change_PWM_CH4(finalAngleAlt);
	*angle1 = finalAngleAzi;
	*angle2 = finalAngleAlt;
	delay(3000000);
}

int main(void){
	int time = 0;
	int angleAzi = 35;
	int angelAlt = 35;
	SysInit();
	SetSysClock();
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();
	Usart_Set();
	NVIC_Configuration();

	TIMx_Configuration();
	TIMx_PWM_Configuration();

	LCD_Init();
	LCD_Clear(WHITE);

	ADC_Configure();
	DMA_Configuration();
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC);

	mode = 1;
	while(1){

		if (mode == 0){ //자동 모드  
			findMaxLux(&angleAzi, &angelAlt);
			time = 0;
			while (time<30000000 && mode == 0){
				time++;
			}
		}
		else { //수동 모드 
			controlMotorByBluetooth(&angleAzi, &angelAlt);
		}
	}
	return 0;

}

