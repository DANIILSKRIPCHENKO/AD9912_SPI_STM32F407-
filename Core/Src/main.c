#include "stm32f407xx.h"
#include "AD9912.h"
#include "math.h"

void SysClockConfig (void)
{

	#define PLL_M 8
	#define PLL_N 336
	#define PLL_P 0 // = PLL_P 2


	RCC->CR |= RCC_CR_HSEON; // enable external high speed oscillator and wait to ready
	while (!(RCC->CR & RCC_CR_HSERDY));
	RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Power interface clock enable
	PWR->CR |= PWR_CR_VOS;  // Regulator voltage  reset value
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS; // configure flash prefetch and the latency related settings
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 prescalar
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;// APB2 prescalar

	// config the main pll
	RCC->PLLCFGR = (PLL_M<<0) // PLL_M
			| (PLL_N<<6)   //PLL_N
			| (PLL_P<<16)    // PLL_P
			| (RCC_PLLCFGR_PLLSRC_HSE);

	RCC->CR |= RCC_CR_PLLON; // enable pll
	while (!(RCC->CR & RCC_CR_PLLRDY)); // wait for it ready

	RCC->CFGR |= RCC_CFGR_SW_PLL; // select clock source
	while ((RCC -> CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // wait for it to be set
}

void Delay_us (uint16_t us)
{
	TIM6->CNT = 0; // Reset the counter
	while (TIM6->CNT < us); // Wait the counter to reach entered value. Each count is 1us delay
}

void Delay_ms (uint16_t ms)
{
	for  (uint16_t i = 0; i<ms; i++)
	{
		Delay_us(1000); // delay of 1ms;
	}
}

void TIMconfig (void)
{
// TIM 6 config
	RCC->APB1ENR |= (1<<4); // enable timer6 clock
	TIM6->PSC = 84-1; // 84MHz/(83+1)=1MHz = 1uS delay
	TIM6->ARR = 0xffff; // MAX ARR value
	TIM6->CR1 |= (1<<0); // enable the counter and wait for the update flag to set
	while (!(TIM6->SR & (1<<0))); // UIF: This bit is set by hardware when the registers are updated
}


void SPICongig(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |= SPI_CR1_CPOL;
	SPI1->CR1 |= SPI_CR1_CPHA;
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR1 |= SPI_CR1_BR_2;
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // msb first
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_SSM;
	SPI1->CR1 &= ~SPI_CR1_RXONLY; // full duplex
	SPI1->CR1 &= ~SPI_CR1_DFF; // 8bit format

	SPI1->CR2 = 0;
}

void GPIOConfig(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock


	GPIOA->MODER |= GPIO_MODER_MODE7_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE7_0;

	GPIOA->MODER |= GPIO_MODER_MODE6_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE6_0;

	GPIOA->MODER |= GPIO_MODER_MODE5_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE5_0;

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_0;

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_0;

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR5_0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_3;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL6_3;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL6_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_0;

	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_3;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_0;


	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	GPIOE->MODER &= ~GPIO_MODER_MODE3_1;
	GPIOE->MODER |= GPIO_MODER_MODE3_0;

	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_1;
	GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_0;


	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	GPIOB->MODER &= ~GPIO_MODER_MODE11_1;
	GPIOB->MODER |= GPIO_MODER_MODE11_0;

	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_0;

	GPIOB->BSRR |= GPIO_BSRR_BR11;

}

void SPI_Enable (void)
{
	SPI1->CR1 |= SPI_CR1_SPE;
}

void SPI_Disable (void)
{
	SPI1->CR1 &= ~SPI_CR1_SPE;
}

void CS_Enable(void)
{
	GPIOE->BSRR |= GPIO_BSRR_BR3;
}

void CS_Disable(void)
{
	GPIOE->BSRR |= GPIO_BSRR_BS3;
}


void SPI_Transmit(uint8_t *data, int size)
{
	int i = 0;
	while (i<size)
	{
		while(!((SPI1->SR) & (1<<1))) {};
		SPI1->DR = data[i];
		i++;
	}

	while(!(SPI1->SR & SPI_SR_TXE)) {}; // wait for TXE bit to set -> This will indicate that the buffer is empty
	while(((SPI1->SR) & (1<<7))) {}; // wait for BSY bit to reset -> This will indicate that SPI is not busy in communication

	// Clearing the Overrun flg by reading DR and SR
	uint8_t temp = SPI1->DR;
	temp = SPI1->SR;

}

void SPI_Receive (uint8_t *data, int size)
{
	while (size)
	{
		while(((SPI1->SR) & (1<<7))) {}; // wait for BSY bit to reset -> This will indicate that SPI is not busy in communication
		SPI1->DR = 0;
		while(!(SPI1->SR & SPI_SR_TXE)) {}; //  Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
		Delay_us(500);
		*data++ = (SPI1->DR);
		size--;
	}
}

uint8_t RxData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Read
void LIS3DSH_Read (uint8_t *data, uint8_t addr, uint8_t NumberOfBytes )
{
	addr |= 0x80; //read operation

	if (NumberOfBytes>1)
	{
		addr|= 0x40; // multibyte read
	}

	CS_Enable();
	SPI_Transmit(&addr, 1);
	SPI_Receive (data, NumberOfBytes);
	CS_Disable();
}


void AD9912_Write (uint8_t *data, uint16_t addr, uint8_t NumberOfBytes)
{
	uint8_t w1 = 0;
	uint8_t w0 = 0;
	uint8_t ReadWrite = 0;
	uint16_t InsructionHeader = 0;
	uint8_t InsructionHeader_Byte1 = 0;
	uint8_t InsructionHeader_Byte2 = 0;

	switch (NumberOfBytes)
	{
		case 1:
			w1 = 0;
			w0 = 0;
			break;
		case 2:
			w1 = 0;
			w0 = 1;
			break;
		case 3:
			w1 = 1;
			w0 = 0;
			break;
	}

	InsructionHeader = InsructionHeader|(ReadWrite<<15)|(w1<<14)|(w0<<13)|addr;

	InsructionHeader_Byte2 = InsructionHeader&0xff;
	InsructionHeader_Byte1 = InsructionHeader>>8;


	CS_Enable();
	SPI_Transmit(&InsructionHeader_Byte1, 1);
	SPI_Transmit(&InsructionHeader_Byte2, 1);
	SPI_Transmit(data, NumberOfBytes);
	CS_Disable();
}

void AD9912_Read (uint8_t *data, uint8_t addr, uint8_t NumberOfBytes )
{
	uint8_t w1 = 0;
	uint8_t w0 = 0;
	uint8_t ReadWrite = 1;
	uint16_t InsructionHeader = 0;
	uint8_t InsructionHeader_Byte1 = 0;
	uint8_t InsructionHeader_Byte2 = 0;

	switch (NumberOfBytes)
			{
				case 1:
					w1 = 0;
					w0 = 0;
					break;
				case 2:
					w1 = 0;
					w0 = 1;
					break;
				case 3:
					w1 = 1;
					w0 = 0;
					break;
			}


	InsructionHeader = InsructionHeader|(ReadWrite<<15)|(w1<<14)|(w0<<13)|addr;
	InsructionHeader_Byte1 = (InsructionHeader>>8);
	InsructionHeader_Byte2 = InsructionHeader&0xff;

	CS_Enable();
	SPI_Transmit(&InsructionHeader_Byte1, 1);
	SPI_Transmit(&InsructionHeader_Byte2, 1);
	SPI_Receive (data, NumberOfBytes);
	CS_Disable();
}


AD9912_ID ID;

void AD9912_Read_ID (AD9912_ID *ID )
{
	uint8_t buffer[2];
	AD9912_Read((uint8_t*)&buffer[0], AD9912_PART_ID_1_ADDR, 1);
	AD9912_Read((uint8_t*)&buffer[1], AD9912_PART_ID_2_ADDR, 1);

	ID->Byte1 = buffer[0];
	ID->Byte2 = buffer[1];

}

/*
void AD9912_IOUpdate()
{

	AD9912_Write(&AD9912_IO_UPDATE_VALUE, AD9912_SERIAL_OPTIONS_2_ADDR, 1);
}
*/

unsigned long long AD9912_FTW_Culc(long double frequency, long double sampling_clock)
{
	unsigned long long FTW = 0;
	FTW = round(281474976710656*(frequency/sampling_clock));
	return FTW;
}

void AD9912_Set_Freq(long double frequency)
{
	unsigned long long FTW = 0;
	unsigned long long FTW1 = 0;
	uint8_t buffer[6];
	FTW = AD9912_FTW_Culc(frequency, 1000000000);

	buffer[0] = FTW>>40;
	buffer[1] = FTW>>32;
	buffer[2] = FTW>>24;
	buffer[3] = FTW>>16;
	buffer[4] = FTW>>8;
	buffer[5] = FTW;



	AD9912_Write(&buffer[0], AD9912_FTW0_5_ADDR, 1);
	AD9912_Write(&buffer[1], AD9912_FTW0_4_ADDR, 1);
	AD9912_Write(&buffer[2], AD9912_FTW0_3_ADDR, 1);
	AD9912_Write(&buffer[3], AD9912_FTW0_2_ADDR, 1);
	AD9912_Write(&buffer[4], AD9912_FTW0_1_ADDR, 1);
	AD9912_Write(&buffer[5], AD9912_FTW0_0_ADDR, 1);


}

void IOUpdate()
{
	GPIOB->BSRR |= GPIO_BSRR_BS11;
	Delay_us(1);
	GPIOB->BSRR |= GPIO_BSRR_BR11;

}

uint8_t data[3] = {0b10101010, 0b10101010, 0b10101010};

uint8_t DATA = 0b10101010;
uint8_t WhoIAm = 0;

uint32_t freqToSend = 19440000;

int main()
	{
	SysClockConfig();
	GPIOConfig();
	TIMconfig();
	SPICongig();
	CS_Disable();
	SPI_Enable();

	uint8_t ZERO = 0b11111111;
	uint8_t three = 0b00000011;
	while (1)
	{

		//Delay_ms(2);
		//Delay_us(20);

		AD9912_Set_Freq(freqToSend);
		IOUpdate();
		Delay_us(100);

		AD9912_Write(&ZERO, AD9912_DAC_current_1_ADDR, 1);
		AD9912_Write(&three, AD9912_DAC_current_2_ADDR, 1);
		IOUpdate();

		Delay_us(100);

		//AD9912_Read_ID(&ID);

	}

}
