#include "main.h"

#define DMA2_BASE 0x40026400
#define DMA_SM0AR 0x1C + 0x18 * 2
#define DMA_SPAR 0x18 + 0x18 * 2
#define DMA_SNDTR 0x14 + 0x18 * 2
#define DMA_SCR 0x10 + 0x18 * 2
#define DMA_LIFCR 0x08

#define UART1_BASE 0x40011000
#define UART_BRR 0x08
#define UART_CR1 0x0C
#define UART_SR 0x00
#define UART_DR 0x04
#define UART_CR3 0x14

#define GPIOB_BASE 0x40020400
#define GPIO_AFRL 0x20
#define GPIO_MODER 0x00

#define RCC_BASE 0x40023800
#define RCC_AHB1ENR 0x30
#define RCC_APB2ENR 0x44

#define FLASH_BASE 0x40023C00
#define FLASH_CR 0x10
#define FLASH_SR 0x0C
#define FLASH_KEYR 0x04

int firmware_updated;
char rxbuff[30000];

void DMA_Init();
void UART1_Init();
void UART_Write(uint8_t data);
char UART_Read();

__attribute__((section(".RamFunc"))) void FLASH_Erase(uint8_t sector);
__attribute__((section(".RamFunc"))) void FLASH_Write(uint8_t* addr, uint8_t* buff, int size);
__attribute__((section(".RamFunc"))) void update();

int main() {
	UART1_Init();
	while (1) {
		if (firmware_updated == 1) {
			__asm("cpsid i");
			update();
		}
	}
	return 0;
}


void DMA_Init() {
	uint32_t* DMA2_EN = (uint32_t*)(RCC_BASE + RCC_AHB1ENR); //Clock enable
	*DMA2_EN |= (0b01 << 22);
	uint32_t* DMA2_SM0AR = (uint32_t*)(DMA2_BASE + DMA_SM0AR);
	*DMA2_SM0AR = rxbuff;
	uint32_t* DMA2_SPAR = (uint32_t*)(DMA2_BASE + DMA_SPAR);
	*DMA2_SPAR = 0x40011004; //chắc cái địa chỉ này đúng chưa?
	uint32_t* DMA2_SNDTR = (uint32_t*)(DMA2_BASE + DMA_SNDTR);
	*DMA2_SNDTR = sizeof(rxbuff);
	uint32_t* DMA2_SCR = (uint32_t*)(DMA2_BASE + DMA_SCR);
	*DMA2_SCR &= ~(0b111 << 25);
	*DMA2_SCR |= (0b100 << 25);
	*DMA2_SCR |= (0b01 << 0) | (0b01 << 10) | (0b01 << 8) | (0b01 << 4);
}

void DMA2_Stream2_IRQHandler() {
	uint32_t* DMA2_LIFCR = (uint32_t*)(DMA2_BASE + DMA_LIFCR);
	*DMA2_LIFCR |= (0b01 << 21);
	firmware_updated = 1;
}

void UART1_Init() {
	uint32_t* GPIOB_EN = (uint32_t*)(RCC_BASE + RCC_AHB1ENR);
	*GPIOB_EN |= (0b01 << 1);
	uint32_t* UART1_EN = (uint32_t*)(RCC_BASE + RCC_APB2ENR);
	*UART1_EN |= (0b01 << 4);

	//PB6 TX and PB7 RX
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE + GPIO_MODER);
	*GPIOB_MODER &= ~(0x0F << 12);
	*GPIOB_MODER |= (0x0A << 12);

	uint32_t* GPIOB_AFRL = (uint32_t*)(GPIOB_BASE + GPIO_AFRL);
	*GPIOB_AFRL &= ~(0xFF << 24);
	*GPIOB_AFRL |= (0x77 << 24);

	uint32_t* UART1_BRR = (uint32_t*)(UART1_BASE + UART_BRR);
	*UART1_BRR = (104 << 4) | (3 << 0);
	uint32_t* UART1_CR1 = (uint32_t*)(UART1_BASE + UART_CR1);
	*UART1_CR1 |= (0b01 << 13) | (0b01 << 12) | (0b01 << 10) | (0b01 << 3) | (0b01 << 2);

#if 0
	*UART1_CR1 |= (0b01 << 5);
	uint32_t* NVIC_ISER1 = (uint32_t*)0xE000E104;
	*NVIC_ISER1 |= (0b01 << 5);
#else
	uint32_t* UART1_CR3 = (uint32_t*)(UART1_BASE + UART_CR3);
	*UART1_CR3 |= (0b01 << 6);
	uint32_t* NVIC_ISER1 = (uint32_t*)0xE000E104;
	*NVIC_ISER1 |= (0b01 << 26); //Enable DMA2 Stream 2 interrupt
	DMA_Init();
#endif
}

void UART_Write(uint8_t data) {
	uint32_t* UART1_DR = (uint32_t*)(UART1_BASE + UART_DR);
	uint32_t* UART1_SR = (uint32_t*)(UART1_BASE + UART_SR);
	while (((*UART1_SR >> 6) & 1) == 0);
	*UART1_DR = data;
	while (((*UART1_SR >> 7) & 1) == 0);
}

char UART_Read() {
	uint32_t* UART1_DR = (uint32_t*)(UART1_BASE + UART_DR);
	uint32_t* UART1_SR = (uint32_t*)(UART1_BASE + UART_SR);
	while (((*UART1_SR >> 5) & 1) == 0);
	char data = *UART1_DR;
	return data;
}

__attribute__((section(".RamFunc"))) void FLASH_Erase(uint8_t sector) {
	uint32_t* flash_CR = (uint32_t*)(FLASH_BASE + FLASH_CR);
	uint32_t* flash_SR = (uint32_t*)(FLASH_BASE + FLASH_SR);
	uint32_t* flash_Keyr = (uint32_t*)(FLASH_BASE + FLASH_KEYR);
	if (((*flash_SR >> 16) & 1) == 0) {
		if (((*flash_CR >> 31) & 1) == 1) {
			*flash_Keyr = 0x45670123;
			*flash_Keyr = 0xCDEF89AB;
		}
		*flash_CR |= (0b01 << 1) | (0b01 << 16) | (sector << 3);
		while (((*flash_SR >> 16) & 1) == 1);
	}
}

__attribute__((section(".RamFunc"))) void FLASH_Write(uint8_t* addr, uint8_t* buff, int size) {
	uint32_t* flash_CR = (uint32_t*)(FLASH_BASE + FLASH_CR);
	uint32_t* flash_SR = (uint32_t*)(FLASH_BASE + FLASH_SR);
	uint32_t* flash_Keyr = (uint32_t*)(FLASH_BASE + FLASH_KEYR);
	if (((*flash_SR >> 16) & 1) == 0) {
		if (((*flash_CR >> 31) & 1) == 1) {
			*flash_Keyr = 0x45670123;
			*flash_Keyr = 0xCDEF89AB;
		}
		*flash_CR |= (0b01 << 0);
		*flash_CR &= ~(0b11 << 8);
		for (int i = 0; i < size; i++) {
			addr[i] = buff[i];
		}
		while (((*flash_SR >> 16) & 1) == 1);
	}
}

__attribute__((section(".RamFunc"))) void update() {
	FLASH_Erase(0);
	FLASH_Write(0x08000000, rxbuff, sizeof(rxbuff));
}
