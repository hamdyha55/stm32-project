/* USART.c */

#include "USART.h"

void USART_begin(uint32_t baudRate) {
    /* Initialization of GPIO and USART settings */
    
    // Enable GPIOA clock
    RCC->AHB1ENR |= 0x00000001;  // RCC_AHB1ENR_GPIOAEN
    
    // Enable USART1 clock
    RCC->APB2ENR |= 0x00000010;  // RCC_APB2ENR_USART1EN
    
    // Configure PA9 (TX) and PA10 (RX) as alternate function
    GPIOA->MODER |= 0x00000800;  // GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1
    GPIOA->AFR[1] |= (7 << ((9 % 8) * 4)) | (7 << ((10 % 8) * 4));
    
    // Configure USART1
    USART1->CR1 &= ~0x00000001;  // USART_CR1_UE (Disable USART1)
    
    USART1->BRR = SystemCoreClock / baudRate;  // Set baud rate
    
    USART1->CR1 &= ~0x00001000;  // USART_CR1_M (8-bit data format)
    USART1->CR2 &= ~0x00003000;  // USART_CR2_STOP (1 stop bit)
    USART1->CR1 &= ~(0x00000200 | 0x00000100);  // USART_CR1_PCE | USART_CR1_PS (No parity)
    
    USART1->CR1 |= 0x00000008 | 0x00000004;  // USART_CR1_TE | USART_CR1_RE (Enable transmitter and receiver)
    USART1->CR1 |= 0x00000001;  // USART_CR1_UE (Enable USART1)
}

int USART_available(void) {
    return (USART1->SR & 0x00000020) ? 1 : 0;  // USART_SR_RXNE
}

uint8_t USART_read(void) {
    while (!(USART1->SR & 0x00000020));  // USART_SR_RXNE (Wait until data is received)
    return (uint8_t)(USART1->DR & 0xFF);
}

void USART_readBytes(uint8_t* buffer, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        while (!USART_available());  // Wait until data is available
        buffer[i] = USART_read();
    }
}

void USART_write(uint8_t data) {
    while (!(USART1->SR & 0x00000080));  // USART_SR_TXE (Wait until the transmit buffer is empty)
    USART1->DR = data;
}

void USART_writeBytes(const uint8_t* buffer, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        USART_write(buffer[i]);
    }
}

void USART_print(const char* str) {
    while (*str) {
        USART_write((uint8_t)(*str++));
    }
}

void USART_println(const char* str) {
    USART_print(str);
    USART_write('\r');
    USART_write('\n');
}
