#include <stdint.h>
#include <stm32f10x.h>
#include <stdbool.h>


void delay_us(uint32_t us){
    __asm volatile(
        "push {r0}\r\n"
        "mov R0, %0\r\n"      //val = (9 * us) for 72Mhz
        "_loop:\r\n"    //approx. 8ticks/iteration
	        "cmp R0, #0\r\n" //1
	        "beq _exit\r\n"      		//1 or 1+P (when condition is True)
	        "sub R0, R0, #1\r\n" 	//1
	        "nop\r\n" 				//1 allignment
	        "b _loop\r\n" 			//1+P (pipeline refill) ~4 cycle
        "_exit:\r\n"
	        "pop {r0}\r\n"
        :: "r"(9*us) // For 72 Mhz
    );
}



/* Interrupt handler */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        if(GPIOC->ODR & GPIO_ODR_ODR13){
            GPIOC->ODR &= ~GPIO_ODR_ODR13;
        } else {
            GPIOC->ODR |= GPIO_ODR_ODR13;
        }
    //Clear Interrupt flag
    TIM2->SR &= ~TIM_SR_UIF;
    }
}

void SPI1_Write(uint8_t data){
    while(!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
}

uint8_t SPI1_Read(void){
    SPI1->DR=0;
    while(!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

void cmd(uint8_t data){
    GPIOA->ODR &= ~GPIO_ODR_ODR12;
    GPIOA->ODR &= ~GPIO_ODR_ODR4;
    delay_us(1000);
    SPI1_Write(data);
    GPIOA->ODR |= GPIO_ODR_ODR4;
}

void dat(uint8_t data){
    GPIOA->ODR |= GPIO_ODR_ODR12;
    GPIOA->ODR &= ~GPIO_ODR_ODR4;
    delay_us(1000);
    SPI1_Write(data);
    GPIOA->ODR |= GPIO_ODR_ODR4;
}

int main(void) {
    // int i = 0;
    // int mask = 8; // 8 = 0b10000 = 0x8 = (1 << 4)
    // i = i | mask; // i |= mask;

    /* IO PORTS Configuration */
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN; // 0b10000=0x10
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); // GPIOC->CRH[23:20]=0000
    GPIOC->CRH |= GPIO_CRH_MODE13_0; // GPIOC->CRH[23:20]=0001

    // SPI IO Configuration
    GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4); // GPIOA->CRL[19:16]=0000
    GPIOA->CRL |= GPIO_CRL_MODE4_0; // GPIOA->CRL[19:16]=0001

    GPIOA->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
    GPIOA->CRH |= GPIO_CRH_MODE11_0; 

    GPIOA->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12);
    GPIOA->CRH |= GPIO_CRH_MODE12_0;

    GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5); // GPIOA->CRL[23:20]=0000
    GPIOA->CRL |= GPIO_CRL_MODE5_0; // GPIOA->CRL[23:20]=0001
    GPIOA->CRL |= GPIO_CRL_CNF5_1;

    GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);
    GPIOA->CRL |= GPIO_CRL_MODE7_0; 
    GPIOA->CRL |= GPIO_CRL_CNF7_1;

    SPI1->CR1 &= ~SPI_CR1_DFF;      //DFF=0
    SPI1->CR1 &= ~SPI_CR1_CRCEN;    //CRCEN=0
    SPI1->CR1 |= SPI_CR1_SSM;       //SSM=1
    SPI1->CR1 |= SPI_CR1_SSI;       //SSI=1
    SPI1->CR1 &= ~SPI_CR1_BR;       //BR[2:0]=100
   //SPI1->CR1 |= SPI_CR1_BR_1;      
    SPI1->CR1 |= SPI_CR1_MSTR;      //MSTR=1
    SPI1->CR1 &= ~SPI_CR1_CPOL;     //CPOL=0
    SPI1->CR1 &= ~SPI_CR1_CPHA;     //CPHA=0
    SPI1->CR1 |= SPI_CR1_SPE;       //SPE=1

    // Display initialization
    GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0
    GPIOA->ODR &= ~GPIO_ODR_ODR11; // RESET=0 - аппаратный сброс
    delay_us(10000); // Wait for the power stabilized
    GPIOA->ODR |= GPIO_ODR_ODR11; // RESET=1
    delay_us(1000); // Wait <1ms
    cmd(0xA2); //LCD Drive set 1/9 bias
    cmd(0xA0); // RAM Address SEG Output normal
    cmd(0xC8); // Common output mode selection
    cmd(0x28 | 0x07); // Power control mode
    cmd(0x20 | 0x05); // Voltage regulator
    cmd(0xA6); // Normal color, A7 = inverse color
    cmd(0xAF); // Display on

    // Port PB0 as Input
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); // GPIOB->CRL[3:0]=0000
    GPIOB->CRL |= GPIO_CRL_CNF0_1; // GPIOB->CRL[3:0]=1000
    GPIOB->ODR |= GPIO_ODR_ODR0; // PB0 Internal pull-up resister

    /* TIM2 Configuration */
    /*RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
    TIM2->PSC = 4000;
    TIM2->ARR = 20000;
    TIM2->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ in NVIC
    TIM2->CR1 |= TIM_CR1_CEN; // Start timer
    while (1) {
        __asm volatile ("nop");
    }*/

    cmd(0xB0);
    cmd(0b00010000);
    cmd(0x00);
    while(1){
        if(GPIOB->IDR & GPIO_IDR_IDR0){
            GPIOC->ODR &= ~GPIO_ODR_ODR13;
        } else {
            GPIOC->ODR |= GPIO_ODR_ODR13;
        }
        dat(0b10101010);
    }

    /*while(1){
        GPIOC->ODR &= ~GPIO_ODR_ODR13;
        delay_us(1000000);
        GPIOC->ODR |= GPIO_ODR_ODR13;
        delay_us(1000000);
    }*/

return 0;
}
