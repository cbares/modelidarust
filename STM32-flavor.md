# Listes des cartes nucleo compatibles

## Nucleo64-F446RE

- <https://www.st.com/en/evaluation-tools/nucleo-f446re.html>
- Flash : 512 KB
- RAM : 128 KB
- MCU : STM32F446RE
- Core : ARM Cortex-M4F
- Freq : 180 MHz
- 10 General Purpose timers
- 2 Advanced Control timers
- 2 Basic timers
- 4 USARTS
- 2 UARTS
- 3 I2C
- 4 SPI
- 1 QUAD SPI
- 2 CAN
- 1 CRC
- 3 ADC 12 bits / 16 canaux
- 1 DAC 12 bits / 2 canaux

TIM1: PWM, complementary outputs, dead time, break input, No Pull, low speed, AF1
    PA7     ------> TIM1_CH1N
    PB1     ------> TIM1_CH3N
    PA9     ------> TIM1_CH2
    PA11     ------> TIM1_CH4
Version alternative avec 4 canaux PWM:
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    PA11     ------> TIM1_CH4

TIM2: left encoder, TIM_ENCODERMODE_TI12, No Pull,low speed, AF1
    PB8     ------> TIM2_CH1
    PB9     ------> TIM2_CH2

TIM5: right encoder, TIM_ENCODERMODE_TI12, No Pull, low speed, AF2
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2

TIM6: timer de lecture des encorders

USART2: 115200 bauds, 8N1, 16 oversampling, No Pull, Very high speed, AF7
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX

USART3: 115200 bauds, 8N1, 16 oversampling, No Pull, Very high speed, AF7
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX


## Nucleo64-F411RE

- <https://www.st.com/en/evaluation-tools/nucleo-f411re.html>
- Flash : 512 KB
- RAM : 128 KB
- MCU : STM32F411RE
- Core : ARM Cortex-M4F
- Freq : 100 MHz
- 8 General Purpose timers
- 3 USARTS/UARTS
- 3 I2C
- 5 SPI
- 1 CRC
- 1 ADC 12 bits / 16 canaux

TIM3, Ch1 for PA6, PB4, PC6
TIM3, Ch2 for **PA7**, PB5, PC7
TIM3, Ch3 for PB0, PC8
TIM3, Ch4 for **PB1**, PC9

TIM1, Ch1 for PA8
TIM1, Ch1N for **PA7**, PB13
TIM1, Ch2 for **PA9**
TIM1, Ch2N for PB0, PB14
TIM1, Ch3 for PA10
TIM1, Ch3N for **PB1**, PB15
TIM1, Ch4 for **PA11**


## Nucleo64-F410RB

- <https://www.st.com/en/evaluation-tools/nucleo-f410rb.html>
- Flash : 128 KB
- RAM : 32 KB
- MCU : STM32F410RB
- Core : ARM Cortex-M4F
- Freq : 100 MHz
- 4 General Purpose timers
- 1 Low Power timer
- 1 Advanced Control timer
- 3 USARTS
- 3 I2C
- 3 SPI
- 1 ADC 12 bits / 16 canaux
- 1 DAC 12 bits / 1 canaux

## Commande moteurs:

```text
#1000,1000
#0990,0990
#0900,0900
#0800,0800
#0750,0750
#0700,0700
#0600,0600
#0500,0500
#0450,0450
#0400,0400
#0300,0300
#0200,0200
#0100,0100
#0000,0000
```

#1200,1200