# Brochage Nucleo-F411RE, à garder en phase avec src/board/f411.rs
from nucleo64 import build

diagram = build(
    "Nucleo-F411RE",
    "MB1136",
    {
        "PC7": [("USART6_RX", "rc"), ("SBUS (inversé)", "rc")],
        "PB4": [("TIM3_CH1", "encoder"), ("Codeur D · A", "encoder")],
        "PB5": [("TIM3_CH2", "encoder"), ("Codeur D · B", "encoder")],
    },
    [
        "SBUS : 100 000 bauds 8E2, inverseur matériel obligatoire (pas de RXINV sur F4).",
        "Pas d'UART5 sur F411 : SBUS sur USART6 (PC7). Codeur droit sur PB4/PB5 (PB8/PB9 sans QEI).",
        "USART2 (PA2/PA3) est relié au ST-Link (port série USB) : absent de CN10 sans changer SB62/SB63.",
        "Horloge 100 MHz (HSE 8 MHz du ST-Link), PWM moteurs 20 kHz.",
    ],
)
