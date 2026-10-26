# Brochage Nucleo-L476RG, à garder en phase avec src/board/l476.rs
from nucleo64 import build

diagram = build(
    "Nucleo-L476RG",
    "MB1136",
    {
        "PD2": [("UART5_RX", "rc"), ("SBUS direct", "rc")],
        "PB4": [("TIM3_CH1", "encoder"), ("Codeur D · A", "encoder")],
        "PB5": [("TIM3_CH2", "encoder"), ("Codeur D · B", "encoder")],
    },
    [
        "SBUS : 100 000 bauds 8E2, inversion faite par l'USART (invert_rx) : pas d'inverseur.",
        "Codeur droit sur PB4/PB5 (PB8/PB9 sans QEI sur L476).",
        "USART2 (PA2/PA3) est relié au ST-Link (port série USB) : absent de CN10 sans changer SB62/SB63.",
        "Horloge 80 MHz (HSI16 interne), PWM moteurs 20 kHz.",
    ],
)
