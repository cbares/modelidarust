# Brochage Nucleo-F446RE, à garder en phase avec src/board/f446.rs
from nucleo64 import build

diagram = build(
    "Nucleo-F446RE",
    "MB1136",
    {
        "PD2": [("UART5_RX", "rc"), ("SBUS (inversé)", "rc")],
        "PB8": [("TIM2_CH1", "encoder"), ("Codeur G · A", "encoder")],
        "PB9": [("TIM2_CH2", "encoder"), ("Codeur G · B", "encoder")],
    },
    [
        "SBUS : 100 000 bauds 8E2, inverseur matériel obligatoire (pas de RXINV sur F4).",
        "USART2 (PA2/PA3) est relié au ST-Link (port série USB) : absent de CN10 sans changer SB62/SB63.",
        "Horloge 180 MHz (HSE 8 MHz du ST-Link), PWM moteurs 20 kHz.",
    ],
)
