# STM32F031K6 Dual PWM Fan Controller

A two‑channel PWM fan controller based on STM32F031K6. Two analog inputs (potentiometers) are sampled via 12‑bit ADC (scan + DMA) and linearly mapped to two PWM outputs (TIM3 CH1/CH2). Target: quiet, fine‑resolution speed setting.

- MCU: STM32F031K6
- ADC: continuous, DMA, channels 0 and 1
- PWM: TIM3, CH1 + CH2, ~25 kHz
- Project: STM32CubeIDE (generated with CubeMX)

Entry point: [Core/Src/main.c](https://github.com/BerndSeggewiss/STM32-F031K6-Fan-Controler/blob/d7e0ef7b6b31d61a415e364a5e6e1b3612f3f2f0/Core/Src/main.c)

## How it works
- ADC continuously scans ADC_IN0 and ADC_IN1 into a 2‑element DMA buffer.
- In `HAL_ADC_ConvHalfCpltCallback`, both ADC values are mapped to PWM compare values (CCR1/CCR2).
- TIM3 outputs PWM on CH1 and CH2.

Snippet (mapping and callback):
```c
static inline uint16_t map_adc_to_pwm(uint16_t adc, uint16_t adcMax, uint16_t pwmMax)
{
    if (adc >= adcMax) return pwmMax;
    if (adc <= 1) return 0;
    return (uint32_t)adc * pwmMax / adcMax;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    const uint16_t adcMax = 4030; // 100% pot ≙ ADC ~4030
    const uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint16_t adc0 = ADC_ValBuffer[0];
    uint16_t adc1 = ADC_ValBuffer[1];
    uint16_t ccr1 = map_adc_to_pwm(adc0, adcMax, arr);
    uint16_t ccr2 = map_adc_to_pwm(adc1, adcMax, arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr2);
}
```

## Status and scope
- Open‑loop control only: there is no closed‑loop regulation implemented yet (no tachometer feedback or speed control loop).
- Designed for standard 4‑wire PC fans’ PWM control signal.

## Hardware pins (typical)
- ADC_IN0 → PA0 (Pot 1)
- ADC_IN1 → PA1 (Pot 2)
- TIM3_CH1 / TIM3_CH2 → typically PA6 / PA7 (or PB4 / PB5, depending on AF)
Please verify exact pin mapping in the project’s CubeMX .ioc configuration.

## PWM frequency
- System clock ≈ 48 MHz (HSI → PLL)
- TIM3 Prescaler = 0, ARR = 1919 ⇒ fPWM ≈ 48 MHz / (ARR + 1) = 48e6 / 1920 ≈ 25 kHz

## Configuration
- Full‑scale calibration: adjust `adcMax` in the callback to your measured maximum (default 4030).
- Optional inversion:
  ```c
  // ccr1 = arr - ccr1;
  // ccr2 = arr - ccr2;
  ```
- Change PWM frequency: modify TIM3 Prescaler and/or ARR in `MX_TIM3_Init`.

## Build & flash
- Open the project in STM32CubeIDE, build, and flash to the STM32F031K6.
- After reset: turning the pots changes PWM duty on CH1/CH2 accordingly.

## Notes for 4‑wire fans
- The fan’s PWM control input typically expects an open‑drain/OC output with a 5 V pull‑up. Use a level shifter/transistor stage as needed.
- Do not source fan current from MCU pins; use external drivers/transistors for power switching if required.

## License
See the STMicroelectronics license notice in the source headers and the repository’s LICENSE file (if present). If none is provided, the code is supplied “AS IS” as noted in the headers.
