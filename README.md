# MK66

Documentation: https://is.muni.cz/auth/el/fi/podzim2024/PV198/um/FRDM-K66F-reference-manual.pdf

## GPIO

```
GPIO_PortToggle(base, pin)
GPIO_PinRead(base, pin)
GPIO_PinRead(base, pin)

# led green
GPIO_PortToggle(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN)
```

## PWM

Pulse-width modulation 
FTM is an enhanced version of the Timer/PWM module (TPM)

```
FTM_UpdatePwmDutycycle(FTM2_PERIPHERAL, kFTM_Chnl_1, kFTM_EdgeAlignedPwm, percentage);
FTM_SetSoftwareTrigger(FTM2_PERIPHERAL,true);
```

## ADC

Analog-to-Digital Converter

```
volatile int xy = 0;

ADC16_SetChannelConfig(ADC1_BASE, 0, &ADC1_channelsConfig[0]);

void ADC1_IRQHANDLER(void) {
...
	uint32_t result_values[2] = { 0 };
	for (int i = 0; i < 2; i++) {
		uint32_t status = ADC16_GetChannelStatusFlags(ADC1_PERIPHERAL, i);
		if (status == kADC16_ChannelConversionDoneFlag) {
			result_values[i] = ADC16_GetChannelConversionValue(ADC1_PERIPHERAL,
					i);
		}
	}

	xy = result_values[0];
...
}

```

## SPI

- Pressure Sensor BMP280
- Documentation: https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
```
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
	dspi_transfer_t masterXfer;
	masterTxBuffer[0] = reg_addr | 0x80;

	memcpy(masterTxBuffer + 1, reg_data, length);

	masterXfer.txData = masterTxBuffer;
	masterXfer.rxData = masterRxBuffer;
	masterXfer.dataSize = length + 1;
	masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcsContinuous; // TODO more flags can be added by ORing them together (using | operator)

	status_t status = DSPI_MasterTransferBlocking(SPI0_BASE, &masterXfer); // Use appropriate DSPI base address

	memcpy(reg_data, masterRxBuffer + 1, length);
	return status == kStatus_Success  ? 0:1;
}


/* raw read*/
rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

/* Getting the 32 bit compensated temperature */
rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);

/* Getting the compensated temperature as floating point value */
rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);

/* Getting the compensated pressure as floating point value */
rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
```
### I2C

- FXOS8700CQ accelerometer
- Documentation: https://www.nxp.com/docs/en/data-sheet/FXOS8700CQ.pdf

```
#define SENSOR_ADDRESS 0x1DU
#define DEFAULT_DATA_SCALE 2U
#define BUFFER_SIZE 6

int8_t buffer[6] = {0};

BOARD_Accel_I2C_Receive(SENSOR_ADDRESS, OUT_X_MSB_REG, 1, buffer, 6);

```


## Misc

### Dirty sleep

```
void delay(void) {
	volatile uint32_t i = 0;
	for (i = 0; i < 2800000; ++i) {
		__asm("NOP");
	}
}
```

### PIT

```
void PIT_CHANNEL_0_IRQHANDLER(void) {
	uint32_t intStatus;
	intStatus = PIT_GetStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0);
	PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, intStatus);
	PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
	
/* Place your code here */

#if defined __CORTEX_M && (__CORTEX_M == 4U)
	__DSB();
#endif
}
PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
```

### HSVtoRGB

```
/*
 * HSV_RGB.hpp
 *
 *  Created on: Jan 25, 2021
 *      Author: danie
 */

#ifndef HSV_RGB_H_
#define HSV_RGB_H_
#include <math.h>
#include <stdint.h>

typedef struct RgbColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} RgbColor;

RgbColor HSVtoRGB(int H, int S, int V) {

  float s = S / 100.;
  float v = V / 100.;
  float C = s * v;
  float X = C * (1 - fabs(fmod(H / 60.0, 2) - 1));
  float m = v - C;
  float r, g, b;
  if (H >= 0 && H < 60) {
    r = C, g = X, b = 0;
  } else if (H >= 60 && H < 120) {
    r = X, g = C, b = 0;
  } else if (H >= 120 && H < 180) {
    r = 0, g = C, b = X;
  } else if (H >= 180 && H < 240) {
    r = 0, g = X, b = C;
  } else if (H >= 240 && H < 300) {
    r = X, g = 0, b = C;
  } else {
    r = C, g = 0, b = X;
  }
  RgbColor ret;
  ret.r = (r + m) * 255;
  ret.g = (g + m) * 255;
  ret.b = (b + m) * 255;
  return ret;
}

#endif /* HSV_RGB_H_ */
```
