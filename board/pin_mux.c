/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v11.0
processor: MIMXRT1176xxxxx
package_id: MIMXRT1176DVMAA
mcu_data: ksdk2_0
processor_version: 11.0.1
board: MIMXRT1170-EVK
pin_labels:
- {pin_num: N3, pin_signal: GPIO_EMC_B2_18, label: USER_GREEN, identifier: SEMC_DQS4;USER_GREEN}
- {pin_num: R15, pin_signal: GPIO_AD_08, label: USER_RED, identifier: USER_RED}
- {pin_num: R17, pin_signal: GPIO_AD_10, label: USER_BLUE, identifier: USER_BLUE}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: M15, peripheral: LPUART1, signal: RXD, pin_signal: GPIO_AD_25, pull_keeper_select: Keeper}
  - {pin_num: L13, peripheral: LPUART1, signal: TXD, pin_signal: GPIO_AD_24, pull_keeper_select: Keeper}
  - {pin_num: N8, peripheral: LPI2C5, signal: SCL, pin_signal: GPIO_LPSR_05, software_input_on: Enable, pull_up_down_config: Pull_Up}
  - {pin_num: N7, peripheral: LPI2C5, signal: SDA, pin_signal: GPIO_LPSR_04, software_input_on: Enable, pull_up_down_config: Pull_Up}
  - {pin_num: M1, peripheral: SAI2, signal: sai_mclk, pin_signal: GPIO_EMC_B2_04}
  - {pin_num: P1, peripheral: SAI2, signal: sai_tx_data, pin_signal: GPIO_EMC_B2_08}
  - {pin_num: R2, peripheral: SAI2, signal: sai_tx_sync, pin_signal: GPIO_EMC_B2_10}
  - {pin_num: N2, peripheral: SAI2, signal: sai_tx_bclk, pin_signal: GPIO_EMC_B2_09}
  - {pin_num: P5, peripheral: MIC, signal: 'mic_bitstream, 00', pin_signal: GPIO_LPSR_09}
  - {pin_num: R5, peripheral: MIC, signal: 'mic_bitstream, 01', pin_signal: GPIO_LPSR_10}
  - {pin_num: U8, peripheral: MIC, signal: CLK, pin_signal: GPIO_LPSR_08}
  - {pin_num: N3, peripheral: GPIO8, signal: 'gpio_io, 28', pin_signal: GPIO_EMC_B2_18, identifier: USER_GREEN, direction: OUTPUT}
  - {pin_num: R15, peripheral: GPIO9, signal: 'gpio_io, 07', pin_signal: GPIO_AD_08, direction: OUTPUT}
  - {pin_num: R17, peripheral: GPIO9, signal: 'gpio_io, 09', pin_signal: GPIO_AD_10, direction: OUTPUT}
  - {pin_num: T8, peripheral: GPIO13, signal: 'gpio_io, 00', pin_signal: WAKEUP}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */
  CLOCK_EnableClock(kCLOCK_Iomuxc_Lpsr);      /* LPCG on: LPCG is ON. */

  /* GPIO configuration of USER_GREEN on GPIO_EMC_B2_18 (pin N3) */
  gpio_pin_config_t USER_GREEN_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_EMC_B2_18 (pin N3) */
  GPIO_PinInit(GPIO8, 28U, &USER_GREEN_config);

  /* GPIO configuration of USER_RED on GPIO_AD_08 (pin R15) */
  gpio_pin_config_t USER_RED_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_08 (pin R15) */
  GPIO_PinInit(GPIO9, 7U, &USER_RED_config);

  /* GPIO configuration of USER_BLUE on GPIO_AD_10 (pin R17) */
  gpio_pin_config_t USER_BLUE_config = {
      .direction = kGPIO_DigitalOutput,
      .outputLogic = 0U,
      .interruptMode = kGPIO_NoIntmode
  };
  /* Initialize GPIO functionality on GPIO_AD_10 (pin R17) */
  GPIO_PinInit(GPIO9, 9U, &USER_BLUE_config);

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_08_GPIO9_IO07,           /* GPIO_AD_08 is configured as GPIO9_IO07 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_10_GPIO9_IO09,           /* GPIO_AD_10 is configured as GPIO9_IO09 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,          /* GPIO_AD_24 is configured as LPUART1_TXD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,          /* GPIO_AD_25 is configured as LPUART1_RXD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_B2_04_SAI2_MCLK,        /* GPIO_EMC_B2_04 is configured as SAI2_MCLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_B2_08_SAI2_TX_DATA,     /* GPIO_EMC_B2_08 is configured as SAI2_TX_DATA */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_B2_09_SAI2_TX_BCLK,     /* GPIO_EMC_B2_09 is configured as SAI2_TX_BCLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_B2_10_SAI2_TX_SYNC,     /* GPIO_EMC_B2_10 is configured as SAI2_TX_SYNC */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_EMC_B2_18_GPIO8_IO28,       /* GPIO_EMC_B2_18 is configured as GPIO8_IO28 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_GPR->GPR1 = ((IOMUXC_GPR->GPR1 &
    (~(IOMUXC_GPR_GPR1_SAI2_MCLK_DIR_MASK)))  /* Mask bits to zero which are setting */
      | IOMUXC_GPR_GPR1_SAI2_MCLK_DIR(0x01U)  /* SAI2_MCLK signal direction control: 0x01U */
    );
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,         /* GPIO_LPSR_04 is configured as LPI2C5_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_04 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_05 is configured as LPI2C5_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_05 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_08_MIC_CLK,            /* GPIO_LPSR_08 is configured as MIC_CLK */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_09_MIC_BITSTREAM0,     /* GPIO_LPSR_09 is configured as MIC_BITSTREAM0 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_10_MIC_BITSTREAM1,     /* GPIO_LPSR_10 is configured as MIC_BITSTREAM1 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_WAKEUP_DIG_GPIO13_IO00,          /* WAKEUP_DIG is configured as GPIO13_IO00 */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,          /* GPIO_AD_24 PAD functional properties : */
      0x02U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,          /* GPIO_AD_25 PAD functional properties : */
      0x02U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high drive strength
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,         /* GPIO_LPSR_04 PAD functional properties : */
      0x0AU);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain LPSR Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_05 PAD functional properties : */
      0x0AU);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull up
                                                 Open Drain LPSR Field: Disabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/