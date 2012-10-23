#include <mach/pinmux.h>
#include "board-acer-t30.h"
#include <mach/gpio.h>

static __initdata struct tegra_pingroup_config picasso_E2_pinmux_common[] = {
	DEFAULT_PINMUX(CLK1_OUT,        EXTPERIPH1,      NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(DAP1_DIN,        I2S0,            NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            PULL_DOWN,     TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(DAP1_FS,         I2S0,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(DAP1_SCLK,       I2S0,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(DAP2_DIN,        I2S1,            NORMAL,        NORMAL,     INPUT), ////// check config state
	DEFAULT_PINMUX(DAP2_DOUT,       I2S1,            NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(DAP2_FS,         I2S1,            NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(DAP2_SCLK,       I2S1,            NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(SPDIF_IN,        SPDIF,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SPDIF_OUT,       SPDIF,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SPI1_CS0_N,      SPI1,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SPI1_MISO,       SPI1,            PULL_DOWN,     TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(SPI1_MOSI,       SPI1,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SPI1_SCK,        SPI1,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SPI2_CS0_N,      SPI2,            NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(SPI2_CS1_N,      SPI2,            PULL_UP,       TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(SPI2_CS2_N,      SPI2,            PULL_DOWN,     TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(SPI2_MISO,       SPI2,            NORMAL,        TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(SPI2_MOSI,       SPI2,            NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(SPI2_SCK,        SPI2,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(DAP3_DIN,        I2S2,            NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(DAP3_DOUT,       RSVD1,           NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(DAP3_FS,         RSVD1,           NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(DAP3_SCLK,       I2S2,            PULL_DOWN,     NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GPIO_PV0,        RSVD,            NORMAL,        TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(GPIO_PV1,        RSVD,            NORMAL,        TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(ULPI_CLK,        UARTD,           NORMAL,        NORMAL,     OUTPUT), // Function o
	DEFAULT_PINMUX(ULPI_DATA0,      UARTA,           NORMAL,        NORMAL,     OUTPUT), // Function o
	DEFAULT_PINMUX(ULPI_DATA1,      UARTA,           NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(ULPI_DATA2,      UARTA,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(ULPI_DATA3,      UARTA,           NORMAL,        TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(ULPI_DATA4,      UARTA,           PULL_UP,       NORMAL,     INPUT), // GI
	DEFAULT_PINMUX(ULPI_DATA5,      UARTA,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(ULPI_DATA6,      UARTA,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(ULPI_DATA7,      UARTA,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(ULPI_DIR,        UARTD,           NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(ULPI_NXT,        UARTD,           NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(ULPI_STP,        UARTD,           NORMAL,        NORMAL,     OUTPUT), //G

	/* I2C3 pinmux */
	I2C_PINMUX(CAM_I2C_SCL,		I2C3,		 NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),
	I2C_PINMUX(CAM_I2C_SDA,		I2C3,		 NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),

	DEFAULT_PINMUX(CAM_MCLK,        VI_ALT2,         NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(GPIO_PBB0,       RSVD1,           NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GPIO_PBB3,       VGP3,            NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(GPIO_PBB4,       VGP4,            NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(GPIO_PBB5,       VGP5,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GPIO_PBB6,       VGP6,            NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(GPIO_PBB7,       I2S4,            PULL_UP,       NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(GPIO_PCC1,       RSVD1,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(GPIO_PCC2,       I2S4,            NORMAL,        NORMAL,     OUTPUT), // G (NC)

	/* I2C2 pinmux */
	I2C_PINMUX(GEN2_I2C_SCL,	I2C2,		 NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),
	I2C_PINMUX(GEN2_I2C_SDA,	I2C2,		 NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),

	DEFAULT_PINMUX(GMI_A16,         SPI4,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_A17,         SPI4,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_A18,         SPI4,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_A19,         RSVD3,           NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_AD0,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD1,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD2,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD3,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD4,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD5,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD6,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD7,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            NORMAL,        NORMAL,     OUTPUT), // Function o
	DEFAULT_PINMUX(GMI_AD9,         NAND,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_AD10,        NAND,            PULL_DOWN,     NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_AD11,        NAND,            NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_AD12,        NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD13,        NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD14,        NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_AD15,        NAND,            NORMAL,        NORMAL,     INPUT), //G
	DEFAULT_PINMUX(GMI_ADV_N,       NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_CLK,         NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_CS0_N,       NAND,            PULL_UP,       TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_CS1_N,       NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_CS2_N,       RSVD1,           NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_CS3_N,       NAND,            NORMAL,        TRISTATE,   INPUT), //BOARD_ID1
	DEFAULT_PINMUX(GMI_CS4_N,       NAND,            NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(GMI_CS6_N,       NAND_ALT,        NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_CS7_N,       NAND_ALT,        NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(GMI_DQS,         RSVD1,           NORMAL,        NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_IORDY,       RSVD1,           PULL_UP,       TRISTATE,   INPUT), //SD_DET#
	DEFAULT_PINMUX(GMI_OE_N,        NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(GMI_RST_N,       RSVD3,           NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(GMI_WAIT,        NAND,            PULL_DOWN,     NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(GMI_WP_N,        RSVD1,           PULL_UP,       NORMAL,     INPUT), //GI
	DEFAULT_PINMUX(GMI_WR_N,        NAND,            NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(CRT_HSYNC,       CRT,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(CRT_VSYNC,       CRT,             NORMAL,        NORMAL,     OUTPUT), // G (NC)

	/* I2C4 pinmux */
	DEFAULT_PINMUX(DDC_SCL,		I2C4,		 NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(DDC_SDA,		I2C4,		 NORMAL,        NORMAL,     OUTPUT), //G (NC)

	DEFAULT_PINMUX(HDMI_INT,        RSVD0,           NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(LCD_CS0_N,       DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_CS1_N,       DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_D0,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D1,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D2,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D3,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D4,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D5,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D6,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D7,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D8,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D9,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D10,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D11,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D12,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D13,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D14,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D15,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D16,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D17,         DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_D18,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(LCD_D21,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(LCD_D22,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(LCD_D23,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(LCD_DC0,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_DC1,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), //EN_3V3_SDCARD
	DEFAULT_PINMUX(LCD_DE,          DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_HSYNC,       DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_M1,          DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_PCLK,        DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_PWR0,        DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_PWR1,        DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_PWR2,        DISPLAYA,        PULL_DOWN,     NORMAL,     OUTPUT), //G
	DEFAULT_PINMUX(LCD_SCK,         DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_SDIN,        DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_SDOUT,       DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(LCD_VSYNC,       DISPLAYA,        NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(LCD_WR_N,        DISPLAYA,        NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(PEX_L2_CLKREQ_N, PCIE,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(PEX_L2_PRSNT_N,  PCIE,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(PEX_L2_RST_N,    PCIE,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(PEX_WAKE_N,      PCIE,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(CLK2_OUT,        EXTPERIPH2,      NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(CLK2_REQ,        DAP,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(GPIO_PV2,        OWR,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(GPIO_PV3,        RSVD1,           NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC1_CMD,      SDMMC1,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC1_DAT0,     SDMMC1,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC1_DAT1,     SDMMC1,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC1_DAT2,     SDMMC1,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC1_DAT3,     SDMMC1,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC3_CLK,      SDMMC3,          NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC3_CMD,      SDMMC3,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC3_DAT0,     SDMMC3,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC3_DAT1,     SDMMC3,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC3_DAT2,     SDMMC3,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC3_DAT3,     SDMMC3,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC3_DAT5,     SDMMC3,          NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SDMMC3_DAT6,     SDMMC3,          NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SDMMC3_DAT7,     SDMMC3,          NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(SDMMC4_CLK,      SDMMC4,          NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_CMD,      SDMMC4,          NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          PULL_UP,       NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(SDMMC4_RST_N,    RSVD1,           PULL_UP,       NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(CLK_32K_OUT,     BLINK,           NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(HDMI_CEC,        CEC,             NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(KB_COL0,         KBC,             PULL_UP,       TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(KB_COL1,         KBC,             PULL_UP,       TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(KB_COL2,         KBC,             PULL_UP,       TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(KB_COL3,         KBC,             NORMAL,        NORMAL,     OUTPUT), ////// check config state
	DEFAULT_PINMUX(KB_COL4,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(KB_COL5,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_COL6,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_COL7,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_ROW0,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(KB_ROW1,         RSVD2,           NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(KB_ROW2,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(KB_ROW3,         KBC,             PULL_UP,       NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_ROW4,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_ROW5,         OWR,             NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(KB_ROW6,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_ROW7,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G (Test point)
	DEFAULT_PINMUX(KB_ROW8,         KBC,             PULL_DOWN,     TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(KB_ROW9,         KBC,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_ROW10,        KBC,             NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(KB_ROW11,        KBC,             NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(KB_ROW12,        KBC,             NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(KB_ROW13,        KBC,             PULL_DOWN,     TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(KB_ROW14,        KBC,             NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(KB_ROW15,        KBC,             NORMAL,        TRISTATE,   INPUT), //GI
	DEFAULT_PINMUX(OWR,             OWR,             NORMAL,        NORMAL,     OUTPUT), // G (NC)

	/* Power I2C pinmux */
	I2C_PINMUX(PWR_I2C_SCL,		I2CPWR,          NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),
	I2C_PINMUX(PWR_I2C_SDA,		I2CPWR,          NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),

	DEFAULT_PINMUX(SYS_CLK_REQ,     SYSCLK,          NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(CLK3_OUT,        EXTPERIPH3,      NORMAL,        NORMAL,     OUTPUT), // Function o
	DEFAULT_PINMUX(CLK3_REQ,        DEV3,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(DAP4_DIN,        I2S3,            NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(DAP4_DOUT,       I2S3,            NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(DAP4_FS,         I2S3,            NORMAL,        NORMAL,     INPUT), // Function o
	DEFAULT_PINMUX(DAP4_SCLK,       I2S3,            NORMAL,        NORMAL,     INPUT), // Function o

	/* I2C1 pinmux */
	I2C_PINMUX(GEN1_I2C_SCL,	I2C1,            NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),
	I2C_PINMUX(GEN1_I2C_SDA,	I2C1,            NORMAL,        NORMAL,     INPUT,   DISABLE,   DISABLE),

	DEFAULT_PINMUX(GPIO_PU0,        RSVD1,           NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(GPIO_PU1,        RSVD1,           NORMAL,        NORMAL,     OUTPUT), //G (NC)
	DEFAULT_PINMUX(GPIO_PU2,        RSVD1,           NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(GPIO_PU3,        RSVD1,           NORMAL,        NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(GPIO_PU4,        PWM1,            NORMAL,        NORMAL,     OUTPUT), // G (NC)
	DEFAULT_PINMUX(GPIO_PU5,        PWM2,            NORMAL,        TRISTATE,   INPUT), // GI
	DEFAULT_PINMUX(GPIO_PU6,        RSVD1,           PULL_DOWN,     NORMAL,     OUTPUT), // G
	DEFAULT_PINMUX(UART2_CTS_N,     UARTB,           NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(UART2_RTS_N,     UARTB,           NORMAL,        NORMAL,     OUTPUT), // Function o
	DEFAULT_PINMUX(UART2_RXD,       IRDA,            NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(UART2_TXD,       IRDA,            NORMAL,        NORMAL,     OUTPUT), // Function o
	DEFAULT_PINMUX(UART3_CTS_N,     UARTC,           NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(UART3_RTS_N,     UARTC,           NORMAL,        NORMAL,     OUTPUT), // Function o
	DEFAULT_PINMUX(UART3_RXD,       UARTC,           NORMAL,        NORMAL,     INPUT), // Function
	DEFAULT_PINMUX(UART3_TXD,       UARTC,           NORMAL,        NORMAL,     OUTPUT), // Function o

	/*Add from 14r2*/
	DEFAULT_PINMUX(CLK1_REQ,        DAP,             NORMAL,        NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L0_PRSNT_N,  PCIE,            NORMAL,        NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L0_RST_N,    PCIE,            NORMAL,        NORMAL,     OUTPUT),
	DEFAULT_PINMUX(PEX_L0_CLKREQ_N, PCIE,            NORMAL,        NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L1_PRSNT_N,  PCIE,            NORMAL,        NORMAL,     INPUT),
	DEFAULT_PINMUX(PEX_L1_RST_N,    PCIE,            NORMAL,        NORMAL,     OUTPUT),
	DEFAULT_PINMUX(PEX_L1_CLKREQ_N, PCIE,            NORMAL,        NORMAL,     INPUT),
	DEFAULT_PINMUX(JTAG_RTCK,       RTCK,            NORMAL,        NORMAL,     OUTPUT),
	VI_PINMUX(VI_D6,                VI,              NORMAL,        NORMAL,     OUTPUT,    DISABLE, DISABLE),
	VI_PINMUX(VI_D8,                SDMMC2,          NORMAL,        NORMAL,     INPUT,     DISABLE, DISABLE),
	VI_PINMUX(VI_D9,                SDMMC2,          NORMAL,        NORMAL,     INPUT,     DISABLE, DISABLE),
	VI_PINMUX(VI_PCLK,              RSVD1,           PULL_UP,       TRISTATE,   INPUT,     DISABLE, DISABLE),
	VI_PINMUX(VI_HSYNC,             RSVD1,           NORMAL,        NORMAL,     INPUT,     DISABLE, DISABLE),
	VI_PINMUX(VI_VSYNC,             RSVD1,           NORMAL,        NORMAL,     INPUT,     DISABLE, DISABLE),
	/* SDMMC1 WP gpio */
	DEFAULT_PINMUX(VI_D11,          RSVD1,           PULL_UP,       NORMAL,     INPUT),
};