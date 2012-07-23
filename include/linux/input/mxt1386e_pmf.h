#ifndef _LINUX_MXT1386E_PMF_H
#define _LINUX_MXT1386E_PMF_H

#if defined(CONFIG_ARCH_ACER_T30)
#include "../../../arch/arm/mach-tegra/board-acer-t30.h"
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#endif
extern int acer_board_id;
extern int acer_board_type;

#define X_MIN                                 0x00
#define Y_MIN                                 0x00
#define X_MAX                                 1919
#define Y_MAX                                 1199
#define MXT_MAX_REPORTED_PRESSURE             255
#define MXT_MAX_TOUCH_SIZE                    255

#define Chip_Vendor                           "AT"
#define Reseved_Chip_Vendor                   "0"
#define ConfigVersion                         1180690 /* 120412 */
#define Reseved_Firmware_Info                 0
#define Reseved_ConfigVersion                 0
#define Reservedinfo                          0

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

enum {
	TOUCH_SENSITIVITY_SYMBOL_HIGH = 0,
	TOUCH_SENSITIVITY_SYMBOL_MEDIUM,
	TOUCH_SENSITIVITY_SYMBOL_LOW,
	TOUCH_SENSITIVITY_SYMBOL_COUNT,
};

#define TOUCH_SENSITIVITY_SYMBOL_DEFAULT TOUCH_SENSITIVITY_SYMBOL_MEDIUM

/* GEN_POWERCONFIG_T7 INSTANCE 0 */
u8 T07OBJ[3]  = {  50, 255,  10};

/* GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
u8 T08OBJ[10] = {  26,   0,  10,  10,   0,   0,   5,  17,  15,   0};

/* TOUCH_MULTITOUCHSCREEN_T9 INSTATNCE 0 */
u8 T09OBJ[35] = { 131,   0,   0,  30,  42,   0,  16,  70,   4,   5,
                   10,   5,   2,   0,  10,  15,  15,  10, 175,   4,
                  127,   7,   0,   0,   0,   0,   0,   0,  64,   0,
                   15,  25,   0,   0,   0};

/* SPT_COMMSCONFIG_T18 INSTANCE 0 */
u8 T18OBJ[2]  = {   0,   0};

/* PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTATNCE 0 */
u8 T24OBJ[19] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0};

/* SPT_SELFTEST_T25 INSTATNCE 0 */
u8 T25OBJ[6]  = {   0,   0,  65,  70,  75,   0};

/* PROCI_TWOTOUCHGESTUREPROCESSOR_T27 INSTATNCE 0 */
u8 T27OBJ[7]  = {   0,   0,   0,   0,   0,   0,   0};

/* SPT_USERDATA_T38 INSTATNCE 0 */
u8 T38OBJ[64] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0};

/* PROCI_GRIPSUPPRESSION_T40 INSTATNCE 0 */
u8 T40OBJ[5]  = {   0,   0,   0,   0,   0};

/* PROCI_TOUCHSUPPRESSION_T42 INSTATNCE 0 */
u8 T42OBJ[10] = {  33,  40,  80,  80,   0,   3,   0,   0,  10,   3};

/* SPT_DIGITIZER_T43 INSTANCE 0 */
u8 T43OBJ[7]  = {   0,   0,   0,   0,   0,   0,   0};

/* SPT_CTECONFIG_T46 INSTANCE 0 */
u8 T46OBJ[9]  = {  64,   0,  16,  32,   0,   0,   1,   0,   0};

/* SPT_CTECONFIG_T46 INSTANCE 0 */
u8 T46OBJ_0[9]= {  64,   0,  16,  32,   0,   0,   1,   0,   0};

/* PROCI_STYLUS_T47 INSTATNCE 0 */
u8 T47OBJ[10] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

/* PROCG_NOISESUPPRESSION_T48 INSTATNCE 0 */
u8 T48OBJ[54] = {   1, 128,   2,   0,   0,   0,   0,   0,   0,   0,
                   16,  55,   0,  10,   0,   0,   0,  63,   0,   0,
                   10,   0,  20,   0,   0,   0,   0,  16,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0};

/* PROCI_SHIELDLESS_T56 INSTATNCE 0  */
u8 T56OBJ[43] = {   1,   0,   1,  48,  21,  21,  21,  21,  21,  21,
                   21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
                   21,  21,  21,  21,  21,  21,  21,  21,  21,  21,
                   21,  21,  21,  21,   0,   0,   0,   0,   0,   1,
                    1,   6,   6};

struct sensitivity_mapping {
	int symbol;
	u8 value;
	u32 checksum_config;
};

struct sensitvity_ver_mapping {
	int Check_ConfigVersion;
	struct sensitivity_mapping *sens_ver_mapping;
};

static struct sensitivity_mapping v01_sensitivity_table[] = {
	{TOUCH_SENSITIVITY_SYMBOL_HIGH,           65,   13065926},
	{TOUCH_SENSITIVITY_SYMBOL_MEDIUM,         80,    5199430},
	{TOUCH_SENSITIVITY_SYMBOL_LOW,            85,    6771910},
};

static struct sensitivity_mapping v02_sensitivity_table[] = {
	{TOUCH_SENSITIVITY_SYMBOL_HIGH,           65,   15936665},
	{TOUCH_SENSITIVITY_SYMBOL_MEDIUM,         70,    4927248},
	{TOUCH_SENSITIVITY_SYMBOL_LOW,            75,   10693017},
};

static struct sensitivity_mapping v03_sensitivity_table[] = {
	{TOUCH_SENSITIVITY_SYMBOL_HIGH,           65,   14313378},
	{TOUCH_SENSITIVITY_SYMBOL_MEDIUM,         70,    6448171},
	{TOUCH_SENSITIVITY_SYMBOL_LOW,            75,    9069218},
};

static struct sensitvity_ver_mapping sensitivity_ver_table[] = {
	{ 1180434, v01_sensitivity_table},
	{ 1180451, v02_sensitivity_table},
	{ 1180690, v03_sensitivity_table},
};

#endif /* _LINUX_MXT1386E_PMF_H */
