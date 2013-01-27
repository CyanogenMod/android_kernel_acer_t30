#ifndef _LINUX_MXT1386E_PM_H
#define _LINUX_MXT1386E_PM_H

#if defined(CONFIG_ARCH_ACER_T30)
#include "../../../arch/arm/mach-tegra/board-acer-t30.h"
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#endif
extern int acer_board_id;
extern int acer_board_type;

#define X_MIN                                 0x00
#define Y_MIN                                 0x00
#define X_MAX                                 1279
#define Y_MAX                                 799
#define MXT_MAX_REPORTED_PRESSURE             255
#define MXT_MAX_TOUCH_SIZE                    255

#define Chip_Vendor                           "AT"
#define Reseved_Chip_Vendor                   "0"
#define ConfigChecksum                        3780549  /* 39AFC5 */
#define ConfigVersion                         1183762  /* 121012 */
#define Reseved_Firmware_Info                 0
#define Reseved_ConfigVersion                 0
#define Reservedinfo                          0

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

// Stock: high = 65, medium = 70, low = 75
#define TOUCH_SENSITIVITY_MIN 45
#define TOUCH_SENSITIVITY_MAX 75
#define TOUCH_SENSITIVITY_DEFAULT 70

/* GEN_POWERCONFIG_T7 INSTANCE 0 */
u8 T07OBJ[3]  = {  15, 255,  10};

/* GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
u8 T08OBJ[10] = {  26,   0,  10,  10,   0,   0,   5,  40,  15, -52};

/* TOUCH_MULTITOUCHSCREEN_T9 INSTATNCE 0 */
u8 T09OBJ[35] = { 131,   0,   0,  30,  42,   0,  16,  70,   2,   5,
                   10,   5,   1,  33,  10,  15,  15,  10,  31,   3,
                  255,   4,   0,   0,   0,   0,   0,   0,  64,   0,
                   15,  25,   0,   0,   0};

/* SPT_COMMSCONFIG_T18 INSTANCE 0 */
u8 T18OBJ[2]  = {   0,   0};

/* PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTATNCE 0 */
u8 T24OBJ[19] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   0};

/* SPT_SELFTEST_T25 INSTATNCE 0 */
u8 T25OBJ[6]  = {   0,   0,  65,  80,  85,   0};

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
u8 T42OBJ[10] = {  33,  15,  55,  55,   0,   3,   0,   0,  10,   3};

/* SPT_DIGITIZER_T43 INSTANCE 0 */
u8 T43OBJ[7]  = {   0,   0,   0,   0,   0,   0,   0};

/* SPT_CTECONFIG_T46 INSTANCE 0 */
u8 T46OBJ[9]  = {  64,   0,  16,  32,   0,   0,   1,   0,   0};

/* SPT_CTECONFIG_T46 INSTANCE 0 */
u8 T46OBJ_0[9]= {   4,   0,   8,  16,   0,   0,   1,   0,   0};

/* PROCI_STYLUS_T47 INSTATNCE 0 */
u8 T47OBJ[10] = {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0};

/* PROCG_NOISESUPPRESSION_T48 INSTATNCE 0 */
u8 T48OBJ[54]  = {   1, 128,   2,   0,   0,   0,   0,   0,   0,   0,
                    16,  60,   0,  10,   0,   0,   0,  63,   0,   0,
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

#endif /* _LINUX_MXT1386E_PM_H */
