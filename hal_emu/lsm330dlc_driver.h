/*
 * itg3200.h
 *
 * Created: 18/05/2012 17:51:08
 *  Author: sfx
 */ 


#ifndef LSM330DLC_H_
#define LSM330DLC_H_
#include "compiler.h"

#define GY_X 0
#define GY_Y 1
#define GY_Z 2

#define LSM330_ACC_SLAVE_ADDRESS  0b0011000
#define LSM330_GYRO_SLAVE_ADDRESS 0b1101010


#define LSM_GYRO_DEN_PIN AVR32_PIN_PD23

typedef struct lsm330dlc_acc_conf_t{
	uint8_t start_address;
	uint8_t ctrl_reg_a[5];
} lms330dlc_acc_conf_t;

typedef struct lsm330dlc_gyro_conf_t{
	uint8_t start_address;
	uint8_t	ctrl_reg_g[5];
} lsm330dlc_gyro_conf_t;


//CTRL_REG_A_1
#define LSM_ACC_DATARATE_OFF		0x00
#define LSM_ACC_DATARATE_1HZ		0x10
#define LSM_ACC_DATARATE_10Hz		0x20
#define LSM_ACC_DATARATE_25Hz		0x30
#define LSM_ACC_DATARATE_50Hz		0x40
#define LSM_ACC_DATARATE_100Hz		0x50
#define LSM_ACC_DATARATE_200Hz		0x60
#define LSM_ACC_DATARATE_400Hz		0x70
#define LSM_ACC_DATARATE_1620Hz		0x80
#define LSM_ACC_DATARATE_1344_5376Hz 0x90

#define LSM_ACC_X_EN	0x01
#define LSM_ACC_Y_EN	0x02
#define LSM_ACC_Z_EN	0x04
#define LSM_ACC_ALL_EN	0x07

#define LSM_ACC_LOW_POWER_EN 0x08

//CTRL_REG_A_2
#define LSM_ACC_HPIS1	0x01
#define LSM_ACC_HPIS2	0x02
#define LSM_ACC_HPCLICK	0x04
#define LSM_ACC_FDS		0x08
#define LSM_ACC_HPCF1	0x10
#define LSM_ACC_HPCF2	0x20
#define LSM_ACC_HPM0	0x40
#define LSM_ACC_HPM1	0x80

//CTRL_REG_A_3
#define LSM_ACC_OVERRUN_INT	0x02
#define LSM_ACC_FIFO_WM_INT	0x04
#define LSM_ACC_DRDY2_INT	0x08
#define LSM_ACC_DRDY1_INT	0x10
#define LSM_ACC_AOI_INT		0x40
#define LSM_ACC_CLICK_INT	0x80

//CTRL_REG_A_4
#define LSM_ACC_SPI_MODE	0x01
#define LSM_ACC_HIGH_RES	0x08

#define LSM_ACC_FULL_SCALE_2G	0x00
#define LSM_ACC_FULL_SCALE_4G	0x10
#define LSM_ACC_FULL_SCALE_8G	0x20
#define LSM_ACC_FULL_SCALE_16G	0x30

#define LSM_ACC_BIG_ENDIAN	0x40

//CTRL_REG_A_5
#define LSM_ACC_D4D_INT	0x04
#define LSM_ACC_LIR_INT	0x08
#define LSM_ACC_FIFO_EN	0x40
#define LSM_ACC_BOOT	0x80


//CTRL_REG_G_1
#define LSM_GYRO_DATARATE_95HZ		0x00
#define LSM_GYRO_DATARATE_190HZ		0x40
#define LSM_GYRO_DATARATE_380Hz		0x80
#define LSM_GYRO_DATARATE_760Hz		0xC0

// note: actual bandwidth depends on datarate - specified for 380Hz. Consult Datasheet.
#define LSM_GYRO_BANDWIDTH_20Hz		0x40
#define LSM_GYRO_BANDWIDTH_25Hz		0x50
#define LSM_GYRO_BANDWIDTH_50Hz		0x60
#define LSM_GYRO_BANDWIDTH_100Hz	0x70

#define LSM_GYRO_X_EN	0x01
#define LSM_GYRO_Y_EN	0x02
#define LSM_GYRO_Z_EN	0x04
#define LSM_GYRO_ALL_EN	0x07
#define LSM_GYRO_POWER_ON	0x08

//CTRL_REG_G_2
#define LSM_GYRO_HPCF0	0x01
#define LSM_GYRO_HPCF1	0x02
#define LSM_GYRO_HPCF2	0x04
#define LSM_GYRO_HPCF3	0x08
#define LSM_GYRO_HPM0	0x10
#define LSM_GYRO_HPM1	0x20
#define LSM_GYRO_LVL_EN	0x40
#define LSM_GYRO_EXTREN	0x80

//CTRL_REG_G_3
#define LSM_GYRO_FIFO_EMPTY_INT	0x01
#define LSM_GYRO_FIFO_OVRUN_INT	0x02
#define LSM_GYRO_FIFO_WM_INT	0x04
#define LSM_GYRO_DRDY_INT		0x08
#define LSM_GYRO_PP_OD			0x10
#define LSM_GYRO_H_L_ACT		0x20
#define LSM_GYRO_I1_BOOT		0x40
#define LSM_GYRO_I1_INT1		0x80

//CTRL_REG_G_4
#define LSM_GYRO_SPI_MODE	0x01

#define LSM_GYRO_FULL_SCALE_250		0x00
#define LSM_GYRO_FULL_SCALE_500		0x10
#define LSM_GYRO_FULL_SCALE_2000	0x20
#define LSM_GYRO_FULL_SCALE_2000	0x30

#define LSM_GYRO_BIG_ENDIAN	0x40
#define LSM_GYRO_BLOCK_DATA	0x80

//CTRL_REG_G_5
#define LSM_OUT_SEL0		0x01
#define LSM_OUT_SEL1		0x02
#define LSM_INT_SEL0		0x04
#define LSM_INT_SEL1		0x08
#define LSM_GYRO_HP_EN		0x10
#define LSM_GYRO_FIFO_EN	0x40
#define LSM_GYRO_BOOT		0x80



#define LSM_ACC_OUT_ADDRESS 0x27

#define LSM_ACC_FIFO_CTRL_ADDRESS 0x2E
#define LSM_ACC_FIFO_SRC_ADDRESS 0x2E

#define LSM_GYRO_OUT_ADDRESS 0x26

#define LSM_AUTO_INCREMENT 0x80

static const lms330dlc_acc_conf_t lsm_acc_default_config=
{.start_address=0x20 | LSM_AUTO_INCREMENT,
	.ctrl_reg_a=
	{	LSM_ACC_DATARATE_400Hz | LSM_ACC_ALL_EN ,	//CTRL_REG_G_1
		0,											//CTRL_REG_G_2
		0,											//CTRL_REG_G_3
		LSM_ACC_HIGH_RES | LSM_ACC_FULL_SCALE_8G,  //|LSM_ACC_BIG_ENDIAN,					//CTRL_REG_G_4
		LSM_ACC_FIFO_EN								//CTRL_REG_G_5
	}
};

static const lsm330dlc_gyro_conf_t lsm_gyro_default_config=
{.start_address=0x20| LSM_AUTO_INCREMENT,
	.ctrl_reg_g=
	{	LSM_GYRO_POWER_ON | LSM_GYRO_DATARATE_380Hz | LSM_GYRO_BANDWIDTH_50Hz | LSM_GYRO_ALL_EN,	//CTRL_REG_A_1
		0,											//CTRL_REG_A_2
		0,											//CTRL_REG_A_3
		LSM_GYRO_FULL_SCALE_2000|LSM_GYRO_BIG_ENDIAN,	//CTRL_REG_A_4
		LSM_GYRO_FIFO_EN
	}
};

static const uint8_t fifo_config[2]={LSM_ACC_FIFO_CTRL_ADDRESS, 0x80};

typedef struct{
	int8_t temperature;
	uint8_t status_register;
	int16_t axes[3];
} lsm_gyro_t;

typedef struct{
	uint8_t status_register;
	int16_t axes[3];
} lsm_acc_t;


void lsm330dlc_init(void);

lsm_gyro_t* lsm330dlc_driver_get_gyro_data(void);
lsm_acc_t* lsm330dlc_driver_get_acc_data(void);


#endif 