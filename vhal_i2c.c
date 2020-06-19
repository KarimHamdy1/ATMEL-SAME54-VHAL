
#if VHAL_I2C

#include "vhal.h"
#include "vbl.h"

#include "hal_i2c_m_sync.h"
#include "hal_atomic.h"
#include "hal_delay.h"
#include "hal_gpio.h"
#include "hal_init.h"
#include "hal_io.h"
#include "vhal_ser_clock_config.h"


#define printf(...) vbl_printf_stdout(__VA_ARGS__)
//#define printf(...)


typedef struct _ser_drv {
    uint32_t* SercomID;             //address of SERCOM instance corresponding to I2C instance, I2C0-> SERCOM6 ...
    struct i2c_m_sync_desc I2C;
    struct io_descriptor *I2C_io;
    VSemaphore sem ;
    uint8_t status;                 //0 = not configured, 1 = configured    
} i2cDriverStr;

#define NUM_I2C 2
uint32_t SERCOMLIST[NUM_I2C] = {SERCOM6,SERCOM3};
i2cDriverStr i2cDriver[NUM_I2C];





void I2C_CLOCK_init(uint8_t drvnum)
{
    if (drvnum == 0)
    {
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_CORE, CONF_GCLK_SERCOM6_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM6_GCLK_ID_SLOW, CONF_GCLK_SERCOM6_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

    hri_mclk_set_APBDMASK_SERCOM6_bit(MCLK);
    }else if (drvnum == 1)
    {
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

    hri_mclk_set_APBBMASK_SERCOM3_bit(MCLK);
    }

}


void I2CDriverInit(uint8_t drvnum, vhalI2CConf *conf)
{

    I2C_CLOCK_init(drvnum);
    i2c_m_sync_init(&i2cDriver[drvnum].I2C, i2cDriver[drvnum].SercomID);

    vhalPinSetToPeripheral(conf->scl, PRPH_I2C, PIN_CLASS_DATA2(conf->scl));
    vhalPinSetToPeripheral(conf->sda, PRPH_I2C, PIN_CLASS_DATA2(conf->sda));

    i2c_m_sync_get_io_descriptor(&i2cDriver[drvnum].I2C, &i2cDriver[drvnum].I2C_io);

    i2c_m_sync_enable(&i2cDriver[drvnum].I2C);
}

int vhalInitI2C(void *data) {
    (void)data;
    uint8_t i =0;

    for (i=0; i<NUM_I2C; i++) {
        i2cDriver[i].status = 0;
        i2cDriver[i].sem = vosSemCreate(1);
        i2cDriver[i].SercomID = SERCOMLIST[i];
    }

    return 0;
}

int vhalI2CInit(uint32_t i2c, vhalI2CConf *conf) {
    if (i2c >= PERIPHERAL_NUM(i2c))
        return -1;
    int i2cid = GET_PERIPHERAL_ID(i2c, i2c);
    
    I2CDriverInit(i2cid,conf);

    
    

    return 0;

}


int vhalI2CRead(uint32_t i2c, uint8_t* buf, uint32_t len, uint32_t timeout) {
    int i2cid = GET_PERIPHERAL_ID(i2c, i2c);
    int32_t ret;
    ret = io_read(i2cDriver[i2cid].I2C_io,buf, len);
    if (ret != 0 )
    {   
        return VHAL_HARDWARE_STATUS_ERROR;
    }
    //succefull read.


    return 0;
}

int vhalI2CTransmit(uint32_t i2c, uint8_t* tx, uint32_t txlen, uint8_t *rx, uint32_t rxlen, uint32_t timeout) {
    int i2cid = GET_PERIPHERAL_ID(i2c, i2c);
    int32_t ret;
    ret = io_write(i2cDriver[i2cid].I2C_io,tx, txlen);

    if (ret != 0 )
    {
        return VHAL_HARDWARE_STATUS_ERROR;
    }

    if (rxlen > 0) {

        ret = io_read(i2cDriver[i2cid].I2C_io,rx, rxlen);
        if (ret != 0 )
        {
            return VHAL_HARDWARE_STATUS_ERROR;
        }
    }

    return 0;
}

int vhalI2CDone(uint32_t i2c) {
    if (i2c >= PERIPHERAL_NUM(i2c))
        return -1;
    int i2cid = GET_PERIPHERAL_ID(i2c, i2c);    
    i2c_m_sync_deinit(&i2cDriver[i2cid].I2C);

    return 0;
}

int vhalI2CLock(uint32_t i2c) {
    if (i2c >= PERIPHERAL_NUM(i2c))
        return -1;
    vosSemWait(i2cDriver[GET_PERIPHERAL_ID(i2c, i2c)].sem);
    return 0;
}

int vhalI2CUnlock(uint32_t i2c) {
    vosSemSignal(i2cDriver[GET_PERIPHERAL_ID(i2c, i2c)].sem);
    return 0;
}

int vhalI2CSetAddr(uint32_t i2c, uint16_t addr) {
    if (i2c >= PERIPHERAL_NUM(i2c))
        return -1;
    int i2cid = GET_PERIPHERAL_ID(i2c, i2c);
    i2c_m_sync_set_slaveaddr(&i2cDriver[i2cid].I2C, addr, I2C_M_SEVEN); // this is seven bit chip address
    return 0;
}



#endif
