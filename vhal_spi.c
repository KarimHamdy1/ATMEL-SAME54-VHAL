
#if VHAL_SPI

#include "vhal.h"
#include "vbl.h"

#include "hal_spi_m_sync.h"
#include "hal_gpio.h"
#include "hal_io.h"
#include "hal_atomic.h"
#include "hal_delay.h"
#include "vhal_ser_clock_config.h"
#include "hpl_spi.h"

#define NUM_SPI 2

//#define printf(...) vbl_printf_stdout(__VA_ARGS__)
#define printf(...)


typedef struct _spi_drv {
    VThread thread;
    uint32_t* SercomID;             //address of SERCOM instance corresponding to I2C instance, I2C0-> SERCOM6 ...
    struct spi_m_sync_descriptor spiDesc;
    VSemaphore sem;
    uint32_t clock;
    uint8_t csPin; // chip select pin
    uint8_t status;
} SpiDrv;


//const Spi* const spireg[] = {SPI0, SPI1};
SpiDrv spidrv[NUM_SPI];
uint32_t SERCOMLIST[NUM_SPI] = {SERCOM4,SERCOM5};

/*
EDit indices of gclk_sercom
#############################################################################################################################################
*/
static uint32_t gclk_indexes[] = {CONF_GCLK_SERCOM4_CORE_FREQUENCY,CONF_GCLK_SERCOM5_CORE_FREQUENCY};

static uint8_t _spi_initialized = 0;


int vhalInitSPI(void *data) {
    (void)data;
    if (!_spi_initialized) {
        memset(spidrv, 0, sizeof(spidrv));
        int i;
        for (i = 0; i < PERIPHERAL_NUM(spi); i++) {
            int idx = GET_PERIPHERAL_ID(spi, i);
            spidrv[idx].sem = vosSemCreate(1);
            spidrv[idx].status = 0;
            spidrv[idx].SercomID = SERCOMLIST[i];
        }
        _spi_initialized = 1;
    }
    return 0;
}


void SPI_PORT_init(vhalSpiConf *conf)
{
    vhalPinWrite(conf->mosi,0);
    vhalPinSetMode(conf->mosi,PINMODE_OUTPUT_PUSHPULL);
    vhalPinSetToPeripheral(conf->mosi,PRPH_SPI,PIN_CLASS_DATA2(conf->mosi));

    vhalPinWrite(conf->sclk,0);
    vhalPinSetMode(conf->sclk,PINMODE_OUTPUT_PUSHPULL);
    vhalPinSetToPeripheral(conf->sclk,PRPH_SPI,PIN_CLASS_DATA2(conf->sclk));

    vhalPinSetMode(conf->miso,PINMODE_INPUT_PULLNONE);
    vhalPinWrite(conf->miso,0);
    vhalPinSetToPeripheral(conf->miso,PRPH_SPI,PIN_CLASS_DATA2(conf->miso));
}
void SPIClockInit(uint8_t drvnum)
{
    if (drvnum == 0)        //initialize clock of SERCOM4 --> SPI0
    {


    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM4_GCLK_ID_SLOW, CONF_GCLK_SERCOM4_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

    hri_mclk_set_APBDMASK_SERCOM4_bit(MCLK);
    }
    if (drvnum == 1)        //initialize clock of SERCOM5 --> SPI1
    {


    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

    hri_mclk_set_APBDMASK_SERCOM5_bit(MCLK);
    }
}

void SPIInit(uint8_t drvnum, vhalSpiConf *conf)
{

    SPIClockInit(drvnum);
    spi_m_sync_init(&spidrv[drvnum].spiDesc, spidrv[drvnum].SercomID);

    SPI_PORT_init(conf);
}

uint32_t calculate_baud_value(uint32_t fref, uint32_t fbaud) {
    //printf("Vhal fref: %d, dbaud %d\n", fref, fbaud);
    return (uint32_t)(((float)fref)/(2*fbaud) - 1);
}

int vhalSpiInit(uint32_t spi, vhalSpiConf *conf) {
    if (spi >= PERIPHERAL_NUM(spi))
        return -1;
    int spiid = GET_PERIPHERAL_ID(spi, spi);
    //printf("spiid =  %d\n",spiid);
    SPIInit(spiid,conf);
    switch (conf->mode) {
        case SPI_MODE_LOW_FIRST:
            spi_m_sync_set_mode(&spidrv[spiid].spiDesc,SPI_MODE_0);
            break;
        case SPI_MODE_LOW_SECOND:
            spi_m_sync_set_mode(&spidrv[spiid].spiDesc,SPI_MODE_1);
            break;
        case SPI_MODE_HIGH_FIRST:
            spi_m_sync_set_mode(&spidrv[spiid].spiDesc,SPI_MODE_2);
            break;
        case SPI_MODE_HIGH_SECOND:
            spi_m_sync_set_mode(&spidrv[spiid].spiDesc,SPI_MODE_3);
            break;
        default:
            return VHAL_HARDWARE_STATUS_ERROR;
    }
    uint32_t baud = calculate_baud_value(gclk_indexes[spiid],conf->clock);
    spi_m_sync_set_baudrate(&spidrv[spiid].spiDesc,baud);
    /*
    SPI_8_BITS=0
    SPI_16_BITS=1
    */
    switch (conf->bits) {
        case 0:
            spi_m_sync_set_char_size(&spidrv[spiid].spiDesc,SPI_CHAR_SIZE_8);
            break;
        case 1:
            spi_m_sync_set_char_size(&spidrv[spiid].spiDesc,SPI_CHAR_SIZE_16);
            break;
        default:
            return VHAL_HARDWARE_STATUS_ERROR;
    }

    spi_m_sync_enable(&spidrv[spiid].spiDesc);
    spidrv[spiid].clock = conf->clock;
    spidrv[spiid].csPin = conf->nss;
    return 0;

}

int vhalSpiLock(uint32_t spi) {
    if (spi >= PERIPHERAL_NUM(spi))
        return -1;
    vosSemWait(spidrv[GET_PERIPHERAL_ID(spi, spi)].sem);
    return 0;
}
int vhalSpiUnlock(uint32_t spi) {
    vosSemSignal(spidrv[GET_PERIPHERAL_ID(spi, spi)].sem);
    return 0;
}


int vhalSpiSelect(uint32_t spi) {
    //clear nss
    uint32_t spiid = GET_PERIPHERAL_ID(spi, spi);
    vhalPinWrite(spidrv[spiid].csPin, 0);
    return 0;
}
int vhalSpiUnselect(uint32_t spi) {
    //set nss
    uint32_t spiid = GET_PERIPHERAL_ID(spi, spi);
    vhalPinWrite(spidrv[spiid].csPin, 1);
    return 0;
}

int vhalSpiExchange(uint32_t spi, void *tosend, void *toread, uint32_t bytes) {
    int32_t errChk = 0;
    if (spi >= PERIPHERAL_NUM(spi))
        return -1;
    int spiid = GET_PERIPHERAL_ID(spi, spi);
    struct spi_xfer transStuct;
    transStuct.txbuf = tosend;
    transStuct.rxbuf = toread;
    transStuct.size = bytes;
    
    errChk = spi_m_sync_transfer(&spidrv[spiid].spiDesc,&transStuct);
    //printf("transStuct.txbuf = %x,transStuct.rxbuf= %x,transStuct.size=%d,errChk = %d \n",transStuct.txbuf,transStuct.rxbuf,transStuct.size,errChk );
    return 0;
}

int vhalSpiDone(uint32_t spi) {
    if (spi >= PERIPHERAL_NUM(spi))
        return -1;
    int spiid = GET_PERIPHERAL_ID(spi, spi);

    return 0;
}

#endif
