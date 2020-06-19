
#if VHAL_ADC

#include "vhal.h"

#include "hal_atomic.h"
#include "hal_delay.h"
#include "hal_gpio.h"
#include "vhal_periph_clock_config.h"
#include "vhal_ser_clock_config.h"
#include "hal_adc_sync.h"

#include "hpl_adc.c"
//#define printf(...) vbl_printf_stdout(__VA_ARGS__)
#define printf(...)


#define NUM_ADC_PINS 16
typedef struct _adc_drv {
    struct adc_sync_descriptor ADC_0;
    uint8_t status;
    uint8_t pin[16];
} ADCDriver;
ADCDriver ADCDrv;



/*
We are mapping Zerynth ADC channels ( A0, A1...) to Atmel SAME54 physical channels ( AIN1, AIN2...)
index = Zerynth ADC channels
value = MUX channel value of AIN of ADC in atmelsame54
look at : adc.h:186
*/
uint8_t mapAinput2ADCpins[16] = {0,1,6,7,8,9,10,12,13,14,0,0,0,0,0,0,0};


void ADC_0_CLOCK_init(void)
{
    hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
    hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}


int vhalInitADC(void *data) {
    (void)data;    
    return 0;
}


int vhalAdcGetPeripheralForPin(int vpin) {
    if (PIN_CLASS(vpin) != PINCLASS_ANALOG)
        return -1;
    return PIN_CLASS_DATA1(vpin);
}

int vhalAdcInit(uint32_t adc, vhalAdcConf *conf) {
    if (adc >= PERIPHERAL_NUM(adc))
        return -1;
    int i=0;

    ADCDrv.status = 0;
    for (i=0; i<NUM_ADC_PINS; i++) {
        ADCDrv.pin[i] = 0;
    }
    
    ADC_0_CLOCK_init();
    //printf("vhalAdcInit, adc = %d\n",adc);

    return 0;
}


int vhalAdcPrepareCapture(uint32_t adc, vhalAdcCaptureInfo *info) {
    if (adc >= PERIPHERAL_NUM(adc))
        return -1;
    int adcid,i;
    uint8_t adc_ch;
    if (info->npins > 16 || info->npins <= 0)
        return -1;
    for (i = 0; i < info->npins; i++) {
        int vpin = info->pins[i];
        adc_ch = PIN_CLASS_DATA0(vpin);
        //printf("vhalAdcPrepareCapture, vpin = %d, adc_ch = %d\n",vpin, adc_ch);
        vhalPinSetToPeripheral(vpin, PRPH_ADC, PIN_CLASS_DATA1(vpin));
        
    }
    if (ADCDrv.status == 0)
    {
    adc_sync_init(&ADCDrv.ADC_0, ADC0, (void *)NULL);
    ADCDrv.status = 1;
    
    }
    adc_sync_enable_channel(&ADCDrv.ADC_0, 6);
    adc_sync_set_inputs(&ADCDrv.ADC_0, mapAinput2ADCpins[adc_ch], ADC_INPUTCTRL_MUXNEG_GND_Val, mapAinput2ADCpins[adc_ch]);


    info->sample_size = 2;
    if (info->samples > 1 && info->samples % 2 != 0)
        info->samples++;
    return info->sample_size * info->samples * info->npins;

}


void adc_convert(uint8_t *buffer, uint32_t samples) {

}
void adc_stop() {

}


int vhalAdcRead(uint32_t adc, vhalAdcCaptureInfo *info) {
    //adc_sync_enable_channel(&ADCDriver.ADC_0, adc_ch);

    int i, j, adcid, sample;
    uint8_t ch;
    int32_t readChk;
    uint16_t *buf = (uint16_t*) info->buffer;
    switch(info->capture_mode){
        case ADC_CAPTURE_SINGLE:
            //printf("Go!#########################################\n");
            for(j = 0; j < info->samples; j++){
                //printf("Sample: %d\n", j);
                for(i = 0; i < info->npins; i++){
                    ch = PIN_CLASS_DATA0(info->pins[i]);
                    adcid = PIN_CLASS_DATA1(info->pins[i]);
                    ADCDrv.pin[adcid] = ch;
                    readChk = adc_sync_read_channel(&ADCDrv.ADC_0, 6, buf, 2);
                    //printf("Pin: %d - ch %d - adcid %d, readChk = %d\n", i, ch, adcid,readChk);
                    buf++;
                }
            }
            break;
        default:
            return -1;
    }
    return 0;
}

int vhalAdcDone(uint32_t adc) {

    return 0;
}
#else


#endif

//[warning] There are 18 missing symbols! This VM does not support the requested features! ['_adc_sync_init', '_adc_sync_deinit', '_adc_sync_enable_channel', '_adc_sync_disable_channel', '_adc_sync_get_data_size', '_adc_sync_convert', '_adc_sync_is_channel_conversion_done', '_adc_sync_read_channel_data', '_adc_sync_set_reference_source', '_adc_sync_set_resolution', '_adc_sync_set_inputs', '_adc_sync_set_thresholds', '_adc_sync_set_channel_gain', '_adc_sync_set_conversion_mode', '_adc_sync_set_channel_differential_mode', '_adc_sync_set_window_mode', '_adc_sync_get_threshold_state', '_adc_sync_is_channel_conversion_done']
