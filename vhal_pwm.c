

#if defined(VHAL_PWM) && VHAL_PWM


#include "vhal.h"
#include "vbl.h"
#include "hal_atomic.h"
#include "hal_delay.h"
#include "hal_gpio.h"
#include "vhal_periph_clock_config.h"
#include "vhal_ser_clock_config.h"
#include "hal_pwm.h"
#include "hpl_tcc.c"
#include "hal_pwm.h"
#include "hpl_tc_base.h"
#include "hpl_tc.c"

struct pwm_descriptor PWM_0;
struct pwm_descriptor PWM_1;
struct pwm_descriptor PWM_2;
struct pwm_descriptor PWM_3;

struct pwm_descriptor PWM_4;
struct pwm_descriptor PWM_5;
struct pwm_descriptor PWM_6;
struct pwm_descriptor PWM_7;



#define printf(...) vbl_printf_stdout(__VA_ARGS__)
//#define printf(...)



#define numPWMChannel 8
uint8_t pwmStatus = 0;

typedef struct pwm_drv {
    uint32_t period;
    uint32_t perDuty;
    uint8_t channel;
    uint8_t status;
} PWMDrv;

PWMDrv pwmDriver[numPWMChannel];
#define IOPORT_CUSTOM_CREATE_PIN(port, pin) (port * 32 + (pin))






void PWM_Clock_Conf()
{

        hri_mclk_set_APBBMASK_TCC0_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TCC0_GCLK_ID, CONF_GCLK_TCC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

        hri_mclk_set_APBAMASK_TC0_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

        hri_mclk_set_APBAMASK_TC1_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TC1_GCLK_ID, CONF_GCLK_TC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

        hri_mclk_set_APBDMASK_TC7_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TC7_GCLK_ID, CONF_GCLK_TC7_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

        hri_mclk_set_APBBMASK_TCC1_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TCC1_GCLK_ID, CONF_GCLK_TCC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

        hri_mclk_set_APBCMASK_TCC3_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TCC3_GCLK_ID, CONF_GCLK_TCC3_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

        hri_mclk_set_APBCMASK_TC5_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TC5_GCLK_ID, CONF_GCLK_TC5_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
        
        hri_mclk_set_APBDMASK_TC6_bit(MCLK);
        hri_gclk_write_PCHCTRL_reg(GCLK, TC6_GCLK_ID, CONF_GCLK_TC6_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
}
int vhalInitPWM(void* data){
    (void)data;
    if (!pwmStatus) {
        memset(pwmDriver, 0, sizeof(pwmDriver));
        int i;
        for (i = 0; i < numPWMChannel; i++) {
            pwmDriver[i].period = 0;
            pwmDriver[i].perDuty = 0;
            pwmDriver[i].channel = 0;
            pwmDriver[i].status = 0;
        }
    }
    PWM_Clock_Conf();

    
    return VHAL_OK;
}


int vhalPwmStart(int vpin, uint32_t period, uint32_t pulse, uint32_t npulses){
    int16_t pwmChannel =0;
    uint8_t k=0;
    uint32_t des_freq, pulse_freq;
    
    vhalPinSetToPeripheral(vpin,PRPH_PWM,PIN_CLASS_DATA0(vpin));
    
    uint32_t pin = IOPORT_CUSTOM_CREATE_PIN(PIN_PORT_NUMBER(vpin), PIN_PAD(vpin));
    pwmChannel = PIN_CLASS_DATA1(vpin);
    printf("pwmChannel = %d, pin = %d\n",pwmChannel,pin );

    switch (GET_TIME_UNIT(period)) {
        case MILLIS:
            des_freq = 1000u / GET_TIME_VALUE(period);
            break;
        case MICROS:
            des_freq = 1000000u / GET_TIME_VALUE(period);
            break;
        case NANOS:
            des_freq = 1000000000u / GET_TIME_VALUE(period);
            break;
        default:
            return VHAL_GENERIC_ERROR;
            break;
    }
    /*
    All PWM intances are running on 12Mhz clock (generic clock generator 0)
    */
    uint32_t base_freq = (CONF_GCLK_TCC0_FREQUENCY / 8);

    uint32_t per = (base_freq / des_freq);



    switch (GET_TIME_UNIT(pulse)) {
        case MILLIS:
            pulse_freq = 1000u / GET_TIME_VALUE(pulse);
            break;
        case MICROS:
            pulse_freq = 1000000u / GET_TIME_VALUE(pulse);
            break;
        case NANOS:
            pulse_freq = 1000000000u / GET_TIME_VALUE(pulse);
            break;
        default:
            return VHAL_GENERIC_ERROR;
            break;
    }

    uint32_t pulse_calculated = (base_freq / pulse_freq);    
    printf("pre = %d, period = %d,pulse = %d,pulse_calculated = %d\n",per,period,pulse,pulse_calculated );

    
    
    /*
    pwmStatus = 0 -> first PWM pin to be enabled.

    */
    switch (pwmChannel)
    {
        case 0:
        pwm_init(&PWM_0, TCC0, _tcc_get_pwm());
        pwm_set_parameters(&PWM_0, per, pulse_calculated);
        pwm_enable(&PWM_0);
        printf("vhalPwmStart, PWM0 started!\n" );
        break;
        case 1:
        pwm_init(&PWM_1, TC0, _tc_get_pwm());
        pwm_set_parameters(&PWM_1, per, pulse_calculated);
        pwm_enable(&PWM_1);
        printf("vhalPwmStart, PWM11111 started\n" );
        break;
        case 2:
        pwm_init(&PWM_2, TC1, _tc_get_pwm());
        pwm_set_parameters(&PWM_2, per, pulse_calculated);
        pwm_enable(&PWM_2);
        printf("vhalPwmStart, PWM22222222222 started\n" );
        break;
        case 3:
        pwm_init(&PWM_3, TC7, _tc_get_pwm());
        pwm_set_parameters(&PWM_3, per, pulse_calculated);
        pwm_enable(&PWM_3);
        printf("vhalPwmStart, PWM333333333 started\n" );
        break;
        case 4:
        pwm_init(&PWM_4, TCC1, _tcc_get_pwm());
        pwm_set_parameters(&PWM_4, per, pulse_calculated);
        pwm_enable(&PWM_4);
        printf("vhalPwmStart, PWM44444444444444444444 started\n" );
        break;
        case 5:
        pwm_init(&PWM_5, TCC3, _tcc_get_pwm());
        pwm_set_parameters(&PWM_5, per, pulse_calculated);
        pwm_enable(&PWM_5);
        printf("vhalPwmStart, PWM555555555555555555555 started\n" );
        break;
        case 6:
        pwm_init(&PWM_6, TC5, _tc_get_pwm());
        pwm_set_parameters(&PWM_6, per, pulse_calculated);
        pwm_enable(&PWM_6);
        printf("vhalPwmStart, PWM666666666666666666666666 started\n" );
        break;
        case 7:
        pwm_init(&PWM_7, TC6, _tc_get_pwm());
        pwm_set_parameters(&PWM_7, per, pulse_calculated);
        pwm_enable(&PWM_7);
        printf("vhalPwmStart, PWM77777777777777777777777777 started\n" );
        break;
        default:
        printf("#ERORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRr\n");
        break;

    }
    
        
    
    pwmDriver[pwmChannel].period = per;
    pwmDriver[pwmChannel].perDuty = pulse_calculated;
    pwmDriver[pwmChannel].channel = pwmChannel;
    pwmDriver[pwmChannel].status = 1;

    return VHAL_OK;
    
}



#endif