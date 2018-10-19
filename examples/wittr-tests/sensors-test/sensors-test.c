/* Copyright (c) 2018 NidaTech AB. All rights reserved.                      */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "ext-flash.h"
#include "lib/sensors.h"
#include "batmon-sensor.h"
#include "button-hal.h"
#include "wittratag/bmg-250-sensor.h"
#include "wittratag/lsm-303-ah-sensor.h"


/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "App"
#ifndef LOG_CONF_LEVEL_APP
#define LOG_CONF_LEVEL_APP LOG_LEVEL_NONE
#endif
#define LOG_LEVEL LOG_CONF_LEVEL_APP


static struct ctimer led_ct;
static int btn_last_duration;
static int btn_last_time;
/* ------------------------------------------------------------------------- */
typedef uint16_t adc_value_t;
/* ------------------------------------------------------------------------- */
int32_t v_ext;
int32_t v_main;

static void publish_led_off(void *d);

/*---------------------------------------------------------------------------*/
PROCESS(led_blinking_process, "LED blinking process");
PROCESS(hall_recording_process, "HALL recording process");
PROCESS(power_adc_process, "ADC Process");
PROCESS(sensors_test_process, "Sensors test process");
AUTOSTART_PROCESSES(&sensors_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensors_test_process, ev, data)
{
  static struct etimer et;
  int batmon_volt, chip_temp;
  int gyro[3];
  int acc[3], acc_temp;
  int mag[3];

  PROCESS_BEGIN();

  LOG_INFO("Running %s\n", PROCESS_CURRENT()->name);

  process_start(&led_blinking_process, NULL);
  process_start(&hall_recording_process, NULL);
  process_start(&power_adc_process, NULL);

  etimer_set(&et, 3 * CLOCK_SECOND);

  while(1){
    PROCESS_YIELD_UNTIL(etimer_expired(&et));
    // Convert from V to mV. Bits 10:8 are integer part, bits 7:0 are fractional
    // part. Fractional part is in the lower 8 bits, thus converting is done as
    // follows:
    // (1/256) / (1/1000) = 1000/256 = 125/32
    // This is done most efficiently by multiplying with 125 and then shifting
    // right 5 steps.
    SENSORS_ACTIVATE(batmon_sensor);
    batmon_volt = (batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT) * 125) >> 5;
    chip_temp = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
    LOG_INFO("Battery %d mV\n", batmon_volt);
    LOG_INFO("CPU temp %d C\n", chip_temp);

    SENSORS_ACTIVATE(bmg_250_sensor);
    PROCESS_YIELD_UNTIL((ev == sensors_event) && (data == &bmg_250_sensor));
    gyro[0] = bmg_250_sensor.value(BMG_250_SENSOR_TYPE_GYRO_X);
    gyro[1] = bmg_250_sensor.value(BMG_250_SENSOR_TYPE_GYRO_Y);
    gyro[2] = bmg_250_sensor.value(BMG_250_SENSOR_TYPE_GYRO_Z);
    LOG_INFO("Gyro X %d Y %d Z %d\n", gyro[0], gyro[1], gyro[2]);

    SENSORS_ACTIVATE(lsm_303_ah_acc_sensor);
    PROCESS_YIELD_UNTIL((ev == sensors_event) && (data == &lsm_303_ah_acc_sensor));
    acc[0] = lsm_303_ah_acc_sensor.value(LSM_303_AH_SENSOR_TYPE_ACC_X);
    acc[1] = lsm_303_ah_acc_sensor.value(LSM_303_AH_SENSOR_TYPE_ACC_Y);
    acc[2] = lsm_303_ah_acc_sensor.value(LSM_303_AH_SENSOR_TYPE_ACC_Z);
    LOG_INFO("Acceleration X %d Y %d Z %d\n", acc[0], acc[1], acc[2]);
    acc_temp = lsm_303_ah_acc_sensor.value(LSM_303_AH_SENSOR_TYPE_TMP);
    LOG_INFO("Board temp %d C\n", acc_temp);

    SENSORS_ACTIVATE(lsm_303_ah_mag_sensor);
    PROCESS_YIELD_UNTIL((ev == sensors_event) && (data == &lsm_303_ah_mag_sensor));
    mag[0] = lsm_303_ah_mag_sensor.value(LSM_303_AH_SENSOR_TYPE_MAG_X);
    mag[1] = lsm_303_ah_mag_sensor.value(LSM_303_AH_SENSOR_TYPE_MAG_Y);
    mag[2] = lsm_303_ah_mag_sensor.value(LSM_303_AH_SENSOR_TYPE_MAG_Z);
    LOG_INFO("Magnetometer X %d Y %d Z %d\n", mag[0], mag[1], mag[2]);

    LOG_INFO("HALL active %d for %d\n", btn_last_time, btn_last_duration);

    LOG_INFO("Vext %ld\n", v_ext);
    LOG_INFO("Vmain %ld\n", v_main);

    etimer_reset(&et);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(led_blinking_process, ev, data)
{
  static struct etimer et;
  static unsigned long time;

  PROCESS_BEGIN();

  LOG_INFO("Running %s\n", PROCESS_CURRENT()->name);

  //wait to join a network
  LOG_INFO("Waiting for RPL root\n");
  do{
    ctimer_set(&led_ct, CLOCK_SECOND / 16, publish_led_off, NULL);
    etimer_set(&et, CLOCK_SECOND);
    leds_on(LEDS_RED);
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_TIMER && etimer_expired(&et));
  }while(uip_ds6_get_global(ADDR_PREFERRED) == NULL);
  LOG_INFO("Joined network\n");

  //LED blinking fast for 2 seconds after joining the network
  time = clock_seconds();
  do{
    leds_on(LEDS_RED);
    ctimer_set(&led_ct, CLOCK_SECOND / 16, publish_led_off, NULL);
    etimer_set(&et, CLOCK_SECOND / 8);
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_TIMER && etimer_expired(&et));
  } while(clock_seconds() < (time + 2));

  etimer_set(&et, CLOCK_SECOND);

  LOG_INFO("Blink LED every 3 seconds when connected to the RPL network\n");
  while(1) {

    PROCESS_YIELD_UNTIL(etimer_expired(&et));

    while(uip_ds6_get_global(ADDR_PREFERRED) != NULL){
      ctimer_set(&led_ct, CLOCK_SECOND / 16, publish_led_off, NULL);
      etimer_set(&et, 3 * CLOCK_SECOND);
      leds_on(LEDS_RED);
      PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_TIMER && etimer_expired(&et));
    }

    etimer_reset(&et);
    leds_off(LEDS_RED);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hall_recording_process, ev, data)
{
  //static struct etimer et;
  //static clock_time_t time;
  button_hal_button_t *btn;

  const static gpio_hal_pin_t hall_power_pin = BOARD_IOID_HALL_PWR;

  PROCESS_BEGIN();

  LOG_INFO("Running %s\n", PROCESS_CURRENT()->name);

  gpio_hal_arch_clear_pin(hall_power_pin);
  gpio_hal_arch_pin_set_output(hall_power_pin);
  gpio_hal_arch_set_pin(hall_power_pin);

  btn = button_hal_get_by_index(0);

  LOG_INFO("Device button count: %u.\n", button_hal_button_count);

  if(btn) {
    LOG_INFO("%s on pin %u with ID=0, Logic=%s, Pull=%s\n",
           BUTTON_HAL_GET_DESCRIPTION(btn), btn->pin,
           btn->negative_logic ? "Negative" : "Positive",
               btn->pull == GPIO_HAL_PIN_CFG_PULL_UP ? "Pull Up" : "Pull Down");
  }

  while(1) {

    PROCESS_YIELD();

    if(ev == button_hal_press_event) {
      btn = (button_hal_button_t *)data;
      LOG_INFO("Press event (%s)\n", BUTTON_HAL_GET_DESCRIPTION(btn));

      if(btn == button_hal_get_by_id(BUTTON_HAL_ID_BUTTON_ZERO)) {
        LOG_INFO("This was button 0, on pin %u\n", btn->pin);
      }

      btn_last_time = clock_seconds();

    } else if(ev == button_hal_release_event) {
      btn = (button_hal_button_t *)data;
      LOG_INFO("Release event (%s)\n", BUTTON_HAL_GET_DESCRIPTION(btn));
    } else if(ev == button_hal_periodic_event) {
      btn = (button_hal_button_t *)data;
      LOG_INFO("Periodic event, %u seconds (%s)\n", btn->press_duration_seconds,
             BUTTON_HAL_GET_DESCRIPTION(btn));

      btn_last_duration = btn->press_duration_seconds;

      if(btn->press_duration_seconds > 5) {
        LOG_INFO("%s pressed for more than 5 secs. Do custom action\n",
               BUTTON_HAL_GET_DESCRIPTION(btn));
      }
    }
  }

  PROCESS_END();
}
/* ------------------------------------------------------------------------- */
static uint8_t adc_channels[] = {
  BOARD_AUXIO_ADC_VEXT,
  BOARD_AUXIO_ADC_VMAIN,
};
#define NUF_ADC_CHANNELS (sizeof(adc_channels) / sizeof(adc_channels[0]))
adc_value_t adc_samples[NUF_ADC_CHANNELS] = { 0 };
enum adc_idx {
  ADC_CHANNEL_VEXT,
  ADC_CHANNEL_VMAIN,
};

const static gpio_hal_pin_t adc_on_pin = BOARD_IOID_ADC_ON;
//const static gpio_hal_pin_t adc_id_pin = BOARD_IOID_ADC_ID; // Make it digital
/* ------------------------------------------------------------------------- */
inline int32_t
adcvalue2millivolt(const int32_t value)
{
  int32_t uv;
  uv = ti_lib_aux_adc_value_to_microvolts(AUXADC_FIXED_REF_VOLTAGE_NORMAL,
                                          value);
  return uv / 1000;
}
/* ------------------------------------------------------------------------- */
const static int32_t adc_divider_total = 133; /* kohm */
const static int32_t adc_divider_bottom = 33; /* kohm */
const static int32_t diode_voltage_drop = 300; /* mV */
/* ------------------------------------------------------------------------- */
static int32_t
unscale_adc_millivolt(const int32_t millivolt_in)
{
  return millivolt_in * adc_divider_total / adc_divider_bottom;
}
/* ------------------------------------------------------------------------- */
PROCESS_THREAD(power_adc_process, ev, data)
{
  static struct etimer et_power_adc, et_delay;

  PROCESS_BEGIN();

  gpio_hal_arch_clear_pin(adc_on_pin);
  gpio_hal_arch_pin_set_output(adc_on_pin);

  etimer_set(&et_power_adc, CLOCK_SECOND * 4);

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et_power_adc));

    /* Turn on ADC_ON and wait one second to voltages to stabilize  */
    gpio_hal_arch_set_pin(adc_on_pin);
    etimer_set(&et_delay, CLOCK_SECOND);
    PROCESS_YIELD_UNTIL(etimer_expired(&et_delay));

    /* intialisation of ADC */
    ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
    while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON));

    /*
     * Enable clock for ADC digital and analog interface (not currently enabled
     * in driver)
     */
    ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK |
                                AUX_WUC_SMPH_CLOCK);
    while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK |
                                      AUX_WUC_SMPH_CLOCK)
          != AUX_WUC_CLOCK_READY);

    /* Disable scaling, which gives maximum input 1.44 V */
    /* This cannot be used, since ADC_ID signal can go above 1.44 V */
    /* 17.4.8.4 */
    // ti_lib_aux_adc_disable_input_scaling();

    /* Set up ADC range, AUXADC_REF_FIXED = nominally 4.3 V */
    ti_lib_aux_adc_enable_sync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_2P7_US,
                               AUXADC_TRIGGER_MANUAL);
    for (uint8_t adc_channel = 0; adc_channel < NUF_ADC_CHANNELS; ++adc_channel) {
      /* Connect defined port as analog input. */
      ti_lib_aux_adc_select_input(adc_channels[adc_channel]);

      /* Trigger ADC converting */
      ti_lib_aux_adc_gen_manual_trigger();

      /* Read value */
      adc_samples[adc_channel] = ti_lib_aux_adc_read_fifo();
    }

    /* Shut the adc down */
    ti_lib_aux_adc_disable();
    gpio_hal_arch_clear_pin(adc_on_pin);

//    power_state_update(adc_samples[ADC_CHANNEL_VEXT],
//                       adc_samples[ADC_CHANNEL_VMAIN]);
    LOG_INFO("ADC Vext %d\n", adc_samples[ADC_CHANNEL_VEXT]);
    LOG_INFO("ADC Vmain %d\n", adc_samples[ADC_CHANNEL_VMAIN]);

    v_ext = unscale_adc_millivolt(adcvalue2millivolt(adc_samples[ADC_CHANNEL_VEXT]));
    v_main = unscale_adc_millivolt(adcvalue2millivolt(adc_samples[ADC_CHANNEL_VMAIN]));

    v_main += diode_voltage_drop;

    etimer_reset(&et_power_adc);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static void
publish_led_off(void *d)
{
  leds_off(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
