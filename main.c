/**
 * @file        main.c
 * @author      Harshvardhan Ruchandani
 * @date        Created: 8th May, 2020
 * @copyright   Copyright (c) 2023 Globalstar Inc. All Rights Reserved.
 * NO WARRANTY of ANY KIND is provided. This heading must NOT be
 * removed from the file.
 */

/* FreeRTOS includes. */
#include "gstar_sdk_config.h"
#include "sdk_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "nrf_crypto_init.h"

#include "GSTAR_HAL_gpio.h"
#include "GSTAR_HAL_i2c_1.h"
#include "GSTAR_HAL_i2c_rc.h"
#include "GSTAR_HAL_uart_2.h"
#include "GSTAR_HAL_wdt.h"

#include "GSTAR_Library_GPS.h"
#include "GSTAR_Library_accelerometer.h"
#include "GSTAR_Library_bleadv.h"
#include "GSTAR_Library_blecommon.h"
#include "GSTAR_Library_blenus.h"
#include "GSTAR_Library_blescan.h"
#include "GSTAR_Library_command_parse.h"
#include "GSTAR_Library_eventqueue.h"
#include "GSTAR_Library_extflash.h"
#include "GSTAR_Library_gpio.h"
#include "GSTAR_Library_rtc.h"
#include "GSTAR_Library_transmitter.h"
#include "GSTAR_Library_usb.h"

#include "GSTAR_UAPI_accelerometer_plat.h"
#include "GSTAR_UAPI_accelerometer_self_test.h"
#include "GSTAR_UAPI_asicdfu.h"
#include "GSTAR_UAPI_bleadv.h"
#include "GSTAR_UAPI_bleasicdfu.h"
#include "GSTAR_UAPI_blecommon.h"
#include "GSTAR_UAPI_bleconfigservice.h"
#include "GSTAR_UAPI_blecus.h"
#include "GSTAR_UAPI_blesensorservice.h"
#include "GSTAR_UAPI_config.h"
#include "GSTAR_UAPI_configservicedatahandling.h"
#include "GSTAR_UAPI_device.h"
#include "GSTAR_UAPI_evtqueue.h"
#include "GSTAR_UAPI_flash_log.h"
#include "GSTAR_UAPI_gpio.h"
#include "GSTAR_UAPI_gpio_plat.h"
#include "GSTAR_UAPI_gps_plat.h"
#include "GSTAR_UAPI_inputs.h"
#include "GSTAR_UAPI_log.h"
#include "GSTAR_UAPI_messages.h"
#include "GSTAR_UAPI_micropython.h"
#include "GSTAR_UAPI_power.h"
#include "GSTAR_UAPI_power_plat.h"
#include "GSTAR_UAPI_rrscheduler.h"
#include "GSTAR_UAPI_security.h"
#include "GSTAR_UAPI_transmitter.h"
#include "GSTAR_UAPI_transmitter_queue.h"
#include "GSTAR_UAPI_urc.h"

#include "GSTAR_APP_AT.h"
#include "GSTAR_APP_NUS_AT.h"
#include "GSTAR_APP_OTA_events.h"
#include "GSTAR_APP_POWER.h"
#include "GSTAR_APP_TOD.h"
#include "GSTAR_APP_Track.h"
#include "GSTAR_APP_UART_AT.h"
#include "GSTAR_APP_USB_AT.h"
#include "GSTAR_APP_uart.h"
#define ARDUINO_13_PIN              NRF_GPIO_PIN_MAP(1, 15)  // Digital pin 13
#define ARDUINO_12_PIN              NRF_GPIO_PIN_MAP(1, 14)  // Digital pin 12
static i2c_config_t config = {
  .u32_scl_pin = ARDUINO_13_PIN,
  .u32_sda_pin = ARDUINO_12_PIN,
  .frequency = I2C_1_FREQUENCY,
  .u8_interrupt_priority = APP_IRQ_PRIORITY_LOW,
};

// Initialize ST150 hal and libraries in dependency order
static void library_init(void)
{
  uint32_t dummy_ret;

  // This should replace the GPIO init above.
  g_hal_gpio_init();

  g_library_gpio_init();

  dummy_ret = g_init_sat_modem();
  if (dummy_ret == 0)
  {
    GSTAR_LOG_DEBUG("Transmitter Library initialized");
  }
  else
  {
    GSTAR_LOG_ERROR("Transmitter Library Failed initialized");
  }

  qspi_lib_init();

  dummy_ret = init_gps_library();

  i2c1_mutex_init();
  i2c_initialize_hal(NULL);
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, char * pcTaskName)
{
  /* This will get called if a stack overflow is detected during the context
   switch.  Set configCHECKFORSTACKOVERFLOWS to 2 to also check for stack
   problems within nested interrupts, but only do this for debug purposes as
   it will increase the context switch time. */
  (void) pxTask;
  (void) pcTaskName;
  taskDISABLE_INTERRUPTS();
  asm("BKPT");
  for (;;)
  {
  }
}

void fault_stack_dump(uint32_t * stack_addr)
{
  volatile uint32_t r0;
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r12;
  volatile uint32_t lr;  /* Link register. */
  volatile uint32_t pc;  /* Program counter. */
  volatile uint32_t psr; /* Program status register. */

  r0 = stack_addr[0];
  r1 = stack_addr[1];
  r2 = stack_addr[2];
  r3 = stack_addr[3];

  r12 = stack_addr[4];
  lr = stack_addr[5];
  pc = stack_addr[6];
  psr = stack_addr[7];

  /* Developer can now check the registers to see what happened */
  for (;;)
    asm("bkpt 0");
}

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler(void) __attribute__((naked));

void HardFault_Handler(void)
{
  __asm volatile(
    " .align 0                                                  \n"
    " tst lr, #4                                                \n"
    " ite eq                                                    \n"
    " mrseq r0, msp                                             \n"
    " mrsne r0, psp                                             \n"
    " ldr r1, [r0, #24]                                         \n"
    " ldr r2, handler2_address_const                            \n"
    " bx r2                                                     \n"
    " handler2_address_const: .word fault_stack_dump            \n");
}

/// Prints runtime and stack free stats for every thread, periodically
#define TASK_STATUS_BUFF_SIZE (sizeof(TaskStatus_t) * 20)

static void t_print_task_stats(void)
{
  static uint8_t pxU8TaskStatusArray[TASK_STATUS_BUFF_SIZE];
  TaskStatus_t * pxTaskStatusArray = (TaskStatus_t *) &pxU8TaskStatusArray;
  volatile UBaseType_t uxArraySize, x = 0;
  float ulStatsAsPercentage = 0.0f;

  // Drop to report once a minute after the initial time it runs
  change_task_frequency(PRINT_TASK_STATS_TASK_PRIORITY, (10 * 60 * 1000));

  uxArraySize = uxTaskGetNumberOfTasks();

  uxArraySize =
    uxTaskGetSystemState(pxTaskStatusArray, TASK_STATUS_BUFF_SIZE, NULL);

  GSTAR_LOG_INFO("\r\n\tThread Status: num threads %u\r\n", uxArraySize);
  for (x = 0; x < uxArraySize; x++)
  {
    GSTAR_LOG_INFO("\ttask: %s Unused Mem Remaining: %u",
      GSTAR_LOG_PUSH((char *) pxTaskStatusArray[x].pcTaskName),
      pxTaskStatusArray[x].usStackHighWaterMark);
  }
  GSTAR_LOG_INFO("\r\n");
  GSTAR_LOG_INFO("FreeRTOS has %d bytes of heap remaining.",
    xPortGetFreeHeapSize());
  // many times initial reset reason is not seen
  // due to log buffer overflow just printing it
  // every time to find the last reset reason
  print_reset_reason(print_reset_reason_debug);
}

// This is the WDT feed task to ensure the RR scheduler is running properly.
// The task runs every 60 seconds and the WDT timout is 120 seconds so the
// latency of the RR sched or FreeRTOS should trip the WDT. Extensive Flash log
// writes/reads will stop RR WDT feed kick Care must be taken during long
// reads/writes since flash operation are still blocking operations
void wdt_feed_task(void)
{
  static uint8_t channel = -1;
  static uint32_t system_on_time = 0;

  if (channel == (uint8_t) -1)
  {
    gstar_hal_wdt_channel_alloc(&channel);
  }

  gstar_hal_wdt_channel_feed(channel);

  system_on_time = rtc_get_on_time();   // disregard overflow

  GSTAR_LOG_DEBUG("WD Feeded by RR | System ON Time : %d seconds",
    system_on_time);
}

/// Main thread of the code base. Helps initialize all the services needed by
/// the device and goes into a forever loop
///
/// @return should not return
int main(void)
{
  ret_code_t err_code;
  gstar_hal_wdt_config_t wdt_config;

  // Clocks Init
  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);
  nrf_drv_clock_lfclk_request(NULL);

  // Init and enable the WDT
  // The Softdevice Task and the RR Sched Task will feed the WDT
  gstar_hal_wdt_get_default_config(&wdt_config);

  // This is just for developement. The WDT will pause when the cpu is halted by
  // the debugger. Set to GSTAR_HAL_WDT_BEHAVIOUR_RUN_SLEEP_HALT for production
  // Set to GSTAR_HAL_WDT_BEHAVIOUR_RUN_SLEEP to capture WDT during debugging
  wdt_config.behaviour = GSTAR_HAL_WDT_BEHAVIOUR_RUN_SLEEP_HALT;
  // Safety net to avoid a reset during ASIC DFU
  wdt_config.reload_val_ms = 120000;
  gstar_hal_wdt_init(&wdt_config, NULL);

  // Pre-allocated two WDT channels for the two feeds we're using in this
  // implementation. We need this because the WDT can't be configured after it's
  // enabled and we want to enable the wdt before initializing the device to
  // catch any lock up that may occur because the FreeRTOS scheduler runs.
  gstar_hal_wdt_channels_prealloc(2);

  // Enable the WDT now. It'll take 120 seconds to time out. The initialization
  // should take less than that. Then the first feed will occur with the
  // FreeRTOS scheduler runs.
  gstar_hal_wdt_enable();

  // dummy initial settings for development.
  // initial settings should be pulled from QSPI config
  gstar_log_settings_t log_settings;
  log_settings.ar_backend_enabled[GSTAR_LOG_RTT_BACKEND] = true;
  log_settings.ar_backend_enabled[GSTAR_LOG_NUS_BACKEND] = false;
  log_settings.ar_backend_enabled[GSTAR_LOG_UART_BACKEND] = false;
  log_settings.ar_backend_enabled[GSTAR_LOG_USB_BACKEND] = false;
  // keep flash logs off unless absolutely needed no filters
  log_settings.ar_backend_enabled[GSTAR_LOG_FLASH_BACKEND] = true;
  log_settings.u32_num_filters = 0;

  err_code = gstar_log_init(&log_settings);
  APP_ERROR_CHECK(err_code);

  gstar_default_log_filter_setup();   // no error expected

  GSTAR_LOG_INFO("\n\n SYSTEM RESET\r\n");
  GSTAR_LOG_INFO("ST150 Nordic FW Version: %d.%d.%d", FW_VERSION_MAJOR,
    FW_VERSION_MINOR, FW_VERSION_PATCH);

  err_code = rtc_init();
  APP_ERROR_CHECK(err_code);

  // UAPI Event Queue
  uapi_evtqueue_init();   // Must occur before security_init

  // URC flash logging
  err_code = gstar_uapi_flash_log_init();
  APP_ERROR_CHECK(err_code);

  library_init();   // Initialize all ST150 libraries

  // Config Library Dependencies
  // When we implimented SKU based device identification, it was necessary to
  // move this to within "uapi_config_init_settings()" Because, it has to happen
  // AFTER reading the settings out of the flash (because that's where the SKU
  // is), but before setting any defaults or device SKU based settings.  So, it
  // is in the proper place within the function below, now.
  // uapi_device_init();

  // Config Library
  uapi_config_init_settings();

  // Do carrier board specific config in here
  if (g_device_type_properties.device_type ==
      GLOBALSTAR_DEVICE_TYPE_ST150_SLAP_N_TRACK)
  {
    // check for battery fault
    app_power_check_battery_fault();
    g_library_gpio_gpio_cleanup();
  }
void i2ctask(void);
void i2cTask(){
    printf("Grabbing I2C Data\n");
    uint8_t data[20];
    ghal_ret_code_t code = i2c_simple_read(0x19, data, 20);
    printf("%s",data);

}
  //****RR Scheduler Tasks****/
  init_scheduler();
  // clang-format off
    //      Task                                           Priority                                                        Enabled,  Run,      Oneshot,Period
    add_task(wdt_feed_task,                                 WDT_FEED_TASK_PRIORITY,                                         true,     true,     false,  WDT_FEED_TASK_PERIOD);
    add_task(i2cTask,                    UAPI_ACCELEROMETER_LOGIC_TASK_PRIORITY,                         true,    true,    false,  UAPI_ACCELEROMETER_LOGIC_TASK_PERIOD);
    add_task(t_uapi_gpio_isr_task,                          UAPI_GPIO_ISR_TASK_PRIORITY,                                    false,    false,    true,   0);
    add_task(t_uapi_security_temp_auth_timeout_task,        UAPI_RR_SECURITY_MODULE_TEMP_AUTH_TIMEOUT_TASK_PRIORITY,        false,    false,    false,  UAPI_RR_SECURITY_MODULE_TEMP_AUTH_TIMEOUT_TASK_PERIOD_MS);
    add_task(set_interval_message                  ,        UAPI_QoS                                               ,        false,    false,    false,  UAPI_QoS_PERIOD_MS);
    add_task(uapi_security_settings_refresh_task,           UAPI_SECURITY_SETTINGS_CHANGED_REFRESH_TASK_PRIORITY,           false,    false,    true,   0);
    add_task(uapi_accelerometer_plat_init_task,             UAPI_ACCELEROMETER_PLAT_INIT_TASK_PRIORITY,                     false,    false,    true,   0);
    add_task(uapi_transmitter_lifetime_validity_task,       UAPI_TRANSMITTER_LIFETIME_VALIDITY_TASK_PRIORITY,                true,    false,    false,  UAPI_LIFETIME_PERIOD_MS);
    add_task(uapi_transmitter_settings_refresh_task,        UAPI_TRANSMITTER_SETTINGS_CHANGED_REFRESH_TASK_PRIORITY,        false,    false,    true,  0);
    add_task(uapi_transmitter_gps_settings_validation_task, UAPI_TRANSMITTER_GPS_CHANGED_REFRESH_TASK,                      false,    false,    true,  0);
    add_task(uapi_gps_settings_validation_task,             UAPI_GPS_SETTINGS_CHANGED_REFRESH_TASK_PRIORITY,                false,    false,    true,  0);
    add_task(uapi_tx_channel_update_task,                   UAPI_TRANSMITTER_CHANNEL_CHANGED_REFRESH_TASK_PRIORITY,         false,    false,    true,  0);
    add_task(app_track_settings_validation_task,            APP_TRACK_SETTINGS_CHANGED_REFRESH_TASK_PRIORITY,               false,    false,    true,  0);
    add_task(uapi_accelerometer_self_test_task,             UAPI_ACCELEROMETER_SELF_TEST_REFRESH_TASK,                      false,    false,    true,  0);
    add_task(uapi_power_settings_refresh,                   UAPI_POWER_SETTINGS_REFRESH_TASK,                               false,    false,    true,  0);
    add_task(uapi_gpio_settings_refresh_task,               UAPI_GPIO_SETTINGS_REFRESH_TASK_PRIORITY,                       false,    false,    true,  0);
    add_task(uapi_power_task,                               UAPI_POWER_TASK,                                                false,    false,    false, UAPI_POWER_TASK_PERIOD_MS);
    add_task(uapi_queue_msg_transmission_task,              UAPI_MSG_QUEUE_TRANSMIT,                                        false,    false,    false, UAPI_MSG_QUEUE_TASK_PERIOD_MS);
    add_task(uapi_ble_update_gstar_adv_data,                UAPI_BLE_ADV_UPDATE_TASK,                                       false,    false,    false, UAPI_BLE_ADV_UPDATE_TASK_PERIOD_MS);
    add_task(uapi_tx_device_esn_verify_task,                UAPI_TRANSMITTER_ESN_VERIFICATION,                              true,     false,    false, UAPI_ESN_VERIFY_PERIOS_MS);
    add_task(uapi_health_check_msg_task,                    UAPI_TX_HEALTH_CHECK_SCHEDULE_TASK,                             true,     false,    false, UAPI_HEALTH_CHECK_TASK_PERIOD_MS);
    add_task(uapi_health_msg_save_flash_task,               UAPI_TX_HEALTH_CHECK_SAVE_TO_FLASH_TASK,                        true,     false,    false, UAPI_HEALTH_CHECK_SAVE_PERIOD_MS);
    add_task(uapi_inputs_task,                              UAPI_INPUTS_TASK_PRIORITY,                                      true,     false,    false, UAPI_INPUTS_TASK_PERIOD_MS);
    add_task(uapi_inputs_settings_refresh_task,             UAPI_INPUTS_SETTINGS_REFRESH_TASK_PRIORITY,                     false,    false,    true,  0);
    add_task(t_print_task_stats,                            PRINT_TASK_STATS_TASK_PRIORITY,                                 true,     false,    false, PRINT_TASK_STATS_TASK_PERIOD_MS);
    add_task(rtc_stale_refresh_thread,                      RTC_STALE_REFRESH_TASK_PRIORITY,                                true,     false,    false, RTC_CHECK_PERIOD_MS);
    add_task(app_tod_interval_msg_task,                     APP_TOD_INTERVAL_MSG_SCHEDULING_TASK,                           false,    false,    false, APP_TOD_INTERVAL_MSG_SCHEDULING_PERIOD_MS);
    add_task(app_track_msg_schedule_task,                   APP_TRACK_MESSAGE_SCHEDULING_TASK,                              true,     false,    false, APP_TRACK_DEFAULT_PERIOD_MS);
    add_task(app_ota_events_periodic_task,                  APP_OTA_EVENTS_PERIODIC_TASK,                                   true,     false,    false, APP_OTA_EVENTS_PERIODIC_TASK_PERIOD_MS);
    add_task(app_ota_events_task,                           APP_OTA_EVENTS_TASK,                                            true,     false,    true,  0);
    add_task(app_power_task,                                APP_POWER_TASK,                                                 true,     false,    false,  APP_POWER_TASK_PERIOD_MS);
    add_task(uapi_led_blink_task,                           UAPI_LED_BLINK_TASK_PRIORITY,                                   false,    false,    false,  UAPI_LED_BLINK_PERIOD_MS);
    add_task(uapi_ext_flash_meta_data_read_task,            UAPI_READ_METADATA_LOGS_PRIORITY,                               false,    false,    false,  UAPI_LOGS_READ_PERIOD_MS);
    add_task(uapi_ext_flash_loc_logs_read_task,             UAPI_READ_LOC_LOGS_PRIORITY,                                    false,    false,    false,  UAPI_LOGS_READ_PERIOD_MS);
    add_task(uapi_ext_flash_debug_logs_read_task,           UAPI_READ_DEBUG_LOGS_PRIORITY,                                  false,    false,    false,  UAPI_LOGS_READ_PERIOD_MS);
    add_task(uapi_ext_erase_logs_task,                      UAPI_ERASE_LOGS_PRIORITY,                                       false,    false,    false,  UAPI_LOGS_READ_PERIOD_MS);
    add_task(app_tod_interval_read_task,                    UAPI_TOD_INTERVAL_READ_PRIORITY,                                false,    false,    false,  UAPI_LOGS_READ_PERIOD_MS);
  // clang-format on
  // UAPI Security Module
  uapi_security_init();   // Must occur before any xTaskCreate

  //****FreeRTOS Tasks****/
  // clang-format off
    xTaskCreate(at_handler_thread,  "ATHandler", AT_HANDLER_TASK_STACKS_SIZE, NULL, APP_AT_HANDLER_FREE_RTOS_THREAD_PRIOIRITY, &g_taskHandlerATHandler, 0);
    xTaskCreate(rr_scheduler_thread, "RRScheduler", RR_TASK_STACK_SIZE, NULL, APP_RRSCHEDULER_FREE_RTOS_THREAD_PRIORITY, &g_task_handle_micropython, 0);
    xTaskCreate(uapi_event_queue_callback_thread, "UAPIEventQueueCallback", EVENT_QUEUE_CB_TASK_STACK_SIZE, NULL, UAPI_EVENT_QUEUE_CALLBACK_THREAD_PRIORITY, &g_task_uapi_event_callback_task_handle, 0);
    xTaskCreate(uapi_config_thread, "Config", CONFIG_TASK_STACK_SIZE, NULL, CONFIG_FREE_RTOS_THREAD_PRIORITY, &g_task_handle_config, 0);
    xTaskCreate(gstar_lib_ble_nus_task,   "NUS Tx",   NUS_TX_TASK_STACK_SIZE,     NULL,    2,  &g_task_handle_nus,             GSTAR_SEC_POLICY_BASIC);
    xTaskCreate(uart_recieve_at_cmd_task, "UART AT",  UART_AT_TASK_STACK_SIZE,    NULL,    2,  &g_uart_at_cmd_task_handle,     GSTAR_SEC_THREAD_TYPE_UART);
    xTaskCreate(nus_recieve_at_cmd_task,  "NUS AT",   NUS_AT_TASK_STACK_SIZE,     NULL,    2,  &g_app_nus_at_cmd_task_handle,  GSTAR_SEC_THREAD_TYPE_NUS);
    xTaskCreate(gps_read_task, "GGPS", GPS_TASK_STACK_SIZE, NULL, UAPI_GPS_FREE_RTOS_THREAD_PRIORITY, NULL, GSTAR_SEC_POLICY_BASIC);
    xTaskCreate(asic_dfu_task, "ASICDFU", ASIC_DFU_TASK_STACK_SIZE, NULL, UAPI_ASIC_DFU_FREE_RTOS_THREAD_PRIORITY, &g_taskHandleASICDFU, GSTAR_SEC_POLICY_BASIC);
    xTaskCreate(config_service_data_handling_task, "CONFIGSERVICEDATAHANDLING", CONFIG_DATA_HANDLING_TASK_STACK_SIZE, NULL, UAPI_CONFIG_SERVICE_DATA_HANDLING_FREE_RTOS_THREAD_PRIORITY, &g_taskHandleconfigbledatahandling, GSTAR_SEC_THREAD_TYPE_BLE_CONFIG);
    xTaskCreate(accelerometer_isr_task, "AccelerometerISR", ACCELEROMETER_ISR_TASK_STACK_SIZE, NULL, GSTAR_LIB_ACCELEROMETER_ISR_FREE_RTOS_THREAD_PRIORITY, &g_hdn_accelerometer_isr_task, GSTAR_SEC_POLICY_BASIC);
    xTaskCreate(ble_task, "BLE COMMON TASK", BLE_COMMON_TASK_STACK_SIZE, NULL, UAPI_BLE_COMMON_FREE_RTOS_THREAD_PRIORITY, &g_taskhandleble, GSTAR_SEC_POLICY_BASIC);
  // clang-format on

#if NRF_LOG_ENABLED && NRF_LOG_DEFERRED
  xTaskCreate(gstar_log_task, "Log", LOG_TASK_STACK_SIZE, NULL,
    tskIDLE_PRIORITY + 1, &g_log_task_handle, GSTAR_SEC_POLICY_BASIC);
#endif

  if (g_device_type_properties.device_type !=
      GLOBALSTAR_DEVICE_TYPE_ST150_SLAP_N_TRACK)
    xTaskCreate(usbd_thread_task, "USBD", USBD_TASK_STACK_SIZE, NULL,
      GSTAR_LIB_USB_THREAD_PRIORITY, &g_app_usb_at_cmd_task_handle,
      GSTAR_SEC_THREAD_TYPE_USB);

  //****UAPI****/

  // UAPI transmitter module
  uapi_transmitter_init();
  tranmsit_message_queue_init();

  // UAPI Accel
  uapi_accelerometer_plat_init();

  // UAPI URC
  uapi_urc_init();

  // UAPI message module
  uapi_messages_init();

  uapi_gps_init();

  // UAPI GPIO module
  uapi_gpio_init();

  // UAPI Inputs module
  uapi_inputs_init();

  // UAPI Power
  uapi_power_init(NULL);

  // APP Power
  app_power_init();

  // APP TOD & Interval
  app_tod_interval_msg_init();

  // Initize the track module
  app_track_msg_init();

  // OTA events
  app_ota_event_init();

  // Crypto
  if (err_code = nrf_crypto_init())
    APP_ERROR_CHECK(err_code);

  // ** Must be after crypto init
  // Peripherals (Configured UART2)
  uapi_gpio_set_uart2_with_config_settings();

  // BLE
  if (ble_init())
    APP_ERROR_CHECK(1);
  // Any error can be ignored during
  // init, writes will handle errors
  // accordingly
  get_ext_flash_memory_current_pointers();

  // Enable stream mode in Tracelyzer
  vTraceEnable(TRC_INIT);

  // Enable Deep Sleep Mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  // Dump Remaining Heap For Debugging Purposes
  GSTAR_LOG_INFO("FreeRTOS has %d bytes of heap remaining.",
    xPortGetFreeHeapSize());

  // Start FreeRTOS scheduler.
  vTaskStartScheduler();

  // Execution shall never reach this while loop
  while (1)
  {
    APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
  }
}
