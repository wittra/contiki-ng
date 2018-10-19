/*---------------------------------------------------------------------------*/
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
#define RF_BLE_CONF_ENABLED                   0
/*---------------------------------------------------------------------------*/
#define ENERGEST_CONF_ON                      0
/*---------------------------------------------------------------------------*/
#define WAKE_ON_MOTION_ENABLED                0
/*---------------------------------------------------------------------------*/
/* Enable the ROM bootloader */
//#define ROM_BOOTLOADER_ENABLE                 1
/*---------------------------------------------------------------------------*/
/*
 * Shrink the size of the uIP buffer, routing table and ND cache.
 * Set the TCP MSS
 */
//#define UIP_CONF_BUFFER_SIZE                900
//#define NETSTCK_ROUTING_STATE_SIZE            5
//#define NBR_TABLE_CONF_MAX_NEIGHBORS         30
//#define NETSTACK_MAX_ROUTE_ENTRIES           30
//#define UIP_CONF_TCP_MSS                    128
/*---------------------------------------------------------------------------*/
/*******************************************************/
/******************* Configure TSCH ********************/
/*******************************************************/

/* IEEE802.15.4 PANID */
#define IEEE802154_CONF_PANID 0xbedf    //0x7327//0x81a5

#ifdef MAC_CONF_WITH_TSCH
/* Start TSCH at init */
#define TSCH_CONF_AUTOSTART 1

/* 6TiSCH hopping sequence*/
//#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 8 }    //(uint8_t[]){ 16, 23, 18, 22 }

/* 6TiSCH minimal schedule length.
 * Larger values result in less frequent active slots: reduces capacity and saves energy. */
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH 3

/* Configuration for TSCH with prop mode driver */
#define TSCH_CONF_DEFAULT_TIMESLOT_LENGTH  40000U
#define TSCH_CONF_RX_WAIT                  1200
/* The data rate is 50 kbps = 6250 bytes per second. */
#define TSCH_CONF_BYTE_DURATION_US (1000000 / 6250)
#else
#define RF_CHANNEL 17
#endif
/*---------------------------------------------------------------------------*/
/* Logging */
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_INFO /* Only for rpl-lite */
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_NULLNET                     LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_6TOP                        LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_COAP                        LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_LWM2M                       LOG_LEVEL_NONE
#define LOG_CONF_LEVEL_MAIN                        LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_APP                         LOG_LEVEL_INFO
#if MAC_CONF_WITH_TSCH == 1
#define TSCH_LOG_CONF_PER_SLOT		    1
#endif
/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
