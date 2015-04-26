/********************************************************************
* Description: hal_zed_can.c 
* Driver for the 2FOC controller RT-CAN communication using CAN8 
* interface board on the ZedBoard platform.
*
* \author Claudio Lorini (claudio.lorini@iit.it)
* License: GPL Version 2
* Copyright (c) 2015.
*
* Some code parts and profund inspiration taken from:
* - the hal_gpio.c by:
*   Author: Michael Haberler
* - the bcm2835 library by:
*   Author: Mike McCauley
* - the stepgen.c component:
*   Author: John Kasunich
*
* The rt-can communication is derived from Xenomai
* - rtcansend and rtcanrcv
*   Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
*
********************************************************************/

/**
 \brief Driver for the 2FOC controller RT-CAN communication using CAN8
 interface board on the ZedBoard platform.

\code
    2FOC component
    +---------------------------+
    |                           |
 ---|> EStop      Feedback[32] >|---
 ---|> DriveEn      Status[32] >|---
 ---|> SetPoint[32]  Error[32] >|---
    |                           |
    |  CAN_Adx                  |
    +---------------------------+

 \details
  This module manages the RT-CAN communication with the 2FOC board and hinder 
  the communication protocol details to the casual user...

  The number FOC axis controlled is determined  by the insmod command 
  line parameter 'FOC_axis' passed from .hal configuration file. 
  It accepts a comma separated (no spaces) list of up to 8 numbers 
  indicating the CAN address of the FOC channel.

  In order to use this driver the 2FOC boards must be configured for 
  speed control, CAN address 3, connecting each axis on a different CAN8 
  line in a point2point configuration.

  2FOC periodic message must be configured for: 
   - position(32bit)
   [- velocity(16bit)]
   [- Iq(16bit)      ]

  \par DS402 state machine

    init CAN
    test EStop
    if estop pressed send 'Shutdown' goto test EStop
    send 'SwithOn' 
    test if 'SwitchedOn'  
    if not 'SwitchedOn' goto test 'SwitchedOn' 
    test if DriveEn 
    if not DriveEn got test DriveEn
    Send 'EnableOp'
    test for 'RotorAligned'
    if not 'RotorAligned' goto test 'RotorAligned'
    start control

  \par Revision history:
  \date 11.03.2014 started development from zeth.c files
  \date 21.03.2014 2FOC feedback, status and error parser
  \date 23.03.2014 extension to multichannel and parametrization
  \date 12.03.2015 ported to machinekit
  \date 20.04.2015 Started porting to rt-can communication

 \version 01
*/

#include "rtapi.h"         // rtapi_print_msg()
#include "rtapi_bitops.h"  // RTAPI_BIT(n)    
#include "rtapi_app.h"
#include "hal.h"

#if !defined(BUILD_SYS_USER_DSO) 
    #error "This driver is for usermode threads only"
#endif
#if !defined(TARGET_PLATFORM_ZEDBOARD)
    #error "This driver is for the Zedboard platform only"
#endif
/**
 \todo Check that the driver is built only when Xenomai threads are configured 
*/

#include <unistd.h>

#include <unistd.h>

#include <xenomai/rtdm/rtcan.h>

#include "2FOC_status.h"

/** \brief maximum number of FOC channels */
#define MAX_FOC_CHAN 8

MODULE_AUTHOR("Claudio Lorini");
MODULE_DESCRIPTION("RT-CAN Driver for 2FOC controller.");
MODULE_LICENSE("GPL");

// RT component ID
static int comp_id;

// 2FOC mirror data
typedef struct {
    //
    // Input pins
    //
    // emergency stop input
    hal_bit_t   *estop;
    // driver enable input
    hal_bit_t   *driven;
    // speed setpoint input
    hal_s32_t   *setpoint;

    //
    // Output pins
    //
    // image of position feedback
    hal_s32_t   *feedback;
    // image of status of 2FOC
    hal_u32_t   *focstatus;
    // image of error of 2FOC
    hal_u32_t   *focerror;

    //
    // params
    //
    
    
} FOC_data_t;

/** \brief Array of FOC_data structs in shared memory, 1 per configured axis */
static FOC_data_t * FOC_data_array;

// socket handles
int sock[MAX_FOC_CHAN] = { [0 ... MAX_FOC_CHAN-1] = -1 };

/** \brief number of FOC channels configured */
static int num_chan = 0;

/** \brief Number and CAN address of active FOC axis */
int FOC_adx[] = { [0 ... MAX_FOC_CHAN-1] = -1 };
RTAPI_MP_ARRAY_INT(FOC_adx,MAX_FOC_CHAN,"CAN address for up to 8 FOC channels");

// Check if the FOC axis is in 'operation enable' status
bool FOCAxisIsOperationEnable[MAX_FOC_CHAN]={false};

/**
 \brief Send rtcan messages to 2FOC
 \pre  */
static void rtcan_send(void *arg, long period)
{

}

/**
 \brief Read rtcan messages from 2FOC
 \pre  */
static void rtcan_receive(void *arg, long period)
{

}

/** 
 \brief Export IO pins and parameters for component
 \param 
   num:  component number 
   addr: pointer to array of the num^th FOC channel data */
static int exportFOCaxis(int num, FOC_data_t * addr)
{    
    int retval = 0;

    // I PINS
    // make available Emergency stop in hal
    if ( (hal_pin_bit_newf(HAL_IN,&(addr->estop), comp_id, "hal_zed_can.%d.estop", num) ) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: pin estop export failed with err=%d", retval);
        hal_exit(comp_id);
        return -1;
    }
    // Drive Enable
    if ( (hal_pin_bit_newf(HAL_IN,&(addr->driven), comp_id, "hal_zed_can.%d.driven", num) ) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: pin driven export failed with err=%d", retval);
        hal_exit(comp_id);
        return -1;
    }
    // setpoint
    if ( (retval = hal_pin_s32_newf(HAL_IN, &(addr->setpoint), comp_id, "hal_zed_can.%d.setpoint", num) ) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: pin setpoint export failed with err=%d", retval);
        hal_exit(comp_id);
        return -1;
    } 

    // O PINS
    // position feedback
    if ( (retval = hal_pin_s32_newf(HAL_OUT, &(addr->feedback), comp_id, "hal_zed_can.%d.feedback", num) ) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: pin feedback export failed with err=%d", retval);
        hal_exit(comp_id);
        return -1;
    }
    // 2FOC Status 
    retval = hal_pin_u32_newf(HAL_OUT, &(addr->focstatus), comp_id, "hal_zed_can.%d.status", num);
    // check for failed debug space mapping
    if(retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: pin status export failed with err=%d" ,retval);
        hal_exit(comp_id);
        return -1;
    }
    // 2FOC Errors
    retval = hal_pin_u32_newf(HAL_OUT, &(addr->focerror), comp_id, "hal_zed_can.%d.error", num);
    // check for failed debug space mapping
    if(retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: param error export failed with err=%d",retval);
        hal_exit(comp_id);
        return -1;
    }

    // PARAMS

    // completed successfully
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZED_CAN: exportFOCaxis(%d) completed successfully.\n", num);
    return 0;
}

/**
 \brief main realtime task */
int rtapi_app_main(void)
{
    int ret;

    // zynq and FPGA code revision
    int n;
    // save messaging level
    static int msg_level;
    int retval = 0;
    
    // save message level on entering 
    msg_level = rtapi_get_msg_level();
    
    /* setup messaging level in:
    RTAPI_MSG_NONE, RTAPI_MSG_ERR, RTAPI_MSG_WARN,
    RTAPI_MSG_INFO, RTAPI_MSG_DBG, RTAPI_MSG_ALL */
    rtapi_set_msg_level(RTAPI_MSG_ALL);

    // parse module parameters in order to find the number
    // of configured FOC channels and their CAN address
    for(n = 0; n < MAX_FOC_CHAN && FOC_adx[n] != -1 ; n++) {
        // check for a valid CAN address 
        if( (FOC_adx[n] <= 0) || ( FOC_adx[n] > 15) ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: bad CAN address '%i', axis %i", FOC_adx[n], n);
            hal_exit(comp_id);
            return -1;
        }
        // found a correctly configured channel 
        num_chan++;
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZED_CAN: FOC axis %d with CAN address %d.",n, FOC_adx[n] );
    }
    if( (0 == num_chan) || (8 < num_chan) ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: channels configured incorrectly.");
        hal_exit(comp_id);
        return -1;
    }

    // allocate shared memory for FOC_data of each axis
    FOC_data_array = hal_malloc(num_chan * sizeof(FOC_data_t));
    if ( 0 == FOC_data_array ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: hal_malloc() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    // try to init the component
    comp_id = hal_init("hal_zed_can");
    if( comp_id < 0 ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: hal_init() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    /* Export the variables/parameters for each FOC axis */
    for (n = 0; n < num_chan; n++) {
        // export pins/params for the n^th component
        retval = exportFOCaxis(n, &(FOC_data_array[n]) );
        if(  retval < 0 ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: exportFOCaxis() failed");
            hal_exit(comp_id);
            return -1;
        }
    }

    // export the send/receive rtcan messages
    retval = hal_export_funct("hal_zed_can.rtcan_send", rtcan_send, FOC_data_array, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: write funct export failed\n");
        hal_exit(comp_id);
        return -1;
    }
    retval = hal_export_funct("hal_zed_can.rtcan_receive", rtcan_receive, FOC_data_array, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: read funct export failed\n");
        hal_exit(comp_id);
        return -1;
    }

    // init RT-CAN
    // create sockets
    for (n = 0; n < num_chan; n++) {
        ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (ret < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZED_CAN: ERROR: rt_dev_socket: %s\n", strerror(-ret));
            hal_exit(comp_id);
            return -1;
        }
        sock[n] = ret;
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZED_CAN: RT-CAN sockets from %d to %d created successfully.\n",sock[0], sock[num_chan-1] );

    // all operations succeded
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZED_CAN: driver installed successfully.\n");
    hal_ready(comp_id);

    // return to previous mesaging level
    rtapi_set_msg_level(msg_level);

    return 0;
}

/** 
 \brief Exit component closing communication with EMS 
 \pre */
void rtapi_app_exit(void)
{ 
    // notify clean termination
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZED_CAN: component terminated successfully \n");
   
    hal_exit(comp_id);
}

