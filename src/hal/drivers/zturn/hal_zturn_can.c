/********************************************************************
* Description: hal_zturn_can.c
* Driver for the 2FOC controller using IC84M 
* interface board on the MYIR ZTurn platform.
*
* \author Claudio Lorini (claudio.lorini@iit.it)
* License: GPL Version 2
*
********************************************************************/

/**
 \brief Driver for the 2FOC controller using IC84M
 interface board on the ZTurn platform.

 \code
    2FOC component
    +---------------------------+
    |                           |
 ---|> EStop      Feedback[32] >|---
 ---|> DriveEn      Status[32] >|---
 ---|> SetPoint[32]  Error[32] >|---
    | parameters:               |
    | CAN_Adx, can_ifn          |
    +---------------------------+

 \details
  This module manages the CAN communication with the 2FOC board and hinder 
  the communication protocol details to the casual user...

  The position loop is closed in MK sending to the 2FOC speed setpoints 
  and receiving back the current position of the motor.
  In order to close the loop with a reasonable control quality only one
  2FOC controller is connected on each CAN line, that permits sub-millisecond
  loop speed without saturating the CAN lines.

  The number FOC axis controlled is determined  by the insmod command 
  line parameter 'FOC_axis' passed from .hal configuration file. 
  It accepts a comma separated (no spaces) list of up to 8 numbers:
  - can_ifn: number of the CAN interface(s) to use
  - CAN_Adx: the CAN address of 2FOC board connected to the channel.

  In order to use this driver the 2FOC boards must be configured for 
  speed control, CAN address 3, connecting each axis on a different CAN8 
  line in a point2point configuration.
  2FOC periodic message must be configured for: 
   [ position(32bit) ]
   [-1 = none ]
   [-1 = none ]

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
 \date 10.12.2016 started development from hal_turn_can.c files

 \version 01
*/

#include "rtapi.h"         // rtapi_print_msg()
#include "rtapi_bitops.h"  // RTAPI_BIT(n)    
#include "rtapi_app.h"
#include "hal.h"

#if !defined(TARGET_PLATFORM_ZTURN)
  #error "This driver is for the Z-Turn platform only."
#endif

// check if enabled xenomai support
#if !defined(HAVE_RT_PREEMPT_THREADS) 
  #error "This driver is for rt-preempt only."
#endif

#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "2FOC_status.h"

/** \brief maximum number of FOC channels */
#define MAX_FOC_CHAN 8

MODULE_AUTHOR("Claudio Lorini");
MODULE_DESCRIPTION("CAN Driver for 2FOC controller.");
MODULE_LICENSE("GPL");

// RT component ID
static int comp_id;


// 2FOC mirror data
/** \brief this data is continuously updated with the contents of the CAN status 
and periodic messages; setpoint data coming from MK is periodically sent to 
the 2FOC boards*/  
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

// socket handles (for both tx and rx)
int sock[MAX_FOC_CHAN] = { [0 ... MAX_FOC_CHAN-1] = -1 };

// CAN frames for each line
static struct can_frame txframe[MAX_FOC_CHAN];
static struct can_frame rxframe[MAX_FOC_CHAN];

/** \brief number of FOC channels configured */
static int num_chan = 0;

/** \brief rt-can interface number  */
int can_ifn[] = { [0 ... MAX_FOC_CHAN-1] = -1 };
RTAPI_MP_ARRAY_INT(can_ifn,MAX_FOC_CHAN,"RT-CAN channel number for up to 8 lines");

/** \brief CAN filters and masks */
struct can_filter rxfilter[MAX_FOC_CHAN][16];

/** \brief CAN receive threads */
static pthread_t rxthread[MAX_FOC_CHAN];

// to identify can rx thread
int nthread[MAX_FOC_CHAN];

// Rx threads stuff
pthread_attr_t thattr[MAX_FOC_CHAN];
// struct sched_param param[MAX_FOC_CHAN] = { [0 ... MAX_FOC_CHAN-1].sched_priority = 1 };
struct sched_param param[MAX_FOC_CHAN] = { [0 ... MAX_FOC_CHAN-1].sched_priority = 82 };

// DS402 State Machine
typedef enum {SHUTTED_DOWN=1, SWITCHED_ON, OPERATION_ENABLED, PERIODIC, IRA_IN_PROGRESS} tDS402;
tDS402 status[MAX_FOC_CHAN]={ [0 ... MAX_FOC_CHAN-1] = SHUTTED_DOWN };

// override MK messaging level to give a lot of debug messagers without much bloating 
// from other modules.
#define OVERRIDE_MESSAGING_LEVEL 1

/**
 \brief  Parse incoming CAN messages from 2FOC controller
 \params rxframe: message to parse
         n: CAN channel
 \return 0 everything OK
        -1    */
int ParseMessage(struct can_frame *frame, int n, int *ack, int *nack)
{
    // parse packet type, mask away CAN address
    switch( frame->can_id ) {
        
        case 0x83FF1004:
        // process position reported as reply to setpoint
        {   
             // check size of the message
            if( 4 < frame->can_dlc ) {
                // rx periodic! grab data (position).
                *(FOC_data_array[n].feedback) = *(hal_s32_t*) frame->data;
            }
            else {
                rtapi_print_msg(RTAPI_MSG_ERR,"HAL_ZTURN_CAN: ERROR: received periodic with wrong lenght (%d) from CAN%d",
                    frame->can_dlc, n);
            }
        }
        break;

        case 0x83FF02FF: 
        // status error (periodic)
        case 0x93FF0117:
        // answer to "Get fault and status info" command
        {
            // check correct size of the message
            if( 8 == frame->can_dlc ) {
                *(FOC_data_array[n].focstatus) = *(hal_u32_t*) (frame->data);
                *(FOC_data_array[n].focerror)  = *(hal_u32_t*) (frame->data+4);
            }
            else {
                rtapi_print_msg(RTAPI_MSG_ERR,"HAL_ZTURN_CAN: ERROR: received sratus/error with wrong lenght (%d) from CAN%d",
                    frame->can_dlc, n);
            }
        }
        break;

        case 0x93FF200E: // TWOFOC_SWITCH_ON
        case 0x93FF200F: // TWOFOC_ENABLE_OPERATION
        case 0x93FF2010: // TWOFOC_DISABLE_OPERATION
        case 0x93FF2011: // TWOFOC_SHUT_DOWN
        case 0x93FF2004: // TWOFOC_SPEED_SETPOINT
        case 0x93FF2017: // TWOFOC_GET_STATUSERROR
        case 0x93FF201E: // TWOFOC_ZERO_AXIS
            // oh, it's just an ack to some boring command...
            // rtapi_print_msg(RTAPI_MSG_ERR,"HAL_ZTURN_CAN: ACK (0x%X) from CAN%d",frame[n].can_id, n);
            *ack = 1;
        break;

        // NACK on this commands could happen, do not bother...
        case 0x93FF300E: // TWOFOC_SWITCH_ON
        case 0x93FF300F: // TWOFOC_ENABLE_OPERATION
        case 0x93FF3010: // TWOFOC_DISABLE_OPERATION
        case 0x93FF3011: // TWOFOC_SHUT_DOWN
            *nack = 1;
        break;

        // NACK on this commands should not happen... better to report user!
        case 0x93FF3004: // TWOFOC_SPEED_SETPOINT
        case 0x93FF3017: // TWOFOC_GET_STATUSERROR
        case 0x93FF301E: // TWOFOC_ZERO_AXIS
            // ok, this is a command NACK, something serious has happened, 
            // like a set-point overrange or such... or maybe is only a mesage sent in 
            // a wrong status such as shutdown when already shutted down.
            // \todo try to mend it.
            rtapi_print_msg(RTAPI_MSG_ERR,"HAL_ZTURN_CAN: NACK (0x%X) from CAN%d",frame[n].can_id, n);
            *nack = 1;
            // hal_exit(comp_id);
            // return (-1);
        break;

        default:
            // aliens incoming!
            rtapi_print_msg(RTAPI_MSG_ERR,"HAL_ZTURN_CAN: Error: unexpected message received 0x%X form CAN%d",frame->can_id, n);
            // hal_exit(comp_id);
            // return (-1);
        break;
    }

    return (0);
}

/**
 \brief *MULTIPLE*THREADS*, receive and parse rtcan messages from 2FOC
 \details This task loops continuously waiting for incoming CAN messages
 (using a non blocking rt_dev_recvfrom) each time a packet is received 
 is parsed and the global status/error/data is updatedd
 \pre  
 \param *arg pointer to channel number */
void *rtcan_rx_and_parse(void *arg)
{
    int retval=0, ack=0, nack=0;
    int n = *(int*)arg;

    // change thread priority
    retval = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param[n]);
    if (retval != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: %s: set thread scheduling parameters failed\n", strerror(-retval));
        return NULL;
    }

    while (1) { 
        // receive CAN messages (sock0)
///        retval = rt_dev_recv(sock[n], (void *)&(rxframe[n]), sizeof(can_frame_t), 0);

/// TO BE DONE 
/// TO BE DONE 
/// TO BE DONE 
/// TO BE DONE 
/// TO BE DONE 

        if (retval != sizeof(rxframe)) {
            switch (retval) {
            case -ETIMEDOUT:
                rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: rt_dev_recvfrom CAN%d timed out", n);
            break;
            case -EBADF:
                rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_recvfrom socket %d was closed", n);
                return NULL;
            break;
            case 0:
                rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_recvfrom received zero bytes");
                return NULL;
            break;
            default:
                rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_recvfrom CAN%d %s <-- \n", n, strerror(-retval));
                return NULL;
            } 
        }
        else {
            // parse message received, retval contains the number of bytes received
            ParseMessage(&(rxframe[n]), n, &ack, &nack);
        }
    }
}

/**
 \brief Send a single CAN message to 2FOC
 \param 
    n CAN cannel to send message to
 \pre fill frame with the message */
int can_send(int n)
{
    int retval=0;

    // send frame
///    retval = rt_dev_send(sock[n], (void *)&(txframe[n]), sizeof(can_frame_t), 0);

/// TO BE DONE 
/// TO BE DONE 
/// TO BE DONE 
/// TO BE DONE 
/// TO BE DONE 

    if (retval < 0) {
       switch (retval) {
           case -ETIMEDOUT:
               rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_send to can%d : timed out\n", n);
           break;
           case -EBADF:
               rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_send to can %d: aborted because socket was closed\n", n);
           break;
           default:
               rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_send to can %d, %s\n",n , strerror(-retval) );
           break;
        }
    }

    return retval;
}

/** 2FOC CAN commands ( \note FIXED ADX=3)! */
#define TWOFOC_SWITCH_ON            ((0x03FFFF0E & ~CAN_RTR_FLAG) | CAN_EFF_FLAG)
#define TWOFOC_ENABLE_OPERATION     ((0x03FFFF0F & ~CAN_RTR_FLAG) | CAN_EFF_FLAG)
#define TWOFOC_DISABLE_OPERATION    ((0x03FFFF10 & ~CAN_RTR_FLAG) | CAN_EFF_FLAG)
#define TWOFOC_SHUT_DOWN            ((0x03FFFF11 & ~CAN_RTR_FLAG) | CAN_EFF_FLAG)
#define TWOFOC_SPEED_SETPOINT       ((0x03FFFF04 & ~CAN_RTR_FLAG) | CAN_EFF_FLAG)
#define TWOFOC_GET_STATUSERROR      ((0x03FFFF17 & ~CAN_RTR_FLAG) | CAN_EFF_FLAG)
#define TWOFOC_ZERO_AXIS            ((0x03FFFF1E & ~CAN_RTR_FLAG) | CAN_EFF_FLAG)

/** 
 \brief Fill frame data for "Zero Axis" command */
int zero_axis(int n)
{
    txframe[n].can_id  = TWOFOC_ZERO_AXIS;
    // payload lenght
    txframe[n].can_dlc = 0;

    return can_send(n);
}

/** 
 \brief Fill frame data for "Switch On" command */
int switch_on(int n)
{
    txframe[n].can_id  = TWOFOC_SWITCH_ON;
    // payload lenght
    txframe[n].can_dlc = 0;

    return can_send(n);
}

/** 
 \brief Fill frame data for "Enable Operation" command */
int enable_operation(int n)
{
    txframe[n].can_id  = TWOFOC_ENABLE_OPERATION;
    // payload lenght
    txframe[n].can_dlc = 0;

    return can_send(n);
}

/** 
 \brief Fill frame data for "Disable Operation" command */
int disable_operation(int n)
{
    txframe[n].can_id  = TWOFOC_DISABLE_OPERATION;
    // payload lenght
    txframe[n].can_dlc = 0;

    return can_send(n);
}

/** 
 \brief Fill frame data for "Shut Down" command */
int shut_down(int n)
{
    txframe[n].can_id  = TWOFOC_SHUT_DOWN;
    // payload lenght
    txframe[n].can_dlc = 0;

    return can_send(n);
}

/** 
 \brief Fill frame data for "Get fault and status info" command */
int statuserror(int n)
{
    txframe[n].can_id  = TWOFOC_GET_STATUSERROR;
    // payload lenght
    txframe[n].can_dlc = 0;

    return can_send(n);
}

/** 
 \brief Fill frame data for "Periodic" command */
void periodic(int n)
{
    __s16 setpoint;

    // speed setpoint command
    txframe[n].can_id  = TWOFOC_SPEED_SETPOINT;

    // prepare packet lenght for speed setpoint
    txframe[n].can_dlc = 2;

    // fill payload with pin Setpoint data
    // payload = speed in mm/sec
    setpoint = *(FOC_data_array[n].setpoint);
    memcpy(txframe[n].data, &setpoint, sizeof(__s16));
}

// send a null setpoint
void sendnullsetpoint(int n)
{
    __s16 setpoint = 0;

    // speed setpoint command
    txframe[n].can_id  = TWOFOC_SPEED_SETPOINT;

    // prepare packet lenght for speed setpoint
    txframe[n].can_dlc = 2;

    // fill payload with pin Setpoint data
    // payload = speed in mm/sec
    memcpy(txframe[n].data, &setpoint, sizeof(__s16));

    can_send(n);
}

/** \brief tell if 2FOC has completed initial rotor aligment*/
int IRAcompleted(int n)
{
    if( 0 == (0x00020000 & ( *(FOC_data_array[n].focstatus)))) {
        return 0;
    }
    else {
        return 1;
    }
}



/**

      'sta roba mi pare 'na cagata, 
      fare una procedura più acconcia per lo switchon ecc.ecc.


 \brief *TIMED*PERIODIC*, Send rtcan messages (setpoints) to 2FOC periodically
 \details If "e-stop" pin is active the shutdown command is issued on each axis,
   when e-stop is released the "switch on" command is issued.
   When the pin "drive enable" if activated the command "enable operation" is issued.
   If the driver has reached "Switched on" status the "periodic" command is issued.

   For detailed description see 2FOC_SM.dia
 \pre  */
// \todo vedere se sono giusti i parametri e forse si possono togliere?
static void rtcan_periodic_send(void *arg, long period)
{
    int n;

    for (n = 0; n < num_chan; n++) {
        // DS402 state machine
        switch (status[n]) {
            case SHUTTED_DOWN:
                shut_down(n);

                if(0 == *(FOC_data_array[n].estop) ) {
                    status[n]=SWITCHED_ON;
                }
            break;

            case SWITCHED_ON:
                switch_on(n);

                if(1 == *(FOC_data_array[n].estop) ) {
                    status[n]=SHUTTED_DOWN;
                }
                else {
                    if(1 == *(FOC_data_array[n].driven) ) {
                        status[n]=OPERATION_ENABLED;
                    }
                }
            break;

            case OPERATION_ENABLED:
                enable_operation(n);

                if(1 == *(FOC_data_array[n].estop) ) {
                    status[n]=SHUTTED_DOWN;
                }
                else {
                    if(0 == *(FOC_data_array[n].driven) ) {
                        status[n]=SWITCHED_ON;
                    }
                    else {
                        if(1 == IRAcompleted(n) ) {
                            status[n]=PERIODIC;
                        } 
                        else  {
                            status[n]=IRA_IN_PROGRESS;
                        }
                    }
                }
            break;

            case IRA_IN_PROGRESS:
                if(1 == *(FOC_data_array[n].estop) ) {
                    status[n]=SHUTTED_DOWN;
                }
                else {
                    if(0 == *(FOC_data_array[n].driven) ) {
                        status[n]=SWITCHED_ON;
                    }
                    else {
                        if(1 == IRAcompleted(n) ) {
                            zero_axis(n);
                            status[n]=PERIODIC;
                        }
                    }
                }
            break;

            case PERIODIC:
                periodic(n);
                can_send(n);

                if(1 == *(FOC_data_array[n].estop) ) {
                    status[n]=SHUTTED_DOWN;
                }
                else {
                    if(0 == *(FOC_data_array[n].driven) ) {
                        status[n]=SWITCHED_ON;
                        sendnullsetpoint(n);
                    }
                }
            break;

            default:
                rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: SM Fu*k-up!.");
            break;
        }
    }
}

/** \brief send initial rotor alignment for each channel */
int sendIRA()
{
    int n, retval=0;
    
    // send commands for rotor alignment
    for (n = 0; n < num_chan; n++) {
        // if on turn it down
        retval = shut_down(n); 
        retval = switch_on(n);
        // todo: check return values
        retval = enable_operation(n);
    }

    // wait for alignment is done in the SM

    return retval;
}

/** 
 \brief export module functions
 \return 
   -1         in case of error 
    0         everithing ok */
static int export_functions() 
{
    int retval=0;

    // export the send/receive rtcan messages
    retval = hal_export_funct("hal_turn_can.rtcan_periodic_send", rtcan_periodic_send, FOC_data_array, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rtcan_periodic_send funct export failed\n");
        return -1;
    }

    return retval;
}

/** 
 \brief Parse module parameters: CAN interface number and the CAN address of the 2FOC connected (p2p)
   Initialize the number of correctly parametrized channels 
 \return 
   -1         in case of error 
    0         everithing ok 
    num_chan  number of configured channels */
static int Parse_Module_Parameters()
{    
    int n;

    for(n = 0; (n < MAX_FOC_CHAN) && (can_ifn[n] != -1) ; n++) {
        // check for a valid rtcan interface number (0..7) 
        if( (can_ifn[n] < 0) || ( can_ifn[n] > 7) ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: bad rtcan interface number %i", can_ifn[n]);
            return -1;
        }
        // found a correctly configured channel 
        num_chan++;
        // report interface number and the connected 2FOC can address
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: FOChal_exit axis %d @ rtcan%d interface.",
            n, can_ifn[n] );
    }
    if( (0 == num_chan) || (8 < num_chan) ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: invalid number of channels: %d.", num_chan);
        return -1;
    }
    // ok.
    return 0;
}

/** 
 \brief Export IO pins and parameters for component
 \pre    Parse_Module_Parameters()
 \return -1 in case of error  */
static int exportFOCaxis()
{    
    int retval = 0;
    int num;

    for ( num = 0; num < num_chan; num++) {

        // I PINS
        // make available Emergency stop in hal
        if ( (retval = hal_pin_bit_newf(HAL_IN,&(FOC_data_array[num].estop), comp_id, "hal_turn_can.%d.estop", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin estop export failed with err=%d", retval);
        }
        // Drive Enable
        if ( (retval = hal_pin_bit_newf(HAL_IN,&(FOC_data_array[num].driven), comp_id, "hal_turn_can.%d.driven", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin driven export failed with err=%d", retval);
        }
        // setpoint
        if ( (retval = hal_pin_s32_newf(HAL_IN, &(FOC_data_array[num].setpoint), comp_id, "hal_turn_can.%d.setpoint", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin setpoint export failed with err=%d", retval);
        }

        // O PINS
        // position feedback
        if ( (retval = hal_pin_s32_newf(HAL_OUT, &(FOC_data_array[num].feedback), comp_id, "hal_turn_can.%d.feedback", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin feedback export failed with err=%d", retval);
        }
        // 2FOC Status
        if( (retval = hal_pin_u32_newf(HAL_OUT, &(FOC_data_array[num].focstatus), comp_id, "hal_turn_can.%d.status", num) != 0) ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin status export failed with err=%d" ,retval);
        }
        // 2FOC Errors
        if( (retval = hal_pin_u32_newf(HAL_OUT, &(FOC_data_array[num].focerror), comp_id, "hal_turn_can.%d.error", num) != 0) ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: param error export failed with err=%d",retval);
        }

        // zero data
        *(FOC_data_array[num].focstatus) = 0;
        *(FOC_data_array[num].focerror)  = 0;
        *(FOC_data_array[num].feedback)  = 0;

        // PARAMS
    }

    // completed successfully
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: exportFOCaxis(%d) completed successfully.\n", num);
    return retval;
}

/**
 \brief Setup CAN Rx masks and filters (up to 16 available)
 \param 
   n: channel
 \pre    Parse_Module_Parameters()
 \return 0 if ok
        -1 in case of error
 \note   during debug phase all filters are nulled to let pass any CAN packet so alien detection can be 
         done. Once we're sure everytning is known packet filtering can be set-up (or not). */
static int SetupCANFilterMask(int n)
{
    int filter_count = 5;

    // Setup Rx filter/mask
    rxfilter[n][0].can_id   = 0x03FF01FF; // process data (periodic)
    rxfilter[n][0].can_mask = 0x1FFFFFFF;

    rxfilter[n][1].can_id   = 0x03FF02FF; // status/error (periodic)
    rxfilter[n][1].can_mask = 0x1FFFFFFF;

    rxfilter[n][4].can_id   = 0x03FF1000; // answer to any command that request data
    rxfilter[n][4].can_mask = 0x1FFFFF00;

    rxfilter[n][2].can_id   = 0x13FF2000; // ACK to any command
    rxfilter[n][2].can_mask = 0x0FFFFF00;
   
    rxfilter[n][3].can_id   = 0x13FF3000; // NACK to any command
    rxfilter[n][3].can_mask = 0x1FFFFF00;

    return setsockopt(sock[n], SOL_CAN_RAW, CAN_RAW_FILTER, &(rxfilter[n]),
      filter_count * sizeof(struct can_filter));
}

/**
 \brief init rtcan, setup sockets, interfaces, rx/tx timeouts and CAN filters/masks
 \param 
 \pre    Parse_Module_Parameters()
 \return 0 if ok
        -1 in case of error
 \note   */
static int init_rtcan()
{
    int n;
    int retval = 0;
    // TxTimeout = 100msec,  RxTimeout = 1000msec
    //nanosecs_rel_t txto = 100000000, rxto = 1000000000;
    //
    struct ifreq ifr[MAX_FOC_CHAN];
    // tx/rx socket addresses
    static struct sockaddr_can rtx_addr[MAX_FOC_CHAN];

    for (n = 0; n < num_chan; n++) {

        // create sockets
        retval = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_socket: %s\n", strerror(-retval));
            return -1;
        }
        sock[n] = retval;

        // compose the name of the interface
        sprintf(ifr[n].ifr_name,"rtcan%d",can_ifn[n]); 
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: sock=%d, ifr_name=%s\n", sock[n], ifr[n].ifr_name);

        // Get interface index (ifr.ifr_ifindex) by given name (ifr.ifr_name)
        retval = ioctl(sock[n], SIOCGIFINDEX, &ifr[n]);
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_ioctl: %s\n", strerror(-retval));
            return -1;
        }
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: interface %s is @ index %d\n",ifr[n].ifr_name, ifr[n].ifr_ifindex );

        // Setup Rx filter/mask for channel n
        retval = SetupCANFilterMask(n);
        if ( retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: RX setsockopt CAN_RAW_FILTER failed: %s\n", strerror(-retval));
            return -1;
        }
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: filters set up successfull.\n");

        // bind sockets
        memset(&rtx_addr[n], 0, sizeof(struct sockaddr_can));
        rtx_addr[n].can_ifindex = ifr[n].ifr_ifindex;
        rtx_addr[n].can_family = AF_CAN;
        retval = bind(sock[n], (struct sockaddr *)&rtx_addr[n],sizeof(struct sockaddr_can));
        if (retval < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: Tx rt_dev_bind: %s\n", strerror(-retval));
            return -1;
        }
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: Rx & Tx socket binded successfully.\n");

        // set Tx timeout in nsec
        // retval = ioctl(sock[n], RTCAN_RTIOC_SND_TIMEOUT, &txto);
        if (retval) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_ioctl SND_TIMEOUT: %s\n", strerror(-retval));
            return -1;
        }            
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: Tx Timeout set up successfull.\n");

        // set Rx timeout in nsec
///     retval = rt_dev_ioctl(sock[n], RTCAN_RTIOC_RCV_TIMEOUT, &rxto);
        if (retval) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rt_dev_ioctl RCV_TIMEOUT: %s\n", strerror(-retval));
            return -1;
        }            
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: Rx Timeout set up successfull.\n");
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: Tx/Rx RT-CAN sockets from %d to %d created successfully.\n",sock[0], sock[num_chan-1] );
    return retval;
}

/**
 \brief  init CAN Rx Threads.
 \param 
 \pre    Parse_Module_Parameters()
 \return 0 if ok
        -1 in case of error
 \note   */
static int 
init_rxthreads()
{
    int n, retval = 0;

    for (n = 0; n < num_chan; n++ ) {

        // init pthread attributes
        retval = pthread_attr_init(&thattr[n]);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: %s: thread attributes init failed\n", strerror(-retval));
            hal_exit(comp_id);
            return -1;
        }

        // 
        retval = pthread_attr_setdetachstate(&thattr[n], PTHREAD_CREATE_JOINABLE);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: %s: pthread_attr_setdetachstate failed\n", strerror(-retval));
            hal_exit(comp_id);
            return -1;
        }

        //
///    retval = pthread_attr_setstacksize(&thattr[n], PTHREAD_STACK_MIN);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: %s: pthread_attr_setstacksize failed\n", strerror(-retval));
            hal_exit(comp_id);
            return -1;
        }

        // create Rx thread
        nthread[n] = n;
        retval = pthread_create(&rxthread[n], &thattr[n], &rtcan_rx_and_parse, &nthread[n]);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: %s: pthread_create(rtcan_rx_and_parse) failed\n", strerror(-retval));
            hal_exit(comp_id);
            return -1;
        }

        // set thread scheduling parameters
        retval = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param[n]);
        if (retval != 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: %s: set thread scheduling parameters failed\n", strerror(-retval));
            hal_exit(comp_id);
            return -1;
        }
    }
    return retval;
}

/**
 \brief  Allocate shared memory for FOC_data of each axis
 \pre    Parse_Module_Parameters()
 \return 0 if ok
        -1 in case of error */
int allocate_foc_data()
{
    FOC_data_array = hal_malloc(num_chan*sizeof(FOC_data_t));
    if ( 0 == FOC_data_array ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: hal_malloc() failed\n");
        return -1;
    }
    return 0;
}

/**
 \brief main realtime task */
int rtapi_app_main(void)
{
    int retval = 0;

#ifdef OVERRIDE_MESSAGING_LEVEL
    static int msg_level;

    // save message level on enter
    msg_level = rtapi_get_msg_level();
    /* setup messaging level in:
    RTAPI_MSG_NONE, RTAPI_MSG_ERR, RTAPI_MSG_WARN,
    RTAPI_MSG_INFO, RTAPI_MSG_DBG, RTAPI_MSG_ALL */
    rtapi_set_msg_level(RTAPI_MSG_ALL);
#endif

    // parse parametrs and determine num_chan
    if( 0 != Parse_Module_Parameters() ) {
        hal_exit(comp_id);
        return -1;
    }

    // allocate shared memory for FOC_data of each axis
    if( 0 != allocate_foc_data() ) {
        hal_exit(comp_id);
        return -1;
    }

    // init the component identifier
    comp_id = hal_init("hal_turn_can");
    if( comp_id < 0 ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: hal_init() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    // Export the variables/parameters for each FOC axis
    if( 0 != (retval = exportFOCaxis() ) ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: exportFOCaxis() failed (%d)", retval);
        hal_exit(comp_id);
        return -1;
    }

    // export component functions
    if( 0 != export_functions()) {
        hal_exit(comp_id);
        return -1;
    }

    // init RT-CAN
    if( 0 != init_rtcan()) {
        hal_exit(comp_id);
        return -1;
    }

    // prevent memory swapping
    if ( 0 != ( retval = mlockall(MCL_CURRENT | MCL_FUTURE) ) ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: mklockall failed (%d).", retval);
        hal_exit(comp_id);
        return -1;
    }

    // Rx threads setup
    if( 0 != init_rxthreads()) {
        hal_exit(comp_id);
        return -1;
    }

    // send initial rotor alignment command, wait for alignment is done in the SM
    sendIRA();
    
    // all operations succeded
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: driver installed successfully.\n");
    hal_ready(comp_id);

#ifdef OVERRIDE_MESSAGING_LEVEL
    // return to previous mesaging level
    rtapi_set_msg_level(msg_level);
#endif
    return 0;
}

/** 
 \brief Exit component closing communication with 2FOC 
 \pre */
void rtapi_app_exit(void)
{ 
    int n; //,retval;

    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: shutting down.");

    for (n = 0; n < num_chan; n++) {
        // shutdown axis
        shut_down(n);
    }

//  \todo: check first if socks already openend.
    for (n = 0; n < num_chan; n++) {
        if (sock[n] >= 0) {
///            while ((rt_dev_close(sock[n]) < 0) && (errno == EAGAIN)) {
///             rtapi_print_msg(RTAPI_MSG_ERR, ".");
///             sleep(1);
///         }
        }
        pthread_kill(rxthread[n], SIGHUP); //SIGTERM
        pthread_join(rxthread[n], NULL);
    }

    // notify termination
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: component terminated.\n");

    hal_exit(comp_id);
}

