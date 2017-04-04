#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "hal.h"
#include "2FOC_status.h"

#include "rtapi.h"         
#include "rtapi_app.h"
#include "rtapi_bitops.h"

#if !defined(TARGET_PLATFORM_ZTURN)
  #error "This driver is for the Z-Turn platform only."
#endif

// override MK messaging level to give a lot of debug messagers without much bloating 
// from other modules.
#define OVERRIDE_MESSAGING_LEVEL 1

MODULE_AUTHOR("Claudio Lorini");
MODULE_DESCRIPTION("CAN Driver for 2FOC controller.");
MODULE_LICENSE("GPL");

/** \brief maximum number of FOC channels */
#define MAX_FOC_CHAN 8

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

/** \brief number of FOC channels configured */
static int num_chan = 0;

/** \brief can interface number  */
int can_ifn[MAX_FOC_CHAN] =  { [0 ... MAX_FOC_CHAN-1] = -1 };
RTAPI_MP_ARRAY_INT(can_ifn, MAX_FOC_CHAN, "CAN channel number for up to 8 lines");

/** \brief can address of the board connected to the can line 
by default all 2FOC boards have adx=3 */
int can_addr[MAX_FOC_CHAN] =  { [0 ... MAX_FOC_CHAN-1] = 3 };
RTAPI_MP_ARRAY_INT(can_addr, MAX_FOC_CHAN, "CAN address of the board connected to the line");

// RT component ID
static int comp_id;

/** \brief Array of FOC_data structs in shared memory, 1 per configured axis */
static FOC_data_t *FOC_data_array = NULL;

int soc;

static void can_periodic_send(void *arg, long period)
{
    struct can_frame frame_rd;
    int recvbytes = 0;
    fd_set readSet;

    //  sec, usec if both 0 select returns immediatly 
    struct timeval timeout = {0,000};

    FD_ZERO(&readSet);
    FD_SET(soc, &readSet);

    if (select((soc + 1), &readSet, NULL, NULL, &timeout) >= 0)
    {
        if (FD_ISSET(soc, &readSet)) {
            recvbytes = read(soc, &frame_rd, sizeof(struct can_frame));
            if(recvbytes) {
                if(frame_rd.can_dlc > 0) {
                    int i;

                    //rtapi_print_msg(RTAPI_MSG_INFO,"\ndlc = %d, id = %x pl=", frame_rd.can_dlc,frame_rd.can_id);
                    for(i=0 ; i < frame_rd.can_dlc ; i++) {
                        ;// rtapi_print_msg(RTAPI_MSG_INFO," %x", frame_rd.data[i]);
                    }
                    //rtapi_print_msg(RTAPI_MSG_INFO,"\n");
                }
                else{
                    ;//rtapi_print_msg(RTAPI_MSG_INFO,"\ndlc = %d, id = %x \n", frame_rd.can_dlc,frame_rd.can_id);
                }
            }
        }
        else {
            // timeout
            rtapi_print_msg(RTAPI_MSG_INFO,"HAL_ZTURN_CAN: Timeout receiving from CAN\n");
        }
    }
}


int open_can_port(const char *port)
{
    struct ifreq ifr;
    struct sockaddr_can addr;
    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0) {
        return (-1);
    }
    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, port);
    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0) {
        return (-1);
    }
    addr.can_ifindex = ifr.ifr_ifindex;
    fcntl(soc, F_SETFL, O_NONBLOCK);
    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        return (-1);
    }
    return 0;
}

int close_can_port()
{
    close(soc);
    return 0;
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
        // check for a valid can interface number (0..7) 
        if( (can_ifn[n] < 0) || ( can_ifn[n] > 7) ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: bad can interface number %i", can_ifn[n]);
            return -1;
        }

        // check for a valid can address number (1..15) 
        if( (can_addr[n] < 1) || ( can_addr[n] > 15) ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: bad board can address %i", can_addr[n]);
            return -1;
        }

        // found a correctly configured channel(s)
        num_chan++;
        
        // report interface number and the connected 2FOC can address
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: FOC axis %d @ can%d interface, board address %d.",
            n, can_ifn[n], can_addr[n] );
    }
    if( (0 == num_chan) || (8 < num_chan) ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: invalid number of channels: %d.", num_chan);
        return -1;
    }
    // ok.
    return 0;
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
 \brief  Export IO pins and parameters for component
 \pre    Parse_Module_Parameters()
 \return -1 in case of error  */
static int exportFOCaxis()
{    
    int retval = 0;
    int num;

    for ( num = 0; num < num_chan; num++) {

        // I PINS
        // make available Emergency stop in hal
        if ( (retval = hal_pin_bit_newf(HAL_IN,&(FOC_data_array[num].estop), comp_id, "hal_zturn_can.%d.estop", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin estop export failed with err=%d", retval);
        }
        // Drive Enable
        if ( (retval = hal_pin_bit_newf(HAL_IN,&(FOC_data_array[num].driven), comp_id, "hal_zturn_can.%d.driven", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin driven export failed with err=%d", retval);
        }
        // setpoint
        if ( (retval = hal_pin_s32_newf(HAL_IN, &(FOC_data_array[num].setpoint), comp_id, "hal_zturn_can.%d.setpoint", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin setpoint export failed with err=%d", retval);
        }

        // O PINS
        // position feedback
        if ( (retval = hal_pin_s32_newf(HAL_OUT, &(FOC_data_array[num].feedback), comp_id, "hal_zturn_can.%d.feedback", num) ) < 0) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin feedback export failed with err=%d", retval);
        }
        // 2FOC Status
        if( (retval = hal_pin_u32_newf(HAL_OUT, &(FOC_data_array[num].focstatus), comp_id, "hal_zturn_can.%d.status", num) != 0) ) {
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: pin status export failed with err=%d" ,retval);
        }
        // 2FOC Errors
        if( (retval = hal_pin_u32_newf(HAL_OUT, &(FOC_data_array[num].focerror), comp_id, "hal_zturn_can.%d.error", num) != 0) ) {
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

static int export_functions() 
{
    int retval=0;

    // export the send/receive can messages
    retval = hal_export_funct("hal_zturn_can.can_periodic_send", can_periodic_send, FOC_data_array, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: can_periodic_send funct export failed\n");
        return -1;
    }

    return retval;
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

    rtapi_set_msg_level(RTAPI_MSG_ALL);

    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: Starting driver.\n");

    // determine driver parameters (number of can and address) 
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
    comp_id = hal_init("hal_zturn_can");
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

    open_can_port("can0");

    // export component functions
    if( 0 != export_functions()) {
        hal_exit(comp_id);
        return -1;
    }

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
    close_can_port();

    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: shutting down.");
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: component terminated.\n");
    hal_exit(comp_id);
}

