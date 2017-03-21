#include "hal.h"
#include "2FOC_status.h"

#include "rtapi.h"         
#include "rtapi_app.h"
#include "rtapi_bitops.h"

#if !defined(TARGET_PLATFORM_ZTURN)
  #error "This driver is for the Z-Turn platform only."
#endif

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

/** \brief rt-can interface number  */
int can_ifn[MAX_FOC_CHAN] = { -1,-1,-1,-1,-1,-1,-1,-1 };
RTAPI_MP_ARRAY_INT(can_ifn, MAX_FOC_CHAN, "CAN channel number for up to 8 lines");

// RT component ID
static int comp_id;

/** \brief Array of FOC_data structs in shared memory, 1 per configured axis */
static FOC_data_t *FOC_data_array = NULL;

static void rtcan_periodic_send(void *arg, long period)
{
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
        // found a correctly configured channel(s)
        num_chan++;
        // report interface number and the connected 2FOC can address
        rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: FOC axis %d @ can%d interface.",
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

static int export_functions() 
{
    int retval=0;

    // export the send/receive rtcan messages
    retval = hal_export_funct("hal_zturn_can.rtcan_periodic_send", rtcan_periodic_send, FOC_data_array, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: rtcan_periodic_send funct export failed\n");
        return -1;
    }

    return retval;
}

/**
 \brief main realtime task */
int rtapi_app_main(void)
{
    rtapi_set_msg_level(RTAPI_MSG_ALL);

    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: Starting driver.\n");

    // init the component identifier
    comp_id = hal_init("hal_zturn_can");
    if( comp_id < 0 ) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_CAN: ERROR: hal_init() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    // determine driver parameters (number of can and address) 
    Parse_Module_Parameters();

    // allocate shared memory for FOC_data of each axis
    if( 0 != allocate_foc_data() ) {
        hal_exit(comp_id);
        return -1;
    }

    // export component functions
    if( 0 != export_functions()) {
        hal_exit(comp_id);
        return -1;
    }

    // all operations succeded
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: driver installed successfully.\n");
    hal_ready(comp_id);

    return 0;
}

/** 
 \brief Exit component closing communication with 2FOC 
 \pre */
void rtapi_app_exit(void)
{ 

    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: shutting down.");
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_CAN: component terminated.\n");
    hal_exit(comp_id);
}

