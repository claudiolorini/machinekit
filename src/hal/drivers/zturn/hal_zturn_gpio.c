/********************************************************************
* Description:  zturn_gpio.c
* Test driver for the ZTurn IC84M GPIO pins 
*
* \author Claudio Lorini (claudio.lorini@iit.it)
* License: GPL Version 2
* Copyright (c) 2015.
*
* code derived from the hal_gpio.c by:
* Author: Michael Haberler
* License: GPL Version 2
* Copyright (c) 2012.
*
********************************************************************/

/**
\brief Test driver for the Zturn-IC84M GPIO (EMIO) pins.
\brief Zynq gpio peripheral register mapping from ug585-Zynq-7000-TRM.pdf 

\details This driver profides 12 insulated digital inputs and 
6 insulated digital outputs and 5 not insulated digital outputs (PMODA)
and 5 not insulated digital inputs (PMODB)
using the IC84M board on Z-Turn platform.

In order to mantain rt performances the access to the peripheral is
done as a memory access to the xilinx-gpio peripheral.
Details of the pheripheral registers structure can be found in the 
following document: ug585-Zynq-7000-TRM.pdf

\par IO connection table:
gpio[53:0]     == MIO  (PS)
gpio[54+64:54] == EMIO (PL)

CAN_LED1 connected to EMIO20 == gpio[54+20]
...
CAN_LED8 connected to EMIO28 == gpio[54+28]
GPI_I1   connected to EMIO29 == gpio[...
...
GPI_I12  connected to EMIO40 
GPO_O1   connected to EMIO41
...
GPO_O6   connected to EMIO46
PMODA1   connected to EMIO47
...
PMODA5   connected to EMIO51
PMODB1   connected to EMIO52
...
PMODB5   connected to EMIO56

IC84M connectors:
I1..3     J3
I4..6     J4
I7..8     J5
I9..12    J6
O1..3     J20
O4..6     J21
PMODA1..5 J24 (not insulated outputs)
PMODB1..5 J25 (not insulated inputs)

\par Revision history:
\date  26.11.2016 started development from hal_gpio.c files

\note
\bug
\warning 
\todo
\pre 
\param 
\return
 
*/

#include "gpio.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "rtapi.h"
#include "rtapi_bitops.h"
#include "rtapi_app.h"
#include "hal.h"

#if !defined(BUILD_SYS_USER_DSO) 
    #error "This driver is for usermode threads only"
#endif
#if !defined(TARGET_PLATFORM_ZTURN)
    #error "This driver is for the Z-Turn platform only"
#endif

MODULE_AUTHOR("Claudio Lorini");
MODULE_DESCRIPTION("Driver for Zturn GPIOs");
MODULE_LICENSE("GPL");

// RT component ID
static int comp_id;

// array of bits, used for input and output
hal_bit_t **oport_data;
hal_bit_t **iport_data;

// number of available isogpin + gpin
#define  NGPI (12+5)
// number of available isogpout + gpout
#define  NGPO  (6+5)

// pointer to start of GPIO pheriferal registers
void *base;
// pointers to access GPIO registers
unsigned *RDATA_2_RO;
unsigned *RDATA_3_RO;
unsigned *RDATA_3;

// file descriptor for mem access
int fd;

/**
 \brief Write insulated outputs (6) and not insulated outputs (5) 
 \pre ngpio and base must be initialized */
static void write_port(void *arg, long period)
{
    static int n=0;

// \todo can be optimized using the RMASK_DATA_3_LSW instead of DATA_3

    for (n = 0; n < NGPO; n++) {
        if (0 == *(oport_data[n])) {
            RTAPI_BIT_CLEAR(RDATA_3, n+8);
        }
        else {
            RTAPI_BIT_SET(RDATA_3, n+8);
        }
    }
}

/**
 \brief Read IO function exported to hal
 \pre ngpio must be initialized  */
static void read_port(void *arg, long period)
{
    // insulated inputs
    *iport_data[0]  = RTAPI_BIT_TEST(RDATA_2_RO, 28);
    *iport_data[1]  = RTAPI_BIT_TEST(RDATA_2_RO, 29);
    *iport_data[2]  = RTAPI_BIT_TEST(RDATA_2_RO, 30);
    *iport_data[3]  = RTAPI_BIT_TEST(RDATA_2_RO, 31);

    *iport_data[4]  = RTAPI_BIT_TEST(RDATA_3_RO, 0);
    *iport_data[5]  = RTAPI_BIT_TEST(RDATA_3_RO, 1);
    *iport_data[6]  = RTAPI_BIT_TEST(RDATA_3_RO, 2);
    *iport_data[7]  = RTAPI_BIT_TEST(RDATA_3_RO, 3);

    *iport_data[8]  = RTAPI_BIT_TEST(RDATA_3_RO, 4);
    *iport_data[9]  = RTAPI_BIT_TEST(RDATA_3_RO, 5);
    *iport_data[10] = RTAPI_BIT_TEST(RDATA_3_RO, 6);
    *iport_data[11] = RTAPI_BIT_TEST(RDATA_3_RO, 7);

    // not insulated inputs on PMODB
    *iport_data[12] = RTAPI_BIT_TEST(RDATA_3_RO, 19);
    *iport_data[13] = RTAPI_BIT_TEST(RDATA_3_RO, 20);
    *iport_data[14] = RTAPI_BIT_TEST(RDATA_3_RO, 21);
    *iport_data[15] = RTAPI_BIT_TEST(RDATA_3_RO, 22);
    *iport_data[16] = RTAPI_BIT_TEST(RDATA_3_RO, 23);
}

/**
 \brief configrure GPIOs, export and configure EMIO gpios, 
 leave untouched MIOs
 \pre base address for the GPIO *MUST* be mapped first */
static void setup_gpio_access()
{    
    unsigned *RDIRM_2 = base + DIRM_2;
    unsigned *RDIRM_3 = base + DIRM_3;
    unsigned *ROEN_3  = base + OEN_3;
    unsigned *RMASK_DATA_3_LSW = base + MASK_DATA_3_LSW;
    unsigned *RMASK_DATA_3_MSW = base + MASK_DATA_3_MSW;

    // configure EMIOs 
    // set LED1..LED8 as outputs
    *RDIRM_2 = *RDIRM_2 | 0x0FF00000;

    // set IO1..IO4 as inputs 
    *RDIRM_2 = *RDIRM_2 & 0x0FFFFFFF;
    // set IO5..IO12 as inputs 
    *RDIRM_3 = *RDIRM_3 & 0xFFFFFF00;

    // set GPO1..GPO6 as outputs
    *RDIRM_3 = *RDIRM_3 | 0x00003F00;
    // enable output drivers
    *ROEN_3  = *ROEN_3  | 0x00003F00;
    // init outputs to 0  
    // \note MASK_DATA_2_LSW ca be used to avoid Read Modify Write
    // operation when writing specific output bits
    // MASK_DATA_x_ySW[31..16] contains the modify mask:
    //  0: pin value is updated
    //  1: pin is masked 
    // MASK_DATA_x_ySW[15..0] contains the data to be written
    *RMASK_DATA_3_LSW = *RMASK_DATA_3_LSW & 0xC0FF3F00;

    // \todo setup PMODs as I or O according to needs 
    // set PMODA1..PMODA5 as outputs
    *RDIRM_3 = *RDIRM_3 | 0x0007C000;
    // enable output drivers
    *ROEN_3  = *ROEN_3  | 0x0007C000;
    // MASK_DATA_x_ySW[15..0] contains the data to be written
    *RMASK_DATA_3_LSW = *RMASK_DATA_3_LSW & 0x3FFFC000;
    *RMASK_DATA_3_MSW = *RMASK_DATA_3_MSW & 0xFFF80007;

    // set PMODB1..PMODB5 as inputs
    *RDIRM_3 = *RDIRM_3 & 0xFF07FFFF;
}

/**
 \brief   Determine Zynq revision
 \details Parse data in /proc/cpuinfo for 'Revision'
 \return 
    -1: unable to parse /proc/cpuinfo
    -1: unable to parse a version number
    nnnn: the one and only revision */
static int zynq_revision()
{
    char *path = "/proc/cpuinfo",  *s, line[1024];
    int rev = -1;
    // parse /proc/cpuinfo for the line: Revision
    char *rev_line = "Revision";
    FILE *f = fopen(path,"r");
  
    if (!f) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: can't open %s: %d - %s\n",
        path, errno, strerror(errno));
        return -1;
    }
  
    while (fgets(line, sizeof(line), f)) {
        if (!strncmp(line, rev_line, strlen(rev_line))) {
            s = strchr(line, ':');
            if (s && 1 == sscanf(s, ":%d", &rev)) {
                fclose(f);
                return rev;
            }
        }
    }
    fclose(f);
    return -1;
}

/**
 \brief   Determine ZTurn FPGA HW revision
 \details The FPGA can contain different resources, a version register determine 
   the available resources
 \todo    Do register read for FPGA versioning  
 \return 
    01: the one and only revision */
static int zb_revision()
{
    return 01;
}

/**
 \brief main realtime task */
int rtapi_app_main(void)
{
    // zynq and FPGA code revision
    int rev, zrev;
    // save messaging level 
    int n, retval = 0;
    // static int msg_level;
    
    // save message level on entering 
    // msg_level = rtapi_get_msg_level();
    
    /* force messaging level in:
    RTAPI_MSG_NONE,
    RTAPI_MSG_ERR,
    RTAPI_MSG_WARN,
    RTAPI_MSG_INFO,
    RTAPI_MSG_DBG,
    RTAPI_MSG_ALL 
    rtapi_set_msg_level(RTAPI_MSG_ALL); */

    // check Zynq revision 
    if ((zrev = zynq_revision()) < 0) {
        // unable to determine zynq revision 
        return -1;
    }
    // notify zynq revision
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_GPIO: Zynq Revision %d \n", zrev);

    // check ZTurn FPGA hardware revision 
    rev = zb_revision();
  
    // do revision specific configuration
    switch (rev) {
        case 01:
            rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_GPIO: ZTurn FPGA Revision 01\n");
        break;

        default:
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: FPGA revision %d not (yet) supported\n", rev);
            return -1;
        break;
    }

    // Open /dev/mem file
    fd = open ("/dev/mem", O_RDWR);
    if (fd < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: Unable to open /dev/mem. Quitting.\n");
        return -1;
    }

    // mmap the device into memory 
    {
        unsigned page_addr, page_offset;
        unsigned page_size=sysconf(_SC_PAGESIZE);

        page_addr = (GPIO_BASE & (~(page_size-1)));
        page_offset = GPIO_BASE - page_addr;
        if (0 != page_offset){
            rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: Pheripheral not aligned to page start! \n");
            return -1;
        }

        base = mmap(NULL, page_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, page_addr);
    }

    // init pointers to access GPIO registers
    RDATA_2_RO = base + DATA_2_RO;
    RDATA_3_RO = base + DATA_3_RO;
    RDATA_3    = base + DATA_3;
 
    // allocate space for IO port data
    iport_data = hal_malloc(NGPI * sizeof(void *));
    oport_data = hal_malloc(NGPO * sizeof(void *));
    if (( 0 == iport_data ) || ( 0 == oport_data )){
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: hal_malloc() failed\n");
        return -1;
    }

    // export and configure gpios
    setup_gpio_access();

    // try to init the component
    comp_id = hal_init("hal_zturn_gpio");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: hal_init() failed\n");
        return -1;
    }

    // make available all gpios in hal
    for (n = 0; n < NGPO; n++) {
        if ( (retval = hal_pin_bit_newf(HAL_IN, &oport_data[n], comp_id, "hal_zturn_gpio.pin-%02d-out", n) ) < 0) {
            break;
        }
    }
    for (n = 0; n < NGPI; n++) {
        if ( (retval = hal_pin_bit_newf(HAL_OUT, &iport_data[n], comp_id, "hal_zturn_gpio.pin-%02d-in", n) ) < 0) {
            break;
        }
    }
    // check for failed gpio pin mapping
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: pin %d export failed with err=%i\n", n,retval);
        hal_exit(comp_id);
        return -1;
    }

    // export the read_port and write_port functions as hal_zturn_gpio.read and hal_zturn_gpio.write in hal
    retval = hal_export_funct("hal_zturn_gpio.write", write_port, 0, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: write funct export failed\n");
        hal_exit(comp_id);
        return -1;
    }
    retval = hal_export_funct("hal_zturn_gpio.read", read_port, 0, 0, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "HAL_ZTURN_GPIO: ERROR: read funct export failed\n");
        hal_exit(comp_id);
        return -1;
    }

    // all operations succeded
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_GPIO: driver installed successfully.\n");
    hal_ready(comp_id);

    // return to previous mesaging level
    // rtapi_set_msg_level(msg_level);

    return 0;
}

/** 
 \brief Exit component */
void rtapi_app_exit(void)
{    
    //close /dev/mem
    close(fd);

    // notify clean termination
    rtapi_print_msg(RTAPI_MSG_INFO, "HAL_ZTURN_GPIO: component terminated successfully \n");

    hal_exit(comp_id);
}


