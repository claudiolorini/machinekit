/**
\brief Zynq gpio peripheral register mapping from ug585-Zynq-7000-TRM.pdf 
*/

/** \brief gpio peripheral base address */
#define GPIO_BASE           0xE000A000 

/** \breief This registers enables software to change the value being output.  
Only data values with a corresponding deasserted mask bit will be changed.
This permits to avoid read-modify-write cycles on writing single gpios.
MASK_DATA_x_ySW[31..16] contains the modify mask:
  0: pin value is updated
  1: pin is masked 
MASK_DATA_x_ySW[15..0] contains the data to be written
*/
#define MASK_DATA_0_LSW     0x00000000 
#define MASK_DATA_0_MSW     0x00000004 
#define MASK_DATA_1_LSW     0x00000008 
#define MASK_DATA_1_MSW     0x0000000C 
#define MASK_DATA_2_LSW     0x00000010 
#define MASK_DATA_2_MSW     0x00000014 
#define MASK_DATA_3_LSW     0x00000018 
#define MASK_DATA_3_MSW     0x0000001C 

/** \breief This register controls the value being output when the GPIO 
signal is configured as an output. All 32bits of this register are written 
at one time. */
#define DATA_0              0x00000040
#define DATA_1              0x00000044
#define DATA_2              0x00000048
#define DATA_3              0x0000004C

/** \breief This register enables software to observe the value on the device pin. 
If the GPIO signal is configured as an output, then this would normally reflect 
the value being driven on the output. Writes to this register are ignored.*/
#define DATA_0_RO           0x00000060 // 00..31 [8:7] cannot be used as inputs, will always return 0 when read.
#define DATA_1_RO           0x00000064 // 32..53 
#define DATA_2_RO           0x00000068 // 54..85
#define DATA_3_RO           0x0000006C // 86..117

/** \breief This register controls whether the IO pin is acting as an input 
or an output. Since the input logic is always enabled, this effectively 
enables/disables the output driver.
0: input
1: output */
#define DIRM_0              0x00000204 
#define DIRM_1              0x00000244 
#define DIRM_2              0x00000284
#define DIRM_3              0x000002C4

/** \breief When the IO is configured as an output, this controls whether 
the output is enabled or not. When the output is disabled, the pin is tri-stated.
0: disabled
1: enabled */
#define OEN_0               0x00000208
#define OEN_1               0x00000248
#define OEN_2               0x00000288
#define OEN_3               0x000002C8

/** \breief This register shows which bits are currently masked and which 
are un-masked/enabled. This register is read only, so masks cannot be 
changed here.
0: interrupt source enabled
1: interrupt source masked */
#define INT_MASK_0          0x0000020C 
#define INT_MASK_1          0x0000024C 
#define INT_MASK_2          0x0000028C
#define INT_MASK_3          0x000002CC 

/** \breief This register is used to enable or unmask a GPIO input for 
use as an interrupt source.Writing a 1 to any bit of this register enables/unmasks 
that signal for interrupts. */
#define INT_EN_0            0x00000210 
#define INT_EN_1            0x00000250
#define INT_EN_2            0x00000290
#define INT_EN_3            0x000002D0 

/** \breief This register is used to disable or mask a GPIO input for use 
as an interrupt source. Writing a 1 to any bit of this register disables/masks 
that signal for interrupts. */
#define INT_DIS_0           0x00000214 
#define INT_DIS_1           0x00000254
#define INT_DIS_2           0x00000294
#define INT_DIS_3           0x000002D4

/** \breief This registers shows if an interrupt event has occurred or not. 
Writing a 1 to a bit in this register clears the interrupt status for that bit. */
#define INT_STAT_0          0x00000218
#define INT_STAT_1          0x00000258
#define INT_STAT_2          0x00000298
#define INT_STAT_3          0x000002D8

/** \breief This register controls whether the interrupt is edge sensitive 
or level sensitive.
0: level-sensitive
1: edge-sensitive */
#define INT_TYPE_0          0x0000021C
#define INT_TYPE_1          0x0000025C
#define INT_TYPE_2          0x0000029C
#define INT_TYPE_3          0x000002DC

/** \breief This register controls whether the interrupt is active-low or 
active high (or falling-edge sensitive or rising-edge sensitive).
0: active low or falling edge
1: active high or rising edge */
#define INT_POLARITY_0      0x00000220 
#define INT_POLARITY_1      0x00000260
#define INT_POLARITY_2      0x000002A0
#define INT_POLARITY_3      0x000002E0 

/** \breief If INT_TYPE is set to edge sensitive, then this register enables an 
interrupt event on both rising and falling edges. This register is ignored if 
INT_TYPE is set to level sensitive.
0: trigger on single edge, using configured interrupt polarity
1: trigger on both edges */
#define INT_ANY_0           0x00000224 
#define INT_ANY_1           0x00000264
#define INT_ANY_2           0x000002A4
#define INT_ANY_3           0x000002E4

