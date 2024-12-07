#ifndef Hardware_h
#define Hardware_h

#define  TMS    0
#define  TDI    1
#define  TDO    2
#define  TCK    3
#define  VREF   4

// DMA variables and structures
#define DMAC_MEMCH 3

extern volatile int Bcount;

typedef struct
{
  uint32_t  SADDR;
  uint32_t  DADDR;
  uint32_t  CTRLA;
  uint32_t  CTRLB;
  uint32_t  DSCR;
} LLI;    // DMA list structure

// IO pins

#define ARBline1     A6      // Used to signal a sync operation or trigger, was A0 on the older rev with the plug
                             // Arduino
#define ARBline2     22      // Used to flag compression mode
#define MUXADD       0x4D    // Mux TWI address used on rev 4.0 and high ARB amp
#define MUXSEL       42      // This line is used with ARB 5.0 and ARB_AMP 4.0, selects readbacks for 
                             // AD8 and AD9
                             // When LOW (Note, rev 4.0 ARB amp has a bug and this mode does not work)
                             //   AD8 = V+ supply
                             //   AD9 = V- supply
                             // When HIGH
                             //   AD8 = Offset monitor
                             //   AD9 = Aux monitor
// CPU ADC channels, used for readbacks
#define AUX_VNEG    9        // Aux or Negative HV read back, MUXSEL high sets AUX
#define OFF_VPOS    8        // Offset or Positive HV read back, MUXSEL high sets OFF
#define VMUX        1        // ARB output readback mux output

extern int ARBsync;          // Used to signal a sync operation or trigger, was A0 on the older rev with the plug
                             // Arduino
extern int CompressPin;      // Used to flag compression mode

#define ExtClockSel  24      // Output to the CPLD is select the external clock source, the MIPS controller
                             // or the external clock source using the ext clock adapter
#define ClockMode    23      // Clock mode to the PLD, high = external clock

#define PowerEnable  25      // Enable the power supply, logic high disables

// DAC constants
#define DACadr      0x12

#define DACrangeCH   0
#define DACoffsetA   6
#define DACoffsetB   7
#define DACoffsetCH  3
#define DACauxCH     2
#define DACrefCH     1

void DMAbuffer2DAC(uint32_t *dst, uint32_t *src, uint32_t n, bool NoTrigger = false);

int readADC(int chan, int num=1);


#endif
