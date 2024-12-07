#include "hardware.h"
#include "SPI.h"

int ARBsync    = A6;      // Used to signal a sync operation or trigger, was A0 on the older rev with the plug in
                          // Arduino DUE. A6 is also corrent on hardware rev 4.2
int CompressPin= 22;      // Used to flag compression mode

// Two structure arrays used to enable continous DMA of the buffer.
#define NumLLI  4
volatile LLI lliA[NumLLI];   
volatile LLI lliB[NumLLI]; 
volatile int lliMax = 0;
volatile int Bcount;    // Used to count the number of buffers transfered to the DACs

// On rev 4.2 controller the following processor pins are connected to the programming 
// pins of the CPLD
// PA8 to TMS Due pin 0
// PA7 to TDI Due pin 26
// PA5 to TDO Due pin not defined
// PA1 to TCK Due pin CANRX
// To use the jtag library we need to create a pin discriptor aray with these pins defined.
// We also need to over load functions in jtag driver to use our pin description array
// M0 pin assigenments
PinDescription jtag_APinDescription[]=
{
  { PIOA, PIO_PA8,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},  // TMS, 0
  { PIOA, PIO_PA7,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},  // TDI, 1
  { PIOA, PIO_PA5,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},  // TDO, 2
  { PIOA, PIO_PA1,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},  // TCK, 3
  { PIOB, PIO_PB22,  ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},  // VREF, 4
};
GPIO_BIT_TYPE mydigitalPinToBitMask(uint8_t pin)
{
  return jtag_APinDescription[pin].ulPin;
}

GPIO_PORT_TYPE mydigitalPinToPort(uint8_t pin)
{
  return jtag_APinDescription[pin].pPort;
}

void mypinMode(GPIO_BIT_TYPE ulPin, uint8_t m)
{
    switch(m)
    {
        case INPUT:
            /* Enable peripheral for clocking input */
            pmc_enable_periph_clk(jtag_APinDescription[ulPin].ulPeripheralId);
            PIO_Configure(jtag_APinDescription[ulPin].pPort,PIO_INPUT,jtag_APinDescription[ulPin].ulPin,0);
            break;
        case INPUT_PULLUP:
            /* Enable peripheral for clocking input */
            pmc_enable_periph_clk( jtag_APinDescription[ulPin].ulPeripheralId);
            PIO_Configure(jtag_APinDescription[ulPin].pPort,PIO_INPUT,jtag_APinDescription[ulPin].ulPin,PIO_PULLUP);
            break;
        case OUTPUT:
            PIO_Configure(jtag_APinDescription[ulPin].pPort,PIO_OUTPUT_0,jtag_APinDescription[ulPin].ulPin,jtag_APinDescription[ulPin].ulPinConfiguration );
            /* if all pins are output, disable PIO Controller clocking, reduce power consumption */
            if ( jtag_APinDescription[ulPin].pPort->PIO_OSR == 0xffffffff)
            {
                pmc_disable_periph_clk(jtag_APinDescription[ulPin].ulPeripheralId);
            }
            break;
        default:
            break;
    }
}


// On hardware rev 4.2 these are the three option pins, enabled an inputs with pull ups
// Port B bit 24 is position 1
// Port B bit 23 is position 2
// Port B bit 22 is position 3
PinDescription opt_APinDescription[]=
{
  { PIOB, PIO_PB24,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},
  { PIOB, PIO_PB23,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},
  { PIOB, PIO_PB22,   ID_PIOB, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_TIMER, NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER},
};

void initOptionPins(void)
{
   // Init the option input pins
   PIO_Configure(opt_APinDescription[0].pPort,PIO_INPUT,opt_APinDescription[0].ulPin,PIO_PULLUP);
   PIO_Configure(opt_APinDescription[1].pPort,PIO_INPUT,opt_APinDescription[1].ulPin,PIO_PULLUP);
   PIO_Configure(opt_APinDescription[2].pPort,PIO_INPUT,opt_APinDescription[2].ulPin,PIO_PULLUP);
}

int8_t readOptionPins(void)
{
  int8_t val=0; 
  
  if(PIO_Get(opt_APinDescription[0].pPort, PIO_INPUT, opt_APinDescription[0].ulPin) == 1) val |= 1;
  if(PIO_Get(opt_APinDescription[1].pPort, PIO_INPUT, opt_APinDescription[1].ulPin) == 1) val |= 2;
  if(PIO_Get(opt_APinDescription[2].pPort, PIO_INPUT, opt_APinDescription[2].ulPin) == 1) val |= 4;
  return val;
}

void ReportOption(void)
{
  serial->println(readOptionPins());
}

//
// DMA routines
//

/** Disable DMA Controller. */
inline void dmac_disable()
{
  DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);
}
/** Enable DMA Controller. */
inline void dmac_enable()
{
  DMAC->DMAC_EN = DMAC_EN_ENABLE;
}
/** Disable DMA Channel. */
inline void dmac_channel_disable(uint32_t ul_num)
{
  DMAC->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
}
/** Enable DMA Channel. */
inline void dmac_channel_enable(uint32_t ul_num)
{
  DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
}
/** Suspend DMA Channel. */
inline void dmac_channel_suspend(uint32_t ul_num)
{
  DMAC->DMAC_CHER = DMAC_CHER_SUSP0 << ul_num;
}
/** Poll for FIFO empty. */
inline bool dmac_channel_fifo_empty(uint32_t ul_num)
{
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_EMPT0 << ul_num)) ? false : true;
}
/** Poll for transfer complete. */
inline bool dmac_channel_transfer_done(uint32_t ul_num)
{
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) ? false : true;
}

// DMA ISR is used to reenable the completed buffers. This interrupt fires when
// a buffer completes and this code clears the done bit to keep the process
// going.
// This routine only files in the DMA mode
void DMAC_Handler(void)
{
  static uint32_t i;
  static bool     lastAlt = false;
  static int      CM;

if(Bcount == -1) 
{
  // Ignore any pending buffer loaded interrupts.
  Bcount = 2;
  return;
}
  i = DMAC->DMAC_EBCISR;
  Bcount++;
  if(ARBparms.Mode == ARBmode) if((Bcount >= ARBparms.NumBuffers-1) && (ARBparms.NumBuffers != 0))
  {
//    dmac_channel_disable(DMAC_MEMCH);
    return;
  }
//  if((ARBparms.Mode == TWAVEmode) && (ARBparms.CompressEnable)  && (ARBparms.Order > 1))
  if((ARBparms.Mode == TWAVEmode) && (ARBparms.CompressEnable)  && (WorkingOrder != 1))
  {
    if(Bcount > WorkingOrder)
    {
      lliA[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      lliB[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      Bcount = 1;
    }
    else if(Bcount == WorkingOrder)
    {
      lliA[0].SADDR = (uint32_t)&(buffer[0]);
      lliB[0].SADDR = (uint32_t)&(buffer[0]);
      Bcount = 0;
      CrampCounter++;
      if((CrampCounter >= abs(ARBparms.CompressRamp)) && (ARBparms.CompressRamp != 0))
      {
        CrampCounter = 0;
        if(ARBparms.CompressRamp < 0) WorkingOrder -= ARBparms.CrampOrder;
        else WorkingOrder += ARBparms.CrampOrder;
        if(WorkingOrder < 2) WorkingOrder = 2;
      }
    }
    else
    {
      lliA[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      lliB[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);      
    }
  }
  else if (ARBparms.Mode == TWAVEmode)
  {
    if(ARBparms.AlternateEnable)
    { 
      lliA[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      lliB[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]); 
      if((!lastAlt) && (ARBparms.AlternateRngEna))  WriteWFrange(ARBparms.AlternateRng); // AuxDACupdate |= UpdateAltRange;, 11/11/22
      if((!lastAlt) && (ARBparms.AlternateFreqEna))
      {

        // If here set the frequency to the alternate value
        CM = digitalRead(ClockMode);
        digitalWrite(ClockMode,HIGH);
        SetFrequency(ARBparms.AltFreq);
        digitalWrite(ClockMode,LOW);
      }
      lastAlt = true;   
    }
    else
    {
       lliA[0].SADDR = (uint32_t)&(buffer[0]);
       lliB[0].SADDR = (uint32_t)&(buffer[0]);
       if((lastAlt) && (ARBparms.AlternateRngEna)) WriteWFrange(ARBparms.VoltageRange); //AuxDACupdate |= UpdateVoltageRange;, 11/11/22
//       AuxDACupdate |= UpdateVoltageRange;  // Aug 16, 2022, removed 12/16/23, this causes constant update of 
                                              // range DAC and interferrs with alt range toggling
      if((lastAlt) && (ARBparms.AlternateFreqEna))
      {

        // If here reset the frequency to the orginal value value
        digitalWrite(ClockMode,HIGH);
        SetFrequency(ARBparms.ActualFreq);
        digitalWrite(ClockMode,CM);
      }
      lastAlt = false;          
    }
  }
  for(i=0;i<NumLLI;i++)
  {
     lliA[i].CTRLA &= ~DMAC_CTRLA_DONE;
     lliB[i].CTRLA &= ~DMAC_CTRLA_DONE;    
  }
}

// This function restarts the DMA buffer transfer. This function will stop the DMA process, reset the done
// bits and then restart the DMA process. This is used in the Twave mode to sync the waveform generation
void DMArestart_old(void)
{
  uint32_t i;

  digitalWrite(PowerEnable,HIGH);
  // Reset the Cramp parameters
  WorkingOrder = ARBparms.Order;
  CrampCounter = 0;
  // Stop the DMA process
  dmac_channel_disable(DMAC_MEMCH);
  while(!dmac_channel_transfer_done(DMAC_MEMCH));
  digitalWrite(PowerEnable,LOW);
//  DMAC->DMAC_EBCIER = 0; //11-3-18
  i = DMAC->DMAC_EBCIER; //11-3-18
  DMAbuffer2DAC((uint32_t *)0x60000000, buffer, ARBparms.ppp * NP * CHANS / 4);
  Bcount = 2;
  if(dmac_channel_fifo_empty(DMAC_MEMCH)) Bcount = -1;    // This will only be true when there is a pending interrupt, can't clear it!
                                                          // Bcount is always -1 due to semi-colon, 11-3-18 removed semi-colon
  return;
}

void DMAstartISR(void)
{
 uint32_t i;

//  DMAC->DMAC_EBCIER = 0; //11-3-18
  i = DMAC->DMAC_EBCIER; //11-3-18
  DMAbuffer2DAC((uint32_t *)0x60000000, buffer, ARBparms.ppp * NP * CHANS / 4);
  Bcount = 2;
  if(dmac_channel_fifo_empty(DMAC_MEMCH)) Bcount = -1;    // This will only be true when there is a pending interrupt, can't clear it!
                                                          // Bcount is always -1 due to semi-colon, 11-3-18 removed semi-colon  
}

void DMArestart(void)
{
  // Set RC count to delay time plus width
  ResetTMR.setRC(150);
  // Start the timer
  ResetTMR.enableTrigger();
  ResetTMR.softwareTrigger();

  // Reset the Cramp parameters
  WorkingOrder = ARBparms.Order;
  CrampCounter = 0;
  // Stop the DMA process
  dmac_channel_disable(DMAC_MEMCH);
  while(!dmac_channel_transfer_done(DMAC_MEMCH));
}

// This function starts the DMA continous transfer.
void DMAbuffer2DAC(uint32_t *dst, uint32_t *src, uint32_t n, bool NoTrigger)
{
  uint32_t i;

  dmac_channel_disable(DMAC_MEMCH);
  while(!dmac_channel_transfer_done(DMAC_MEMCH));
  // Calculate lliMax based on buffer size, each DMA BTSIZE can not exceed 4095.
  // Setup each sent of LLI arrays
  lliMax = 0;
  for(i=0;i<NumLLI;i++)
  {
    if(i < NumLLI)
    {
      lliA[i].DSCR = (uint32_t)&lliA[i+1];
      lliB[i].DSCR = (uint32_t)&lliB[i+1];
    }
    lliA[i].SADDR = (uint32_t)src + (4095 * 4 * i);
    lliA[i].DADDR = (uint32_t)dst + (4095 * 4 * i);
    lliB[i].SADDR = (uint32_t)src + (4095 * 4 * i);
    lliB[i].DADDR = (uint32_t)dst + (4095 * 4 * i);
    if(n <= 4095)
    {
       lliA[i].CTRLA = n | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;
       lliB[i].CTRLA = n | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;
       lliA[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING;
       lliB[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING;
       break;
    }
    else
    {
       lliA[i].CTRLA = 4095 | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;
       lliB[i].CTRLA = 4095 | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;     
       lliA[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING | DMAC_CTRLB_IEN;
       lliB[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING | DMAC_CTRLB_IEN;
       n -= 4095;
       lliMax++; 
    }
  }
  // Setup the source and destination addresses and link the ping-pong  buffers
  lliA[0].SADDR = (uint32_t)src;
  lliA[0].DADDR = (uint32_t)dst;
  lliA[lliMax].DSCR =  (uint32_t)&lliB[0];
  lliB[0].SADDR = (uint32_t)src;
  lliB[0].DADDR = (uint32_t)dst;
  lliB[lliMax].DSCR =  (uint32_t)&lliA[0];
  // Setup the source registers when in compress mode. 
  if((ARBparms.CompressEnable) && (WorkingOrder == 2))
  {
     lliA[0].SADDR = (uint32_t)&(src[ARBparms.ppp * CHANS / 4]);   // Compress waveform
     lliB[0].SADDR = (uint32_t)src;                       // Normal waveform
  }
//  if((ARBparms.CompressEnable) && (ARBparms.Order > 2))
  if((ARBparms.CompressEnable) && ((WorkingOrder > 2) || (WorkingOrder == 0)))
  {
     lliA[0].SADDR = (uint32_t)&(src[ARBparms.ppp * CHANS / 4]);
     lliB[0].SADDR = (uint32_t)&(src[ARBparms.ppp * CHANS / 4]);
  }
  
  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_SADDR = 0;
  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_DADDR = 0;

  if((ARBparms.NumBuffers == 1) && (ARBparms.Mode == ARBmode)) lliB[0].CTRLA |= DMAC_CTRLA_DONE;

  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_DSCR =  (uint32_t) & (lliA[0]);

  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_CTRLB = DMAC_CTRLB_FC_MEM2MEM_DMA_FC |
      DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING;

  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_CFG = DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG;

  DMAC->DMAC_EBCIER |= 1 << (DMAC_MEMCH);

  NVIC_ClearPendingIRQ(DMAC_IRQn);
  NVIC_EnableIRQ(DMAC_IRQn);

  i = DMAC->DMAC_EBCISR;

  Bcount = 0;
  if(!NoTrigger) dmac_channel_enable(DMAC_MEMCH);
}

// ******* End DMA routines *********

void Software_Reset()
{
  //============================================================================================
  //   führt ein Reset des Arduino DUE aus...
  //
  //   Parameter: keine
  //   Rueckgabe: keine
  //============================================================================================
  const int RSTC_KEY = 0xA5;
  RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
  while (true);
}

void WriteWFrange(float value)
{
  int i = ARBparms.GainDACm * value + ARBparms.GainDACb;
  if(i > 65535) i = 65535;
  if(i < 0) i = 0;
  //noInterrupts();  // Removed 3/11/23, this caused the DMA transfer to stall due to missing interrupts
  AD5592writeDAC(AD5592_CS, DACrangeCH, i);
  if((readOptionPins() & 0x1) == 0)
  {
    AD5592writeDAC(AD5592_CS, DACoffsetA, i * 0.545);
  }
  //interrupts();
}

void WriteWFoffset(float value)
{
  int i = ARBparms.OffsetDACm * value + ARBparms.OffsetDACb;
  if(i > 65535) i = 65535;
  if(i < 0) i = 0;
  AD5592writeDAC(AD5592_CS, DACoffsetCH, i);
}

void WriteWFaux(float value)
{
  int i = ARBparms.AuxDACm * value + ARBparms.AuxDACb;
  if(i > 65535) i = 65535;
  if(i < 0) i = 0;
  AD5592writeDAC(AD5592_CS, DACauxCH, i);
}

void WriteBoardBias(int board, float value)
{
  int i = ARBparms.BiasCalM[board] * value + ARBparms.BiasCalB[board];
  if(i > 4095) i = 4095;
  if(i < 0) i = 0;
  analogWrite(DACaddBias[board],i);
  dacc_set_timing(DACC_INTERFACE, 0x01, 0, 0x10);
}

void MeasureVoltages(void)
{
  int   Counts;
  float Vp, Vn;

  // Calculate the +- 12 Volt supples from the readback channels
  analogRead(10);
  Counts = analogRead(10);
  Vp = ((float)Counts / 4095.0) * 19.8;
  analogRead(11);
  Counts = analogRead(11);
  Vn = 1.4 * Vp - (((float)Counts / 4095.0) * 3.3) * 2.4;
  PS12v[0] = Vp;
  PS12v[1] = Vn;
  // Calculate the +- 50 Volt supples from the readback channels
  digitalWrite(MUXSEL,LOW);
  analogRead(8);
  Counts = analogRead(8);
  Vp = ((float)Counts / 4095.0) * 80.85;
  analogRead(9);
  Counts = analogRead(9);
  Vn = 1.1064 * Vp - (((float)Counts / 4095.0) * 3.3) * 2.1064;
  PS50v[0] = Vp;
  PS50v[1] = Vn;  
  }

// AD5625 is a 4 channel DAC.
//
// This function outputs a value to the selected channel.
// adr = TWI address of device
// chan = channel number, 0,1,2, or 3
// val = binary value to output to the DAC
//
// Return the status of the TWI transaction, 0 if no errors.
int AD5625(int8_t adr, uint8_t chan, uint16_t val)
{
  AD5625(adr, chan, val, 0);
}

int AD5625(int8_t adr, uint8_t chan, uint16_t val,int8_t Cmd)
{
  int iStat;

  Wire1.beginTransmission(adr);
  Wire1.write((Cmd << 3) | chan);
  //    if(chan <= 3) val <<= 4;
  Wire1.write((val >> 8) & 0xFF);
  Wire1.write(val & 0xFF);
  {
    iStat = Wire1.endTransmission();
  }
  return (iStat);
}

// This function enables the internal voltage reference in the
// AD5625
int AD5625_EnableRef(int8_t adr)
{
  int iStat;

  Wire1.beginTransmission(adr);
  Wire1.write(0x38);
  Wire1.write(0);
  Wire1.write(1);
  iStat = Wire1.endTransmission();
  return (iStat);
}

// AD5592 IO routines. This is a analog and digitial IO chip with
// a SPI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5592
void AD5592write(int CS, uint8_t reg, uint16_t val)
{
  int iStat;

  digitalWrite(CS,LOW);
  SPI.transfer(((reg << 3) & 0x78) | (val >> 8));
  SPI.transfer(val & 0xFF);
  digitalWrite(CS,HIGH);
}

// Read from AD5593R
// returns 16 bit value read
int AD5592readWord(int CS)
{
  uint16_t  val;

  digitalWrite(CS,LOW);
  val = SPI.transfer16(0);
  digitalWrite(CS,HIGH);
  return val;
}

// Returns -1 on error. Error is flaged if the readback channel does not match the
// requested channel.
// chan is 0 thru 7
int AD5592readADC(int CS, int8_t chan)
{
   uint16_t  val;

   // Write the channel to convert register
   AD5592write(CS, 2, 1 << chan);
   // Dummy read
   digitalWrite(CS,LOW);
   SPI.transfer16(0);
   digitalWrite(CS,HIGH);
   // Read the ADC data 
   digitalWrite(CS,LOW);
   val = SPI.transfer16(0);
   digitalWrite(CS,HIGH);
   // Test the returned channel number
   if(((val >> 12) & 0x7) != chan) return(-1);
   // Left justify the value and return
   val <<= 4;
   return(val & 0xFFF0);
}

int AD5592readADC(int CS, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5592readADC(CS, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

// There is a potential for this function to be interrupted and then called again
// from the interrupt. The code was updated 12/15/23 to queue up a request when
// busy and execute when finished.
void AD5592writeDAC(int CS, int8_t chan, int val)
{
   uint16_t      d;
   static bool   busy=false;
   static bool   queded = false;
   static int    qCS,qval;
   static int8_t qchan;
   
   if(busy)
   {
    queded = true;
    qCS=CS;
    qval=val;
    qchan=chan;
    return;
   }
   busy = true;
   // convert 16 bit DAC value into the DAC data data reg format
   d = ((val>>4) & 0x0FFF) | (((uint16_t)chan) << 12) | 0x8000;
   digitalWrite(CS,LOW);
   val = SPI.transfer((uint8_t)(d >> 8));
   val = SPI.transfer((uint8_t)d);
   digitalWrite(CS,HIGH);
   busy = false;
   if(queded)
   {
    queded = false;
    AD5592writeDAC(qCS,qchan,qval);
   }
}

// End of AD5592 routines


// AD5593 IO routines. This is a analog and digitial IO chip with
// a TWI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5593
// Return 0 if no error else an error code is returned
int AD5593write(uint8_t addr, uint8_t pb, uint16_t val)
{
  int iStat;
  
  Wire1.beginTransmission(addr);
  Wire1.write(pb);
  Wire1.write((val >> 8) & 0xFF);
  Wire1.write(val & 0xFF);
  iStat = Wire1.endTransmission();
  return (iStat);
}

// Read from AD5593R
// returns -1 on any error
int AD5593readWord(uint8_t addr, uint8_t pb)
{
  int iStat;
  
  Wire1.beginTransmission(addr);
  Wire1.write(pb);
  iStat = Wire1.endTransmission();
  if(iStat != 0) return (-1);
  // Now read the data word
  int i = 0,j = 0;
  Wire1.requestFrom(addr, (uint8_t)2);
  while(Wire1.available())
  {
     if(i==0) j = Wire1.read() << 8;
     if(i==1) j |= Wire1.read();
     i++;
  }
  return(j);
}

int AD5593readADC(int8_t addr, int8_t chan)
{
   int iStat;
   
   // Select the ADC channel number
   if((iStat = AD5593write(addr, 0x02, (1 << chan))) != 0) return(-1);
   // Read the data and make sure the address is correct then left 
   // justify in 16 bits
   int i = AD5593readWord(addr, 0x40);
   if(((i >> 12) & 0x7) != chan) return(-1);
   i <<= 4;
   return(i & 0xFFFF);
}

int AD5593readADC(int8_t addr, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5593readADC(addr, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

int AD5593writeDAC(int8_t addr, int8_t chan, int val)
{
   uint16_t  d;
   // convert 16 bit DAC value into the DAC data data reg format
   d = (val>>4) | (chan << 12) | 0x8000;
   return(AD5593write(addr, 0x10 | chan, d));
}

// Reads the CPU ADC and averages
int readADC(int chan, int num)
{
  int j = 0;
  
  analogRead(chan);
  for(int i = 0;i<num;i++) j += analogRead(chan);
  return j / num;
}

// Mux select function, bits 0 - 7 are switch 0 - 7
void AD728mux(int8_t addr,int8_t msk)
{
  Wire1.beginTransmission(addr);
  Wire1.write(msk);
  Wire1.endTransmission();  
}

// Mux select function
void LTC13801mux(int8_t addr,int8_t chan)
{
  Wire1.beginTransmission(addr);
  Wire1.write(0x08 | chan);
  Wire1.endTransmission();  
}

// Read the TW readback value. Chan is 0 to 7
int readTWreadback(int chan)
{
  if((readOptionPins() & 0x2) == 0) LTC13801mux(0x49,chan);
  else AD728mux(MUXADD,1 << chan);
  delay(1);
  return readADC(VMUX,10);
}

void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// This function will send the FLASH saved setting to the serial port using the 
// active serial port. The data is converted to an ASCII hex block and sent using
// the protocol described above. The MIPS host app is designed to use this function.
// The FLASH  buffer is assumed to
// be binary and its contents are converted to hex and send. after the ACK is sent.
// After the ACK, the files size is sent as an ascii string with a EOL termination, then
// the data block is sent as ascii hex followed by a EOL, and finally the 8 bit CRC is
// sent as a byte followed by a EOL.
void FLASHtoSerial(void)
{
  char sbuf[3];
  byte by;
  byte crc=0;

  SendACK;
  // Send the filesize
  serial->println(sizeof(ARB_PARMS));
  // Send the data as hex
  for(int i=0; i<sizeof(ARB_PARMS); i++)
  {
    by = dueFlashStorage.readAbs((uint32_t)NonVolStorage + i);
    ComputeCRCbyte(&crc,by);
    sprintf(sbuf,"%02x",by);
    serial->print(sbuf);
  }
  serial->println("");
  // Send the CRC then exit
  serial->println(crc);
}

void SerialtoFLASH(void)
{
  char sbuf[3],*Token,c;
  int  addr,board,numBytes,val,crc;
  byte *buf;
  uint32_t start;

  SendACK;
  start = millis();
  // Receive the number of bytes
  while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
  sscanf(Token,"%d",&numBytes); 
  GetToken(true); // Get the \n and toss
  buf = new byte[numBytes];
  // Read the data block
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    sbuf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    sbuf[1] = c;
    sbuf[2] = 0;
    sscanf(sbuf,"%x",&val);
    buf[i] = val;
  }
  start = millis();
  // Now we should see an EOL, \n
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
  if(c == '\n')
  {
    // Get CRC and test
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    sscanf(Token,"%d",&crc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    if((Token[0] == '\n') && (crc == ComputeCRC(buf,numBytes)))
    {
       noInterrupts();
       if(dueFlashStorage.writeAbs((uint32_t)NonVolStorage, buf, numBytes))
       {
         interrupts();
         serial->println("FLASH data written!");
         SendACK;
         return;
      }
      interrupts();
    }
  }
  serial->println("Unable to write to FLASH!");
  return;
TimeoutS2F:
  serial->println("\nFLASH data receive from host timedout!");
  return;
}

// The function will program the FLASH memory by receiving a file from the USB connected host. 
// The file must be sent in hex and use the following format:
// First the FLASH address in hex and file size, in bytes (decimal) are sent. If the file can
// be burned to FLASH an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct and ACK is returned.
void ProgramFLASH(char * Faddress,char *Fsize)
{
  String sToken;
  uint32_t FlashAddress;
  int    numBytes,fi,val,tcrc;
  char   c,buf[3],*Token;
  byte   fbuf[256],b,crc=0;
  uint32_t start;
  
  FlashAddress = strtol(Faddress, 0, 16);
  sToken = Fsize;
  numBytes = sToken.toInt();
  SendACK;
  fi = 0;
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    fbuf[fi++] = val;
    ComputeCRCbyte(&crc,val);
    WDT_Restart(WDT);
    if(fi == 256)
    {
      fi = 0;
      // Write the block to FLASH
      noInterrupts();
      if(!dueFlashStorage.writeAbs((uint32_t)FlashAddress, fbuf, 256))
      {
        interrupts();
        serial->println("FLASH data write error!");
        SendNAK;
        return;
      }
      interrupts();
      FlashAddress += 256;
      serial->println("Next");
    }
  }
  // If fi is > 0 then write the last partial block to FLASH
  if(fi > 0)
  {
    noInterrupts();
    if(!dueFlashStorage.writeAbs((uint32_t)FlashAddress, fbuf, fi))
    {
      interrupts();
      serial->println("FLASH data write error!");
      SendNAK;
      return;
    }
    interrupts();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->println("File received from host and written to FLASH.");
       SendACK;
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  SendNAK;
  return;
TimeoutExit:
  serial->println("\nFile receive from host timedout!");
  SendNAK;
  return;
}
