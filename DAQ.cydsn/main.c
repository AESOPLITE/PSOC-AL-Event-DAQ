/* ========================================
 *
 * Robert P. Johnson
 * Copyright U.C. Santa Cruz, 2020
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF U.C. Santa Cruz.
 *
 * Code to run in the Event PSOC on the AESOP-Lite DAQ board.
 *
 * ========================================
 */
#include "project.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define VERSION 1

/*=========================================================================
 *
 * Calibration/PMT input connections, from left to right looking down at the end of the DAQ board:
 *              T3        G        T4        T1        T2
 * Connector  J10/12    J2/11    J17/18    J15/16    J25/26
 * Peak det.   p4[3]    p4[7]     p3[0]     p0[7]     p3[4]
 * Schem pin   11         17       13         7        20
 * ADC          2          2        1         2         1
 * ADC Chan     2          1        1         0         0
 * Preamp      p4[6]    p3[2]     p3[3]     p4[5]      p2[0]
 * Schem pin    9         16       15        14        19
 * Channel      2          1        4         3         5 
 * TOF                              2         1   
 * Trig bit     1         N/A       0         3         2
 *
 * The 4 PSOC DACs are labeled by channel number.
 * Note that T2 is the channel with the external 12-bit DAC for setting its threshold. The digital signal goes to p2[0].
 * T1 and T4 are connected to the two channels that go to the TOF chip.
 * The guard signal, G, does not participate in the trigger logic but is discriminated and registered in the data.
 *
 *  Event output format:
 *  Header "ZERO" in ASCII (5A, 45, 52, 4F)
 *  Event Header:
 *      - Run number 2 bytes
 *      - Event number 4 bytes (counts accepted triggers)
 *      - Trigger time stamp 4 bytes
 *      - Trigger count (including deadtime) 4 bytes
 *      - real time and date 4 bytes
 *      - Trigger status word 1 byte               
 *  PHA Data:
 *      - T1 2 bytes
 *      - T2 2 bytes
 *      - T3 2 bytes
 *      - T4 2 bytes
 *      - Guard 2 bytes
 *      - extra channel 2 bytes (can be removed once not needed)
 *  TOF Data:
 *      Time difference in units of 10 ps, 2 bytes, signed integer
 *  Tracker trigger count 2 bytes
 *  Tracker command count 1 byte
 *  Tracker trigger pattern 1 byte
 *  TOF debugging data 10 bytes (can be removed once not needed)
 *  Number of tracker boards 1 byte
 *  Tracker Data
 *  Trailer "FINI" in ASCII (46, 49, 4e, 49)
 *
 *  The Event PSOC can take commands from the USB-UART or main PSOC UART.    
 *    Each command is formatted as "S1234<sp>xyW" repeated 3 times, followed by <cr><lf>
 *    where 1234 are 4 ASCII characters, each representing a nibble 
 *    12 gives us the data byte and 34 the address byte             
 *    data byte: {7:0} gives the command code                 
 *    address byte: {7:6} and {1:0] give the number of data-byte "commands" to follow, 0 to 15
 *                  {5:2} = 0x8 indicate the event PSOC        
 *    All data arrive in up to 15 subsequent data-byte "commands" For those,
 *    bits {7:0} of the command byte are the data for the command in progress
 *    bits {7:6} and {1:0} give the data-byte number, 1 through 15
 *    Subsequent commands must wait until after the correct number of data bytes has arrived
 *
 *    The following commands are defined:
 *    Byte Code    Number Data Bytes            Data Byte Definition
 */

/* I2C mode */
#define ACK  (1u)
#define NACK (0u)
#define I2C_READ (1u)
#define I2C_WRITE (0u)

/* Default DAC threshold setting */
#define THRDEF (5u)

/* Timeout in 5 millisecond units when waiting for command completion */
#define TIMEOUT 200u 

/* Packet IDs */
#define FIX_HEAD ('\xDB')  // This one is no longer used, because the command echo was added to the data return
#define VAR_HEAD ('\xDC')

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow the usage of floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif

#define FIFO_LEN 100 
#define MXERR 64
#define MAX_CMD_DATA 16
#define TOFSIZE 17
#define TKRHOUSE_LEN 70
#define TOFMAX_EVT 64
#define MAX_TKR_BOARDS 8
#define MAX_TKR_BOARD_BYTES 203     // Two leading bytes, 12 bit header, 12 chips * (12-bit header and up to 10 12-bit cluster words) + CRC byte
#define USBFS_DEVICE (0u)
#define BUFFER_LEN  32u
#define MAX_DATA_OUT 256
#define MXERR 64
#define SPI_OUTPUT 0u
#define USBUART_OUTPUT 1u
#define CALMASK 1u
#define DATAMASK 2u
#define TRIGMASK 3u

#define ERR_DAC_LOAD 1u
#define ERR_DAC_READ 2u 
#define ERR_TOF_DAC_LOAD 3u
#define ERR_TOF_DAC_READ 4u
#define ERR_CMD_IGNORE 5u
#define ERR_TKR_READ_TIMEOUT 6u
#define ERR_TKR_BAD_ID 7u
#define ERR_TKR_BAD_LENGTH 8u
#define ERR_TKR_BAD_ECHO 9u
#define ERR_GET_TKR_DATA 10u
#define ERR_TKR_BAD_FPGA 11u
#define ERR_TKR_BAD_TRAILER 12u
#define ERR_TKR_BAD_NDATA 13u
#define ERR_PMT_DAQ_TIMEOUT 14u
#define ERR_TKR_NUM_BOARDS 15u
#define ERR_TKR_BAD_BOARD_ID 16u
#define ERR_TKR_BOARD_SHORT 17u
#define ERR_TKR_NO_MEMORY 18u
#define ERR_TX_FAILED 19u
#define ERR_BAD_CMD 20u
#define ERR_EVT_TOO_BIG 21u
#define ERR_BAD_BYTE 22u
#define ERR_TKR_BAD_STATUS 23u
#define ERR_TKR_TRG_ENABLE 24u
#define ERR_TKR_BAD_TRGHEAD 25u
#define ERR_TKR_TOO_BIG 26u
#define ERR_TKR_LYR_ORDER 27u
#define ERR_TRK_WRONG_DATA_TYPE 28u
#define ERR_CMD_BUF_OVERFLOW 29u

#define TKR_EVT_DATA 0xD3
#define TKR_HOUSE_DATA 0xC7
#define TKR_ECHO_DATA 0xF1

#define TKR_READ_TIMEOUT 31u    // Length of time to wait before giving a time-out error

uint8 nDataReady;
uint8 dataOut[MAX_DATA_OUT];      // Buffer for output data
uint16 tkrCmdCount;               // Command count returned from the Tracker
uint8 tkrCmdCode;                 // Command code echoed from the Tracker

// Register pointers for the power monitoring chips
const uint8 INA226_Config_Reg = 0x00;
const uint8 INA226_ShuntV_Reg = 0x01;
const uint8 INA226_BusV_Reg = 0x02;
const uint8 INA226_Power_Reg = 0x03;
const uint8 INA226_Current_Reg = 0x04;
const uint8 INA226_Calib_Reg = 0x05;
const uint8 INA226_Mask_Reg = 0x06;
const uint8 INA226_Alert_Reg = 0x07;

const uint8 I2C_Address_TMP100 = '\x48';
const uint8 TMP100_Temp_Reg = '\x00';
const uint8 I2C_Address_Barometer = '\x70';
const uint8 I2C_Address_RTC = '\x6F';

// Masks for DC control register
const uint8 LED1 = '\x01';
const uint8 LED2 = '\x02';
const uint8 TKRLED = '\x04'; 
const uint8 DATLED = '\x08';
const uint8 RSTPEAK = '\x10';

#define trgBit_T4 = 0x01;
#define trgBit_T3 = 0x02;
#define trgBit_T2 = 0x04;
#define trgBit_T1 = 0x08;

/* Bit definitions for the pulse control register */
#define PULSE_TOF_RESET 0x01
#define PULSE_TKR_TRIG 0x02
#define PULSE_LOGIC_RST 0x04
#define PULSE_CNTR_RST 0x08

// Slave addresses for the SPI interface
const uint8 SSN_Main = 1;
const uint8 SSN_TOF  = 2;

const uint8 triggerEnable_Mask = '\x01';
const uint8 SSN_Mask = '\xF3';

// Command codes for the TOF chip
const uint8 TOF_enable = 0x18;
const uint8 powerOnRESET = 0x30;
const uint8 writeConfig = 0x80;
const uint8 readConfig = 0x40;
const uint8 readResults =  0x60;   // Not used

RTC_1_TIME_DATE* timeDate;

// Circular buffer for commands from the Main PSOC
#define CMD_LENGTH 29u
#define MX_CMDS 32u
struct MainPSOCcmds {
    uint8 buf[CMD_LENGTH];
    uint8 nBytes;
} cmd_buffer[MX_CMDS];
uint8 cmdWritePtr, cmdReadPtr;

// TOF circular data buffers
struct TOF {
    uint32 shiftReg[TOFMAX_EVT];
    uint16 clkCnt[TOFMAX_EVT];
    bool filled[TOFMAX_EVT];
    uint8 ptr;
} tofA, tofB;
bool outputTOF;
uint32 tofA_sampleArray[3] = {0};
uint32 tofB_sampleArray[3] = {0};

// Temporary storage of Tracker housekeeping data
uint8 nTkrHouseKeeping;
uint8 tkrHouseKeepingFPGA;
uint8 tkrHouseKeepingCMD;
uint8 tkrHouseKeeping[TKRHOUSE_LEN];

uint32 timeStamp;
uint8 trgStatus;
bool triggered;

struct TkrData {
    uint16 triggerCount;
    uint8 cmdCount;
    uint8 trgPattern;              // bit 7 = non-bending; bit 6 = bending
    uint8 nTkrBoards;              // number of boards read out
    struct BoardHits {
        uint8 nBytes;              // number of bytes in the hit list
        uint8* hitList;            // pointer to the variable length hit list
    } boardHits[MAX_TKR_BOARDS];
} tkrData;
uint8 numTkrBrds;
bool readTracker;

struct Error {
    uint8 errorCode;
    uint8 value0;
    uint8 value1;
} errors[MXERR];
uint8 nErrors = 0;

uint32 clkCnt;
uint32 time() {
    //uint8 cnt200val = Counter_1_ReadCounter();
    uint8 cnt200val = Cntr8_Timer_ReadCount();
    return clkCnt + cnt200val;
}

char8 *parity[] = { "None", "Odd", "Even", "Mark", "Space" };
char8 *stop[] = { "1", "1.5", "2" };

uint16 ch1Count, ch1CountSave;
uint16 ch2Count, ch2CountSave;
uint16 ch3Count, ch3CountSave;
uint16 ch4Count, ch4CountSave;
uint16 ch5Count, ch5CountSave;
uint8 ch1CtrSave, ch2CtrSave, ch3CtrSave, ch4CtrSave, ch5CtrSave;
uint32 cntGO;
uint32 cntGO1;
uint16 runNumber;

/* Defines for DMA_1 and DMA_2 */
#define DMA_BYTES_PER_BURST 2
#define DMA_REQUEST_PER_BURST 1
#define DMA_NO_OF_SAMPLES 3
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_BASE)

void LED2_OnOff(bool on) {
    uint8 status = Control_Reg_LED_Read() & ~LED2;
    if (on) {
        Control_Reg_LED_Write(status | LED2);
    } else {
        Control_Reg_LED_Write(status);
    }
}

void addError(uint8 code, uint8 val1, uint8 val2) {
    if (nErrors < MXERR) {
        errors[nErrors].errorCode = code;
        errors[nErrors].value0 = val1;
        errors[nErrors].value1 = val2;
        nErrors++;
    }
}

// Load a single I2C byte register
uint8 loadI2Creg(uint8 I2C_Address, uint8 regAddress, uint8 regValue) {

    // Send the start byte, with address
    uint8 rc = I2C_2_MasterSendStart(I2C_Address,I2C_WRITE);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the register address
    rc = I2C_2_MasterWriteByte(regAddress);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the register value
    rc = I2C_2_MasterWriteByte(regValue);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the stop condition
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    /* Wait for the data transfer to complete */
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }  
    
    return 0;    
}

// Read bytes from an I2C chip register
uint8 readI2Creg(int nBytes, uint8 I2C_Address, uint8 regAddress, uint8 regValue[]) {
   
    // Send the start byte, with address and control set to write
    uint8 rc = I2C_2_MasterSendStart(I2C_Address,I2C_WRITE);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the register address
    rc = I2C_2_MasterWriteByte(regAddress);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;

    // Send the stop condition
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;    

    // Wait for the data transfer to complete 
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }      
    
    // Send the start byte, with address and control set to read
    rc = I2C_2_MasterSendStart(I2C_Address,I2C_READ);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    for (int i=0; i<nBytes-1; ++i) {
        regValue[i] = I2C_2_MasterReadByte(ACK);
    }
    regValue[nBytes-1] = I2C_2_MasterReadByte(NACK);
        
    // Send the stop condition
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    /* Wait for the data transfer to complete */
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }  
    
    return 0;    
}

// Load the AD5622 DAC
uint8 loadDAC(uint8 I2C_Address, uint16 voltage) {
    
    uint8 nib0 = (voltage & 0x00FF);
    uint8 nib1 = (voltage & 0x0F00)>>8;
    
    uint8 rc;
    
    rc = I2C_2_MasterSendStart(I2C_Address,I2C_WRITE);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    rc = I2C_2_MasterWriteByte(nib1);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    rc = I2C_2_MasterWriteByte(nib0);
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    /* Wait for the data transfer to complete */
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }  
    
    return 0;
} // end of loadDAC

// Read back the setting from the AD5622 DAC and return it
uint8 readDAC(uint8 I2C_Address, uint16* rvalue) {
           
    uint8 bytes[2];
    //I2C_2_MasterClearReadBuf();
    //uint8 rc = I2C_2_MasterReadBuf(I2C_Address, bytes, 2, I2C_2_MODE_COMPLETE_XFER);
    uint8 rc = I2C_2_MasterSendStart(I2C_Address,I2C_READ);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;    

    bytes[0] = I2C_2_MasterReadByte(ACK);
    bytes[1] = I2C_2_MasterReadByte(NACK);

    rc = I2C_2_MasterSendStop();
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;

    *rvalue = ((uint16)(bytes[0] & '\x3F'))<<6;
    *rvalue = *rvalue | ((uint16)(bytes[1] & '\xFC')>>2);
    
    rc = I2C_2_MasterStatus();    
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    } 
    return 0;
}

bool isTriggerEnabled() {
    uint8 regValue = Control_Reg_Trg_Read();
    return (regValue & triggerEnable_Mask);
}

void setPeakDetResetWait(uint8 waitTime) {
    Count7_3_WritePeriod(waitTime);
}

void setTriggerMask(char trigger, uint8 mask) {
    mask = mask & 0x0F;
    if (trigger == 'e') {
        Control_Reg_Trg1_Write(mask);
    } else if (trigger == 'p') {
        Control_Reg_Trg2_Write(mask);
    }
}

uint8 getTriggerMask(char trigger) {
    if (trigger == 'e') return Control_Reg_Trg1_Read();
    else if (trigger == 'p') return Control_Reg_Trg2_Read();
    return 0;
}

// Control of the SPI slave address. The slave select is active low.
// **** Note that the TOF chip needs to go high, for reset, before each SPI transaction. ****
void set_SPI_SSN(uint8 SSN, bool clearBuffer) {
    // SSN = SSN_Main = 1 for main PSOC
    // SSN = SSN_TOF  = 2 for TOF chip
    // SSN = 0 (or anything else) to deselect all slaves
    while (!(SPIM_ReadTxStatus() & SPIM_STS_SPI_IDLE));
    //uint8 regValue = Control_Reg_SSN_Read() & SSN_Mask;
    if (SSN == SSN_Main) {
        Control_Reg_SSN_Write(0xFE);    // The SPI chip select is active low
    } else if (SSN==SSN_TOF) {
        Control_Reg_SSN_Write(0xFF);    // This resets the TOF chip to get it ready for a transaction
        Control_Reg_SSN_Write(0xFD);    // The SPI chip select is active low
    } else {
        Control_Reg_SSN_Write(0xFF);
    }

    if (clearBuffer) SPIM_ClearTxBuffer();
}

// Control of the trigger enable bit
void triggerEnable(bool enable) {
    uint8 regValue = Control_Reg_Trg_Read() & ~triggerEnable_Mask;
    if (enable) {
        // Reset the TOF chip time reference. It also gets reset every 5 ms by interrupt.
        //Control_Reg_Pls_Write(PULSE_TOF_RESET);
        
        // Enable both TOF chip channels
        //set_SPI_SSN(SSN_TOF, true);
        //SPIM_WriteTxData(writeConfig+1);
        //SPIM_WriteTxData(0x05);
        
        // Enable the master trigger
        Control_Reg_Trg_Write(regValue | triggerEnable_Mask);  
    } else {
        // Disable the master trigger
        Control_Reg_Trg_Write(regValue);
        
        // Stop the TOF chip acquisition by disabling both channels
        //set_SPI_SSN(SSN_TOF, true);
        //SPIM_WriteTxData(writeConfig+1);
        //SPIM_WriteTxData(0x00);
    }
}

uint8 byte32(uint32 word, int byte) {
    const uint32 mask[4] = {0xFF000000, 0x00FF0000, 0x0000FF00, 0x000000FF};
    return (uint8)((word & mask[byte]) >> (3-byte)*8);
}
uint8 byte16(uint16 word, int byte) {
    const uint16 mask[2] = {0xFF00, 0x00FF};
    return (uint8)((word & mask[byte]) >> (1-byte)*8);
}

void logicReset() {
    int InterruptState = CyEnterCriticalSection();
    LED2_OnOff(true);
    int state = isr_clk200_GetState();
    isr_clk200_Disable();
    int stateTrg = isr_GO1_GetState();
    isr_GO1_Disable();
    clkCnt = 0;             
    ch1Count = 0;
    ch2Count = 0;
    ch3Count = 0;
    ch4Count = 0;
    ch5Count = 0;
    cntGO = 0;
    cntGO1 = 0;
    Control_Reg_Pls_Write(PULSE_LOGIC_RST);
    Control_Reg_Pls_Write(PULSE_CNTR_RST);
    CyDelay(20);
    if (stateTrg) isr_GO1_Enable();
    if (state) isr_clk200_Enable();
    LED2_OnOff(false);
    for (int brd=0; brd<MAX_TKR_BOARDS; ++brd) {
        if (tkrData.boardHits[brd].nBytes > 0) {
            tkrData.boardHits[brd].nBytes = 0;
            free(tkrData.boardHits[brd].hitList);
        }
    }
    CyExitCriticalSection(InterruptState);
}

// Get a byte of data from the Tracker UART, with a time-out in case nothing is coming.
// The second argument (flag) helps to identify where a timeout error originated.
uint8 tkr_getByte(uint32 startTime, uint8 flag) {
    while (!(UART_TKR_ReadRxStatus() & UART_TKR_RX_STS_FIFO_NOTEMPTY)) {
        uint32 timeElapsed = time() - startTime;
        if (timeElapsed > TKR_READ_TIMEOUT) {
            uint8 temp = (uint8)(timeElapsed & 0x000000ff);
            addError(ERR_TKR_READ_TIMEOUT, temp, flag);
            return 0x00;
        }
    }
    return UART_TKR_ReadRxData();
}

// Function to receive ASIC register data from the Tracker
void getASICdata() {
    uint32 startTime = time();
    nDataReady = tkr_getByte(startTime, 69);
    dataOut[0] = nDataReady;
    nDataReady++;
    for (int i=1; i<nDataReady; ++i) {
        startTime = time();
        dataOut[i] = tkr_getByte(startTime, 70+i);
    }
}

// Function to receive i2c register data from the Tracker
void getTKRi2cData() {
    uint32 startTime = time();
    nDataReady = 4;
    dataOut[0] = tkr_getByte(startTime, 0x89);
    dataOut[1] = tkr_getByte(startTime, 0x90);
    dataOut[2] = tkr_getByte(startTime, 0x91);
    dataOut[3] = tkr_getByte(startTime, 0x92);
}

// Receive trigger-primitive and TOT data from the tracker, for calibration-pulse events only
int getTrackerBoardTriggerData(uint8 FPGA) {
    int rc = 0;
    uint32 startTime = time();
    // Ignore the first byte, which is rubbish (not sure why. . .)
    uint8 theByte = tkr_getByte(startTime, 0x44);
    // The first good byte received encodes the FPGA address, so we check it here:
    theByte = tkr_getByte(startTime, 0x45);
    uint8 fpgaRet = (theByte & 0x38)>>3;
    if (fpgaRet != FPGA) {
        addError(ERR_TKR_BAD_TRGHEAD, FPGA, fpgaRet);
        rc = 1;
    }
    nDataReady = 9;   
    dataOut[0] = theByte;
    // Read in the other 8 bytes
    for (int i=1; i<nDataReady; ++i) {
        dataOut[i] = tkr_getByte(startTime, 0x46);
    }
    return rc;
}

// Function to get a full data packet from the Tracker.
int getTrackerData(uint8 idExpected) {
    int rc = 0;
    uint32 startTime = time();
    uint8 len = tkr_getByte(startTime, 1);
    uint8 IDcode = tkr_getByte(startTime, 2);
    if (IDcode != idExpected) {
        if (idExpected != 0) {
            addError(ERR_TRK_WRONG_DATA_TYPE, IDcode, idExpected);
            if (idExpected == TKR_EVT_DATA) {
                tkrData.triggerCount = 0;  // Send back a packet that won't cause a crash down the road
                tkrData.cmdCount = 0;
                tkrData.trgPattern = 0;
                tkrData.nTkrBoards = numTkrBrds;
                for (int brd=0; brd<numTkrBrds; ++brd) {  // Make a default empty ASIC hit list
                    tkrData.boardHits[brd].nBytes = 5;
                    tkrData.boardHits[brd].hitList = (uint8*) malloc(5);
                    if (tkrData.boardHits[brd].hitList != NULL) {
                        tkrData.boardHits[brd].hitList[0] = 0xE7;
                        tkrData.boardHits[brd].hitList[1] = brd;
                        tkrData.boardHits[brd].hitList[2] = 0;
                        tkrData.boardHits[brd].hitList[3] = 0x01;
                        tkrData.boardHits[brd].hitList[4] = 0x30;
                    }
                }
                return 54;
            }
        } else {
            if (IDcode == TKR_EVT_DATA) {
                addError(ERR_TRK_WRONG_DATA_TYPE, IDcode, idExpected);
                return 53;
            }
        }
    }
    if (IDcode == TKR_EVT_DATA) {         // Event data
        if (len != 5) {           // Formal check
            addError(ERR_TKR_BAD_LENGTH, IDcode, len);
            tkrData.triggerCount = 0;  // Send back a packet that won't cause a crash down the road
            tkrData.cmdCount = 0;
            tkrData.trgPattern = 0;
            tkrData.nTkrBoards = numTkrBrds;
            for (int brd=0; brd<numTkrBrds; ++brd) {  // Make a default empty ASIC hit list
                tkrData.boardHits[brd].nBytes = 5;
                tkrData.boardHits[brd].hitList = (uint8*) malloc(5);
                if (tkrData.boardHits[brd].hitList != NULL) {
                    tkrData.boardHits[brd].hitList[0] = 0xE7;
                    tkrData.boardHits[brd].hitList[1] = brd;
                    tkrData.boardHits[brd].hitList[2] = 0;
                    tkrData.boardHits[brd].hitList[3] = 0x02;
                    tkrData.boardHits[brd].hitList[4] = 0x30;
                }
            }
            return 55;
        }
        tkrData.triggerCount = ((uint16)tkr_getByte(startTime, 3) & 0x00FF) << 8;
        tkrData.triggerCount = tkrData.triggerCount | ((uint16)tkr_getByte(startTime, 4) & 0x00FF);
        tkrData.cmdCount = tkr_getByte(startTime, 5);
        uint8 nBoards = tkr_getByte(startTime, 6);
        tkrData.trgPattern = nBoards & 0xC0;
        nBoards = nBoards & 0x3F;
        if (nBoards != numTkrBrds) {
            addError(ERR_TKR_NUM_BOARDS, nBoards, tkrData.trgPattern);
            nBoards = numTkrBrds;
            tkrData.nTkrBoards = nBoards;
            for (int brd=0; brd<nBoards; ++brd) {  // Make a default empty ASIC hit list
                tkrData.boardHits[brd].nBytes = 5;
                tkrData.boardHits[brd].hitList = (uint8*) malloc(5);
                if (tkrData.boardHits[brd].hitList != NULL) {
                    tkrData.boardHits[brd].hitList[0] = 0xE7;
                    tkrData.boardHits[brd].hitList[1] = brd;
                    tkrData.boardHits[brd].hitList[2] = 0;
                    tkrData.boardHits[brd].hitList[3] = 0x03;
                    tkrData.boardHits[brd].hitList[4] = 0x30;
                }
            }
            return 56;
        }
        tkrData.nTkrBoards = nBoards;
        for (int brd=0; brd < nBoards; ++brd) {
            uint8 nBrdBytes = tkr_getByte(startTime, 7);  // Length of the hit list, in bytes
            if (nBrdBytes < 4) {
                addError(ERR_TKR_BOARD_SHORT, nBrdBytes, brd);
                tkrData.boardHits[brd].nBytes = 5;
                tkrData.boardHits[brd].hitList = (uint8*) malloc(5);
                if (tkrData.boardHits[brd].hitList != NULL) {
                    tkrData.boardHits[brd].hitList[0] = 0xE7;
                    tkrData.boardHits[brd].hitList[1] = brd;
                    tkrData.boardHits[brd].hitList[2] = 0;
                    tkrData.boardHits[brd].hitList[3] = 0x04;
                    tkrData.boardHits[brd].hitList[4] = 0x30;
                }
                rc = 57;
                continue;
            }
            uint8 IDbyte = tkr_getByte(startTime, 8);     // Hit list identifier, should always be 11100111
            if (IDbyte != 0xE7) {
                addError(ERR_TKR_BAD_BOARD_ID, IDbyte, brd);
                tkrData.boardHits[brd].nBytes = 5;
                tkrData.boardHits[brd].hitList = (uint8*) malloc(5);
                if (tkrData.boardHits[brd].hitList != NULL) {
                    tkrData.boardHits[brd].hitList[0] = 0xE7;
                    tkrData.boardHits[brd].hitList[1] = brd;
                    tkrData.boardHits[brd].hitList[2] = 0;
                    tkrData.boardHits[brd].hitList[3] = 0x05;
                    tkrData.boardHits[brd].hitList[4] = 0x30;
                }
                rc = 58;
                continue;
            }
            uint8 byte2 = tkr_getByte(startTime, 9);        // Byte containing the board address
            if (byte2 > 8) {   // Formal check. Note that 8 denotes the master board, which really is layer 0
                addError(ERR_TKR_BAD_FPGA, byte2, brd);
                rc = 59;
            }
            uint8 lyr = 0x7 & byte2;  // Get rid of the master bit, leaving just the layer number
            // Require the boards to be set up to read out in order:
            if (lyr != brd) addError(ERR_TKR_LYR_ORDER, lyr, brd);
            if (nBrdBytes > MAX_TKR_BOARD_BYTES) {    // This really should never happen, due to ASIC 10-hit limit
                tkrData.boardHits[lyr].nBytes = MAX_TKR_BOARD_BYTES;
                addError(ERR_TKR_TOO_BIG, nBrdBytes, lyr);
            } else {
                tkrData.boardHits[lyr].nBytes = nBrdBytes;
            }
            tkrData.boardHits[lyr].hitList = (uint8*) malloc(nBrdBytes);
            if (tkrData.boardHits[lyr].hitList == NULL) {
                addError(ERR_TKR_NO_MEMORY, nBrdBytes-2, brd);
                rc = 60;
                continue;
            }
            tkrData.boardHits[lyr].hitList[0] = IDbyte;
            tkrData.boardHits[lyr].hitList[1] = byte2;
            for (int i=2; i<nBrdBytes; ++i) {
                uint8 theByte = tkr_getByte(startTime, 10);
                if (i<MAX_TKR_BOARD_BYTES) {       
                    tkrData.boardHits[lyr].hitList[i] = theByte;
                }
            }
        }
    } else if (IDcode == TKR_HOUSE_DATA) {  // Housekeeping data
        uint8 nData = tkr_getByte(startTime, 11);
        if (len != nData+6) {   // Formal check
            addError(ERR_TKR_BAD_NDATA, len, nData);
            nData = len - 6;
        }
        tkrCmdCount = (uint16)(tkr_getByte(startTime, 12)) << 8;
        tkrCmdCount = (tkrCmdCount & 0xFF00) | (uint16)tkr_getByte(startTime, 13);
        tkrHouseKeepingFPGA = tkr_getByte(startTime, 14);
        if (tkrHouseKeepingFPGA > 8) {   // Formal check
            addError(ERR_TKR_BAD_FPGA, tkrCmdCode, tkrHouseKeepingFPGA);
        }
        uint8 tkrHouseKeepingCMD = tkr_getByte(startTime, 15);
        if (tkrHouseKeepingCMD != tkrCmdCode) {   // Formal check
            addError(ERR_TKR_BAD_ECHO, tkrHouseKeepingCMD, tkrCmdCode);
        }
        nTkrHouseKeeping = 0;      // Overwrite any old data, even if it was never sent out.
        for (int i=0; i<nData; ++i) {
            uint8 tmpData = tkr_getByte(startTime, 16);
            if (i < TKRHOUSE_LEN) {
                tkrHouseKeeping[i] = tmpData;
                nTkrHouseKeeping++;
            }
        }
        if (tkrHouseKeeping[nTkrHouseKeeping-1] != 0x0F) {    // Formal check
            addError(ERR_TKR_BAD_TRAILER, tkrCmdCode, tkrHouseKeeping[nTkrHouseKeeping-1]);           
        }
    } else if (IDcode == TKR_ECHO_DATA) {  // Command Echo
        if (len != 4) {           // Formal check
            addError(ERR_TKR_BAD_LENGTH, IDcode, len);
        }
        nDataReady = 3;
        dataOut[0] = tkr_getByte(startTime, 17);
        tkrCmdCount = (uint16)dataOut[0] << 8;
        dataOut[1] = tkr_getByte(startTime, 18);
        tkrCmdCount = (tkrCmdCount & 0xFF00) | dataOut[1];
        uint8 tkrCmdCodeEcho = tkr_getByte(startTime, 19);
        dataOut[2] = tkrCmdCodeEcho;
        if (tkrCmdCode != tkrCmdCodeEcho) {
            addError(ERR_TKR_BAD_ECHO, tkrCmdCodeEcho, tkrCmdCode);
            rc = 1;
        }
    } else {    // WTF?!?   Not sure what to do with this situation, besides flag it.
        if (nErrors < MXERR) {
            addError(ERR_TKR_BAD_ID, IDcode, len);
        }
        nDataReady = len;
        if (len > 15) len = 15;
        for (int i=0; i<nDataReady; ++i) {
            dataOut[i] = tkr_getByte(startTime,20+i);
        }       
    }
    return rc;
}

void tkrLED(bool on) {
    if (on) {
        uint8 status = Control_Reg_LED_Read() & ~TKRLED;
        Control_Reg_LED_Write(status | TKRLED);
    } else {
        Timer_1_Start();
    }
}

// Function to send a command to the tracker that has no data bytes
void sendSimpleTrackerCmd(uint8 FPGA, uint8 code) {
    tkrLED(true);
    tkrCmdCode = code;
    UART_TKR_PutChar(FPGA);         // FPGA address
    UART_TKR_PutChar(tkrCmdCode);   // Command code
    UART_TKR_PutChar(0);            // No data bytes

    // Wait around for the data to transmit
    uint32 tStart = time();
    while (UART_TKR_GetTxBufferSize() > 0) {                                           
        if (time() - tStart > 200) {
            addError(ERR_TX_FAILED, code, 0xFF);
            tkrLED(false);
            return;
        }
    }                                
    // Now look for the echo coming back from the Tracker.
    if (tkrCmdCode != 0x67 && tkrCmdCode != 0x6C) {
        int rc = getTrackerData(TKR_ECHO_DATA);
        if (rc != 0) {
            addError(ERR_GET_TKR_DATA, rc, tkrCmdCode);
        }
    }
    nDataReady = 0;  // Suppress the echo from being send out to the world
    tkrLED(false);
}

void resetAllTrackerLogic() {
    for (uint8 brd=0; brd<numTkrBrds; ++brd) {
        sendSimpleTrackerCmd(brd, 0x04);
    }
}

// Read the ASIC configuration register 
void readASICconfig(uint8 FPGA, uint8 chip) {
    tkrLED(true);
    tkrCmdCode = 0x22; // Config read command code 
    UART_TKR_PutChar(FPGA);   // FPGA address
    UART_TKR_PutChar(tkrCmdCode);   
    UART_TKR_PutChar(1);      // One data byte
    UART_TKR_PutChar(chip);   // ASIC address for the data byte

    // Wait around for the data to transmit
    uint32 tStart = time();
    while (UART_TKR_GetTxBufferSize() > 0) {                                           
        if (time() - tStart > 200) {
            addError(ERR_TX_FAILED, tkrCmdCode, 0xEE);
            tkrLED(false);
            return;
        }
    }                                
    // Now look for the housekeeping data coming back from the Tracker.
    getASICdata();
    tkrLED(false);
}

void calibrateInputTiming(uint8 FPGA) {
    sendSimpleTrackerCmd(FPGA, 0x81);      // Tell the input circuits to self calibrate
    CyDelay(1);
    for (uint8 chip=0; chip<12; ++chip) {  // Each ASIC communication path to the FPGA gets calibrated in turn
        for (uint8 i=0; i<5; ++i) {        // Read the configuration register several times to provide data transitions for calibration
            readASICconfig(FPGA, chip);
            nDataReady = 0;          // Throw away the resulting data so it doesn't get out to the world
        }
    }
    CyDelay(2);
    sendSimpleTrackerCmd(FPGA, 0x82);      // Tell the input circuit to set its delay to the calibrated value
}

// Calibrate the delays for capturing data in the FPGAs coming from the 12 ASICs on each board.
void calibrateAllInputTiming() {
    for (int brd=0; brd<numTkrBrds; ++brd) {
        calibrateInputTiming(brd);
    }
}

uint8 ptrNext(uint8 ptr) {
    if (ptr < MX_CMDS-1) {
        return ptr + 1;
    } else {
        return 0;
    }
}

// Read out the shift register when a TOF stop event arrives for channel A
CY_ISR(Store_A)
{
    if (ShiftReg_A_GetIntStatus() == ShiftReg_A_STORE) {
        while (ShiftReg_A_GetFIFOStatus(ShiftReg_A_OUT_FIFO) != ShiftReg_A_RET_FIFO_EMPTY) {
            uint32 AT = ShiftReg_A_ReadData();
            tofA.shiftReg[tofA.ptr] = AT;
            tofA.clkCnt[tofA.ptr] = (uint16)time();
            tofA.filled[tofA.ptr] = true;
            tofA.ptr++;
            if (tofA.ptr >= TOFMAX_EVT) tofA.ptr = 0;
            if (outputTOF) {    // Send data directly to the PC for this special debugging mode
                uint8 oReg[7];
                oReg[0] = 0xAA;
                oReg[1] = (AT & 0x0000FF00)>>8;
                oReg[2] =  AT & 0x000000FF;
                oReg[3] = (AT & 0xFF000000)>>24;
                oReg[4] = (AT & 0x00FF0000)>>16;            
                uint16 clk16 = (uint16)time();
                oReg[5] = (uint8)((clk16 & 0xFF00)>>8);
                oReg[6] = (uint8)(clk16 & 0x00FF);             
                //while(USBUART_CDCIsReady() == 0u);
                USBUART_PutData(oReg,7);
            }
        }
    }
}

// Read out the shift register when a TOF stop event arrives for channel B
CY_ISR(Store_B)
{
    if (ShiftReg_B_GetIntStatus() == ShiftReg_B_STORE) { 
        while (ShiftReg_B_GetFIFOStatus(ShiftReg_B_OUT_FIFO) != ShiftReg_B_RET_FIFO_EMPTY) {
            uint32 BT = ShiftReg_B_ReadData();
            tofB.shiftReg[tofB.ptr] = BT;
            tofB.clkCnt[tofB.ptr] = (uint16)time();
            tofB.filled[tofB.ptr] = true;
            tofB.ptr++;
            if (tofB.ptr >= TOFMAX_EVT) tofB.ptr = 0;
            if (outputTOF) {  // Send data directly to the PC for this special debugging mode
                Timer_1_Start();
                uint8 oReg[7];
                oReg[0] = 0xBB;
                oReg[1] = (BT & 0x0000FF00)>>8;
                oReg[2] =  BT & 0x000000FF;
                oReg[3] = (BT & 0xFF000000)>>24;
                oReg[4] = (BT & 0x00FF0000)>>16;
                uint16 clk16 = (uint16)time();
                oReg[5] = (uint8)((clk16 & 0xFF00)>>8);
                oReg[6] = (uint8)(clk16 & 0x00FF);                  
                //while(USBUART_CDCIsReady() == 0u);
                USBUART_PutData(oReg,7);
            }
        }
    }
}

// Turn off an LED once the interval timer has timed out (this is just to make the LED stay on long enough to be visible)
CY_ISR(intTimer) {
    uint8 status = Control_Reg_LED_Read();
    status = status & ~DATLED;
    status = status & ~TKRLED;
    //status = status & ~LED2;
    Control_Reg_LED_Write(status);
    Timer_1_Stop();
}

// Increment the internal clock count every second, and also make the LED blink
CY_ISR(clk200) {       // Interrupt every second (200 ticks of the 5ms period clock)
    clkCnt += 200;     // Increment the clock counter used for time stamps
    uint8 status = Control_Reg_LED_Read();
    uint8 blink = status & LED1;
    if (blink == 0x00) blink = LED1;
    else blink = 0x00;
    status = (status & ~LED1) | blink;
    Control_Reg_LED_Write(status);
}

// Interrupts to keep count of PMT singles rates. These interrupt every time the 8-bit hardware counter turns over.
CY_ISR(isrCh1)
{
    ch1Count++;
}

CY_ISR(isrCh2)
{
    ch2Count++;
}

CY_ISR(isrCh3)
{
    ch3Count++;
}

CY_ISR(isrCh4)
{
    ch4Count++;
}

CY_ISR(isrCh5)
{
    ch5Count++;
}

// Receive and store commands from the Main PSOC via the UART
CY_ISR(isrUART) {
    while (UART_CMD_GetRxBufferSize() > 0 && cmdWritePtr != cmdReadPtr) {
        cmd_buffer[cmdWritePtr].buf[cmd_buffer[cmdWritePtr].nBytes] = UART_CMD_GetByte();
        cmd_buffer[cmdWritePtr].nBytes++;
        if (cmd_buffer[cmdWritePtr].nBytes == CMD_LENGTH) {  // This command is fully received
            if (cmdReadPtr >= MX_CMDS) cmdReadPtr = cmdWritePtr;
            cmdWritePtr = ptrNext(cmdWritePtr);
            if (cmdWritePtr == cmdReadPtr) {  // This will almost surely make a mess if it happens!
                addError(ERR_CMD_BUF_OVERFLOW, cmdWritePtr, 0);
            } else {
                cmd_buffer[cmdWritePtr].nBytes = 0;
            }
        }
    }
}

// GO signal (system trigger). Start the full event readout if trigger is enabled.
CY_ISR(isrGO1)    
{
    if (isTriggerEnabled()) {
        // Disable the trigger until the event readout has been completed
        triggerEnable(false);
        trgStatus = Status_Reg_Trg_Read();
        cntGO++;
        triggered = true;
        timeStamp = time();            
        LED2_OnOff(true);
        Timer_1_Start();
    }
    cntGO1++;     // Count all GO signals during a run, even if the trigger is not enabled.
    
    // At this point execution returns to its normal flow, allowing other interrupts. The remainder of the
    // event readout process is done in main(), in the infinite for loop.
}

void dataLED(bool on) {
    uint8 status;
    if (on) {
        status = Control_Reg_LED_Read() & ~DATLED;
        Control_Reg_LED_Write(status | DATLED);
    } else {
        Timer_1_Start();
    }
}

void setCoincidenceWindow(uint8 dt) {
    TrigWindow_V1_1_Count7_1_WritePeriod(dt);
    TrigWindow_V1_2_Count7_1_WritePeriod(dt);
    TrigWindow_V1_3_Count7_1_WritePeriod(dt);
    TrigWindow_V1_4_Count7_1_WritePeriod(dt);
    TrigWindow_V1_5_Count7_1_WritePeriod(dt);
}

int main(void)
{     
    uint8 outputMode = USBUART_OUTPUT;    
    triggered = false;
    tkrData.nTkrBoards = 0;
    tofA.ptr = 0;
    tofB.ptr = 0;
    outputTOF = false;
    for (int i=0; i<TOFMAX_EVT; ++i) {
        tofA.filled[i] = false;
        tofB.filled[i] = false;
    }
    
    nDataReady = 0;
    clkCnt = 0;
    nTkrHouseKeeping = 0;
    readTracker = false;
    
    runNumber = 0;
    timeStamp = time();
    
    uint8 buffer[BUFFER_LEN];  // Buffer for incoming UART commands
    uint8 code[256];  // ASCII code translation to hex nibbles
    for (int i=0; i<256; ++i) code[i] = 0;
    code[49] = 1;
    code[50] = 2;
    code[51] = 3;
    code[52] = 4;
    code[53] = 5;
    code[54] = 6;
    code[55] = 7;
    code[56] = 8;
    code[57] = 9;
    code[65] = 10;
    code[97] = 10;   // not case sensitive for ABCDEF
    code[66] = 11;
    code[98] = 11;
    code[67] = 12;
    code[99] = 12;
    code[68] = 13;
    code[100] = 13;
    code[69] = 14;
    code[101] = 14;
    code[70] = 15;
    code[102] = 15;
    
    // Buffer for output of a 3-byte data packet. Set the invariant parts of the header and trailer bytes.
    uint8 dataPacket[9];
    dataPacket[0] = VAR_HEAD;
    dataPacket[1] = '\x00';
    dataPacket[2] = '\xFF';
    dataPacket[6] = '\xFF';
    dataPacket[7] = '\x00';
    dataPacket[8] = '\xFF';
    uint8 Padding[2];
    Padding[0] = '\x01';
    Padding[1] = '\x02';
    
    // General hardware logic reset (not including the tracker), and reset of counters
    logicReset();
    
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Initialize interrupts */
    isr_timer_StartEx(intTimer);
    isr_timer_Disable();
    isr_clk200_StartEx(clk200);
    isr_clk200_Disable();
    isr_Store_A_StartEx(Store_A);
    isr_Store_A_Disable();
    isr_Store_B_StartEx(Store_B);
    isr_Store_B_Disable();
    isr_Ch1_StartEx(isrCh1);
    isr_Ch1_Disable();
    isr_Ch2_StartEx(isrCh2);
    isr_Ch2_Disable();
    isr_Ch3_StartEx(isrCh3);
    isr_Ch3_Disable();
    isr_Ch4_StartEx(isrCh4);
    isr_Ch4_Disable();
    isr_Ch5_StartEx(isrCh5);
    isr_Ch5_Disable();
    isr_GO1_StartEx(isrGO1);
    isr_GO1_Disable();
    isr_UART_StartEx(isrUART);
    isr_UART_Disable();
    
    cmdReadPtr = 255;
    cmdWritePtr = 0;
    for (uint i=0; i<MX_CMDS; ++i) cmd_buffer[i].nBytes = 0;
    
    /* Start up the various hardware components */
    I2C_2_Start();
    ShiftReg_1_Start();
    
    // Counters for loading TOF shift registers. The periods are set in the schematic and should never change!
    Count7_1_Start();
    Count7_2_Start();
    
    // Set up the counter used for timing. It counts a 200 Hz clock derived from the watch crystal, and every 200 counts
    // i.e. once each second, it interrupts the CPU, which then increments a 1 Hz count. The time() function adds the two
    // counts together to get a time tag that increments every 5 ms. Note that the main purpose of the 200 Hz clock is to
    // send a hardware reset to the time-of-flight chip every 5 ms, so that we know exactly when its counting starts.
    Cntr8_Timer_WritePeriod(200);
    
    // Counter for the delay time to wait before resetting the peak detectors.
    Count7_3_Start();
    // The peak detector output takes about 4us to settle down after its upward swing, so at 12MHz this should be at least 48 ticks to set
    // the time to start digitizing. This also affects wait times to sent peak detector resets and start looking for new triggers.
    setPeakDetResetWait(72);

    // TOF shift registers
    ShiftReg_A_Start();
    ShiftReg_B_Start();
    
    SPIM_Start();
    
    USBUART_Start(USBFS_DEVICE, USBUART_3V_OPERATION);
    
    Comp_Ch1_Start();
    Comp_Ch2_Start();
    Comp_Ch3_Start();
    Comp_Ch4_Start();
    
    // Internal and external voltage DACs
    uint8 thrDACsettings[] = {THRDEF, THRDEF, THRDEF, THRDEF};
    VDAC8_Ch1_Start();
    VDAC8_Ch1_SetValue(THRDEF);   // This is in DAC counts, 4 mV/bit
    VDAC8_Ch2_Start();
    VDAC8_Ch2_SetValue(THRDEF);
    VDAC8_Ch3_Start();
    VDAC8_Ch3_SetValue(THRDEF);
    VDAC8_Ch4_Start();
    VDAC8_Ch4_SetValue(THRDEF);
    const uint8 I2C_Address_DAC_Ch5 = '\x0E';
    loadDAC(I2C_Address_DAC_Ch5, 0x000F);
    const uint8 I2C_Address_TOF_DAC1 = '\x0C';
    loadDAC(I2C_Address_TOF_DAC1, 0x00FF);
    const uint8 I2C_Address_TOF_DAC2 = '\x0F';
    loadDAC(I2C_Address_TOF_DAC2, 0x00FF);
    
    ADC_SAR_1_Start();
    ADC_SAR_2_Start();
 
    UART_TKR_Start();
    UART_CMD_Start();

    // Start counters buried inside of the edge detectors for the trigger inputs
    TrigWindow_V1_1_Count7_1_Start();
    TrigWindow_V1_2_Count7_1_Start();
    TrigWindow_V1_3_Count7_1_Start();
    TrigWindow_V1_4_Count7_1_Start();
    TrigWindow_V1_5_Count7_1_Start();
    setCoincidenceWindow(12);
    
    // Start the internal real-time-clock component
    RTC_1_Start();
    
    // Configure the i2c Real-Time-Clock if that bus extends to the event PSOC (normally not)
    //loadI2Creg(I2C_Address_RTC, 0x00, 0x59);
    //loadI2Creg(I2C_Address_RTC, 0x07, 0x80);     
    
    /* Variable declarations for DMA_1 */
    uint8 DMA_1_Chan;
    uint8 DMA_1_TD[1];  
    uint16 adc1_sampleArray[3] = {0};

    /* Variable declarations for DMA_2 */
    uint8 DMA_2_Chan;
    uint8 DMA_2_TD[1];   
    uint16 adc2_sampleArray[3] = {0};
            
    /* DMA Configuration for DMA_1 SAR ADC */
    DMA_1_Chan = DMA_1_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, 
                 HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_1_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_1_TD[0], DMA_BYTES_PER_BURST*DMA_NO_OF_SAMPLES, DMA_1_TD[0], 
                                 DMA_1__TD_TERMOUT_EN | CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(DMA_1_TD[0], LO16((uint32)ADC_SAR_1_SAR_WRK0_PTR), LO16((uint32)adc1_sampleArray));
    CyDmaChSetInitialTd(DMA_1_Chan, DMA_1_TD[0]);
    CyDmaChEnable(DMA_1_Chan, 1);             /* Enable the DMA channel for the ADC */
    
    /* DMA Configuration for DMA_2 SAR ADC */
    DMA_2_Chan = DMA_2_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, 
                 HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_2_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_2_TD[0], DMA_BYTES_PER_BURST*DMA_NO_OF_SAMPLES, DMA_2_TD[0], 
                                 DMA_1__TD_TERMOUT_EN | CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(DMA_2_TD[0], LO16((uint32)ADC_SAR_2_SAR_WRK0_PTR), LO16((uint32)adc2_sampleArray));
    CyDmaChSetInitialTd(DMA_2_Chan, DMA_2_TD[0]);
    CyDmaChEnable(DMA_2_Chan, 1);             /* Enable the DMA channel for the ADC */
    
    // Variable declarations for DMA_3,4 
    /*
    #define TOF_DMA_BYTES_PER_BURST 4
    #define TOF_DMA_REQUEST_PER_BURST 1
    #define TOF_DMA_NO_OF_SAMPLES 1
    uint8 DMA_3_Chan;
    uint8 DMA_3_TD[1];   
    uint8 DMA_4_Chan;
    uint8 DMA_4_TD[1];   

    // DMA Configuration for TOF shift register A 
    DMA_3_Chan = DMA_3_DmaInitialize(TOF_DMA_BYTES_PER_BURST, TOF_DMA_REQUEST_PER_BURST, 
                 HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_3_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_3_TD[0], TOF_DMA_BYTES_PER_BURST*TOF_DMA_NO_OF_SAMPLES, DMA_3_TD[0], 
                                 DMA_3__TD_TERMOUT_EN | CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(DMA_3_TD[0], LO16((uint32)ShiftReg_A_OUT_FIFO_VAL_LSB_PTR), LO16((uint32)tofA_sampleArray));
    CyDmaChSetInitialTd(DMA_3_Chan, DMA_3_TD[0]);
    CyDmaChEnable(DMA_3_Chan, 1);        
    
    // DMA Configuration for TOF shift register B 
    DMA_4_Chan = DMA_4_DmaInitialize(TOF_DMA_BYTES_PER_BURST, TOF_DMA_REQUEST_PER_BURST, 
                 HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_4_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_4_TD[0], TOF_DMA_BYTES_PER_BURST*TOF_DMA_NO_OF_SAMPLES, DMA_4_TD[0], 
                                 DMA_4__TD_TERMOUT_EN | CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(DMA_4_TD[0], LO16((uint32)ShiftReg_B_OUT_FIFO_VAL_LSB_PTR), LO16((uint32)tofB_sampleArray));
    CyDmaChSetInitialTd(DMA_4_Chan, DMA_4_TD[0]);
    CyDmaChEnable(DMA_4_Chan, 1);        
    */
    // Default configuration of the TOF chip. The second byte should be 0x05 for stop events to be accepted.
    // That is not normally turned on here by default, but rather is turned on when the master trigger is enabled.
    // The reference clock is 12 MHz, which has a period of 83333 picoseconds. With 16 bits it will count up to
    // about 5.46 milliseconds. We reset it every 5 milliseconds, so it should only get up to a count of 60,000
    // which is EA60 in hex. We set the LSB by dividing the reference clock period by 8333, which is ~10ps.
    // Only 14 bit are needed, then, for the stop clocks, but 16 bits are read out. The maximum stop-clock count
    // should be 8333, or hex 208D.
    // Addr0: B5  Make active STOPA, STOPB, REFCLK, LVDS LCLK & LCLKOUT, Ref clk reset
    // Addr1: 05  Activate A and B top inputs; no channel combine; standard resolution
    // Addr2: 0C  Ref and Stop both set to 16 bits; single data rate, no common read, std FIFO
    // Addr3: 8D  Ref Clk divisions = 00208D = 8333 which sets the LSB to be 10 picoseconds with a 12 MHz reference clock
    // Addr4: 20  Ref Clk divisions
    // Addr5: 00  Ref Clk divisions
    // Addr6: 00  Normal LVDS operation; no test pattern
    // Addr7: 08  0ps LVDS adjustment
    // Addr8 through 15 are defaults
    // Addr16 00  Differential LVDS input
    uint8 tofConfig[TOFSIZE] = {0xB5, 0x05, 0x0C, 0x8D, 0x20, 0x00, 0x00, 0x08, 0xA1, 0x13, 0x00,
                         0x0A, 0xCC, 0xCC, 0xF1, 0x7D, 0x00};
    
    // Set up the configuration of the TOF chip AS6501:
    SPIM_ClearTxBuffer();
    SPIM_ClearRxBuffer();

    set_SPI_SSN(SSN_TOF, true);
    SPIM_WriteTxData(powerOnRESET);
    CyDelay(1);

    // Set up the default AS6501 TOF configuration
    set_SPI_SSN(SSN_TOF, true);
    SPIM_WriteTxData(writeConfig);
    for (int i=0; i<TOFSIZE; ++i) {
        SPIM_WriteTxData(tofConfig[i]);
    }
    CyDelay(1);
    
    // Enable the TOF
    set_SPI_SSN(SSN_TOF, true);
    SPIM_WriteTxData(TOF_enable); 
    
    int cmdCountGLB = 0;               // Count of all command packets received
    int cmdCount = 0;                  // Count of all event PSOC commands received
    uint8 command = 0;                 // Most recent command code
    uint8 cmdData[MAX_CMD_DATA];       // Data sent with commands
    uint8 nDataBytes = 0;              // Number of data bytes in the current command
    int dCnt = 0;                      // To count the number of data bytes received
    int nCmdTimeOut = 0;
    const uint8 eventPSOCaddress = '\x08';
    
    // Set up the default trigger configuration
    Cntr8_V1_TKR_WritePeriod(255);    // Tracker trigger prescale
    Cntr8_V1_PMT_WritePeriod(255);    // PMT hadron trigger prescale
    setTriggerMask('e',0x01);
    setTriggerMask('p',0x05);

    // Set interrupt priorities, enable interrupts and configure TOF shift register interrupt signals
    isr_UART_SetPriority(6);
    isr_UART_Enable();     
    isr_timer_SetPriority(7);
    isr_timer_Enable();
    isr_clk200_SetPriority(7);
    isr_clk200_Enable(); 
    isr_Store_A_SetPriority(5);
    isr_Store_A_Enable();   
    ShiftReg_A_EnableInt();  
    ShiftReg_A_SetIntMode(ShiftReg_A_STORE_INT_EN);  // This can hang up indefinitely if the TOF chip is not set up properly
    isr_Store_B_SetPriority(5);
    isr_Store_B_Enable();
    ShiftReg_B_EnableInt();
    ShiftReg_B_SetIntMode(ShiftReg_B_STORE_INT_EN);  // This can hang up indefinitely if the TOF chip is not set up properly 
    isr_Ch1_SetPriority(7);
    isr_Ch1_Enable();
    isr_Ch2_SetPriority(7);
    isr_Ch2_Enable();
    isr_Ch3_SetPriority(7);
    isr_Ch3_Enable();
    isr_Ch4_SetPriority(7);
    isr_Ch4_Enable();
    isr_Ch5_SetPriority(7);
    isr_Ch5_Enable();
    isr_GO1_SetPriority(5);
    isr_GO1_Enable();

    numTkrBrds = 1; //MAX_TKR_BOARDS;
    bool eventDataReady = false;
    bool awaitingCommand = true;   // The system is ready to accept a new command when true
    time_t cmdStartTime = time();
    uint8 rc;
    bool cmdDone = false;    // If true, A command has been fully received but data have not yet been sent back
    set_SPI_SSN(0, true);   // Deselect all SPI slaves
    //uint32 cmdTime = time();
    triggerEnable(false);
    for(;;)
    {
        if (USBUART_IsConfigurationChanged() != 0u) {
            /* Wait for USB-UART Device to enumerate */
            if (USBUART_GetConfiguration() != 0u) {
                /* Enumeration is done, enable OUT endpoint to receive data from Host */
                USBUART_CDC_Init();
            }
        }

//        if ((Status_Reg_2_Read() & 0x01) != 0) {
//            uint8 status = Control_Reg_1_Read() & ~RSTPEAK;
//            Control_Reg_1_Write(status | RSTPEAK);     // Reset the peak detector
//            phSAR = ADC_SAR_1_GetResult16();
//        }

        // Build an event and send it out each time a GO is received
        if (triggered && !cmdDone && awaitingCommand) {    // Delay the data output if a command is in progress
            uint32 timeStampSave = timeStamp;  // Store current count so it cannot change via interrupt
            timeDate = RTC_1_ReadTime();
            triggered = false;
            // Read the digitized PMT data after waiting for the digitizers to finish
            uint t0 = time();
            while (!(Status_Reg_M_Read() & 0x08)) {   // Wait here for the done signal
                if (time() - t0 > 20) {
                    addError(ERR_PMT_DAQ_TIMEOUT, (uint8)cntGO, (uint8)(cntGO >> 8));
                    break;
                }
            }
            // By this point the ADC sample arrays should have been filled by DMA
            // Check that a tracker trigger was received and whether data are ready
            // This check generally works the first try and can maybe be removed in the long run.
            uint8 tkrDataReady = 0;
            uint8 nTry =0;
            if (readTracker) {
                while (tkrDataReady != 0x59) {
                    tkrCmdCode = 0x57;
                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                    UART_TKR_WriteTxData(0x00);    // Address byte
                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                    UART_TKR_WriteTxData(tkrCmdCode);    // Check status
                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                    UART_TKR_WriteTxData(0x00);    // Number of data bytes
                    getTrackerData(TKR_HOUSE_DATA);
                    if (nTkrHouseKeeping > 0) {
                        nTkrHouseKeeping = 0;    // Keep the housekeeping data from being sent out to the world
                        if (tkrHouseKeeping[0] == 0x59) {
                            tkrDataReady = 0x59;
                            break;
                        } else if (tkrHouseKeeping[0] == 0x4E) {
                            tkrDataReady = 0x4E;
                        } else {
                            addError(ERR_TKR_BAD_STATUS, tkrHouseKeeping[0], nTry);
                        }
                    }
                    nTry++;
                    if (nTry > 9) {
                        addError(ERR_TKR_BAD_STATUS, tkrHouseKeeping[0], nTry+1);
                        break;
                    }
                }           
            }
            if (tkrDataReady == 0x59) {
                // Start the read of the Tracker data by sending a read-event command
                tkrLED(true);
                tkrCmdCode = 0x01;
                while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                UART_TKR_WriteTxData(0x00);    // Address byte
                while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                UART_TKR_WriteTxData(tkrCmdCode);    // Read event command
                while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                UART_TKR_WriteTxData(0x01);    // Number of data bytes
                while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                UART_TKR_WriteTxData(0x00);    // Use internally generated trigger tags
                // Read the Tracker event data in from the UART and into internal arrays
                rc = getTrackerData(TKR_EVT_DATA);        
                if (rc != 0) {
                    addError(ERR_GET_TKR_DATA, rc, 0x77);
                    UART_TKR_ClearRxBuffer();
                    resetAllTrackerLogic();
                }
                tkrLED(false);
            } else {   // Make up an empty tracker event if no data were ready
                tkrData.triggerCount = 0;  
                tkrData.cmdCount = 0;
                tkrData.trgPattern = 0;
                tkrData.nTkrBoards = numTkrBrds;
                for (int brd=0; brd<numTkrBrds; ++brd) {  
                    tkrData.boardHits[brd].nBytes = 5;
                    tkrData.boardHits[brd].hitList = (uint8*) malloc(5);
                    if (tkrData.boardHits[brd].hitList != NULL) {
                        tkrData.boardHits[brd].hitList[0] = 0xE7;
                        tkrData.boardHits[brd].hitList[1] = brd;
                        tkrData.boardHits[brd].hitList[2] = 0;
                        tkrData.boardHits[brd].hitList[3] = 0x06;  // A bad CRC will get flagged to indicate the error!
                        tkrData.boardHits[brd].hitList[4] = 0x30;
                    }
                }
            }
            
            // Search for nearly coincident TOF data. Note that each TOF chip channel operates asynchronously w.r.t. the
            // instrument trigger, so we have to correlate the two channels with each other and with the event
            // by looking at the course timing information.
            int InterruptState = CyEnterCriticalSection();
            uint16 timeStamp16 = (uint16)(timeStampSave & 0x0000FFFF);
            int nI=0;
            uint8 idx[TOFMAX_EVT];
            for (int i=0; i<TOFMAX_EVT; ++i) {           // Make a list of TOF hits in channel A
                int iptr = tofA.ptr - i - 1;             // Work backwards in time, starting with the most recent measurement
                if (iptr < 0) iptr = iptr + TOFMAX_EVT;  // Wrap around the circular buffer
                if (!tofA.filled[iptr]) continue;        // Use only entries filled since the previous readout
                if (timeStamp16 == tofA.clkCnt[iptr] || timeStamp16 == tofA.clkCnt[iptr]+1) {
                    idx[nI] = iptr;                      // Only look at entries within two 5ms clock periods of the event time stamp
                    ++nI;
                }
            }
            uint16 aCLK = 65535;
            uint16 bCLK = 65535;
            uint16 aTOF = 65535;
            uint16 bTOF = 65535;
            int16 dtmin = 32767;
            int nJ=0;
            for (int j=0; j<TOFMAX_EVT; ++j) {           // Loop over the TOF hits in channel B
                int jptr = tofB.ptr - j - 1;             // Work backwards in time, starting with the most recent measurement
                if (jptr < 0) jptr = jptr + TOFMAX_EVT;  // Wrap around the circular buffer
                if (!tofB.filled[jptr]) continue;        // Use only entries filled since the previous readout
                // Look only at entries filled within two 5 ms clock periods of the event time stamp
                if (!(tofB.clkCnt[jptr] == timeStamp16 || tofB.clkCnt[jptr] == timeStamp16-1)) continue;
                uint32 BT = tofB.shiftReg[jptr];
                uint16 stopB = (uint16)(BT & 0x0000FFFF);       // Stop time for channel B
                uint16 refB = (uint16)((BT & 0xFFFF0000)>>16);  // Reference clock for channel B
                int timej = refB*8333 + stopB;                  // Full time for channel B in 10 picosecond units
                ++nJ;
                for (int i=0; i<nI; ++i) {                          // Loop over the channel A hits
                    int iptr = idx[i];
                    if (abs(tofA.clkCnt[iptr] - tofB.clkCnt[jptr]) > 1) continue; // Two channels must be within +- 1 clock period
                    uint32 AT = tofA.shiftReg[iptr];
                    uint16 stopA = (uint16)(AT & 0x0000FFFF);       // Stop time for channel A
                    uint16 refA = (uint16)((AT & 0xFFFF0000)>>16);  // Reference clock for channel A
                    int timei = refA*8333 + stopA;                  // Full time for channel A in 10 picosecond units
                    // Here we try to handle cases in which a reference clock rolled over
                    int dt;
                    if (refA > 49152 && refB < 16384) {
                        dt = timej - (timei - 500000000);
                    } else if (refB > 49152 && refA < 16384) {
                        dt = (timej - 500000000) - timei; 
                    } else {
                        dt = timej - timei;
                    }
                    if (abs(dt) < abs(dtmin)) { // Keep the smallest time difference of all combinations
                        dtmin = dt;
                        aCLK = tofA.clkCnt[iptr];  // Save the clock and reference counts for debugging
                        bCLK = tofB.clkCnt[jptr];
                        aTOF = refA;
                        bTOF = refB;
                    }
                }
            }
            CyExitCriticalSection(InterruptState);
            
            // Build the event by filling the output buffer according to the output format.
            // Pack the time and date information into a 4-byte unsigned integer
            uint32 timeWord = ((uint32)timeDate->Year - 2000) << 26;
            timeWord = timeWord | ((uint32)timeDate->Month << 22);
            timeWord = timeWord | ((uint32)timeDate->DayOfMonth << 17);
            timeWord = timeWord | ((uint32)timeDate->Hour << 12);
            timeWord = timeWord | ((uint32)timeDate->Min << 6);
            timeWord = timeWord | ((uint32)timeDate->Sec); 
            //
            // Start the event with a 4-byte header
            dataOut[0] = 0x5A;
            dataOut[1] = 0x45;
            dataOut[2] = 0x52;
            dataOut[3] = 0x4F;
            dataOut[4] = byte16(runNumber, 0);
            dataOut[5] = byte16(runNumber, 1);
            dataOut[6] = byte32(cntGO, 0);     // Event number
            dataOut[7] = byte32(cntGO, 1);
            dataOut[8] = byte32(cntGO, 2);
            dataOut[9] = byte32(cntGO, 3);
            dataOut[10] = byte32(timeStampSave, 0); // Time stamp
            dataOut[11] = byte32(timeStampSave, 1);
            dataOut[12] = byte32(timeStampSave, 2);
            dataOut[13] = byte32(timeStampSave, 3);
            dataOut[14] = byte32(cntGO1, 0);   // Trigger count
            dataOut[15] = byte32(cntGO1, 1);
            dataOut[16] = byte32(cntGO1, 2);
            dataOut[17] = byte32(cntGO1, 3);
            dataOut[18] = byte32(timeWord, 0); // Time and date
            dataOut[19] = byte32(timeWord, 1);
            dataOut[20] = byte32(timeWord, 2);
            dataOut[21] = byte32(timeWord, 3);
            dataOut[22] = trgStatus;
            uint16 T1mV = adc2_sampleArray[2]; //ADC_SAR_2_CountsTo_mVolts(adc2_sampleArray[2]);
            uint16 T2mV = adc1_sampleArray[0]; //ADC_SAR_1_CountsTo_mVolts(adc1_sampleArray[0]);
            uint16 T3mV = adc2_sampleArray[0]; //ADC_SAR_2_CountsTo_mVolts(adc2_sampleArray[0]);
            uint16 T4mV = adc1_sampleArray[1]; //ADC_SAR_1_CountsTo_mVolts(adc1_sampleArray[1]);
            uint16 GmV =  adc2_sampleArray[1]; //ADC_SAR_2_CountsTo_mVolts(adc2_sampleArray[1]);
            dataOut[23] = byte16(T1mV, 0);   // T1
            dataOut[24] = byte16(T1mV, 1);
            dataOut[25] = byte16(T2mV, 0);   // T2
            dataOut[26] = byte16(T2mV, 1);
            dataOut[27] = byte16(T3mV, 0);   // T3
            dataOut[28] = byte16(T3mV, 1);
            dataOut[29] = byte16(T4mV, 0);   // T4
            dataOut[30] = byte16(T4mV, 1);
            dataOut[31] = byte16(GmV, 0);   // G
            dataOut[32] = byte16(GmV, 1);
            dataOut[33] = byte16(adc1_sampleArray[2], 0);   // Extra (for test work)
            dataOut[34] = byte16(adc1_sampleArray[2], 1);
            dataOut[35] = byte16(dtmin, 0);   // TOT
            dataOut[36] = byte16(dtmin, 1);
            dataOut[37] = byte16(tkrData.triggerCount, 0);
            dataOut[38] = byte16(tkrData.triggerCount, 1);
            dataOut[39] = tkrData.cmdCount;
            dataOut[40] = tkrData.trgPattern;
            dataOut[41] = nI;   // Number of TOF readouts since the last trigger
            dataOut[42] = nJ; 
            dataOut[43] = byte16(aTOF,0);    // TOF chip reference clock (for debugging)
            dataOut[44] = byte16(aTOF,1);
            dataOut[45] = byte16(bTOF,0);
            dataOut[46] = byte16(bTOF,1);
            dataOut[47] = byte16(aCLK,0);    // Internal clock at time of TOF event
            dataOut[48] = byte16(aCLK,1);
            dataOut[49] = byte16(bCLK,0);
            dataOut[50] = byte16(bCLK,1);
            dataOut[51] = tkrData.nTkrBoards;
            nDataReady = 52;
            for (int brd=0; brd<tkrData.nTkrBoards; ++brd) {
                if (nDataReady > MAX_DATA_OUT - (5 + tkrData.boardHits[brd].nBytes)) {
                    addError(ERR_EVT_TOO_BIG, dataOut[6], dataOut[10]);
                    break;
                }
                dataOut[nDataReady++] = brd;
                if (tkrData.boardHits[brd].hitList == NULL) {
                    dataOut[nDataReady++] = 5;
                    dataOut[nDataReady++] = 0xE7;
                    dataOut[nDataReady++] = brd;
                    dataOut[nDataReady++] = 0;
                    dataOut[nDataReady++] = 0x07;   // A bad CRC will flag this error
                    dataOut[nDataReady++] = 0x30;
                } else {
                    dataOut[nDataReady++] = tkrData.boardHits[brd].nBytes;
                    for (int b=0; b<tkrData.boardHits[brd].nBytes; ++b) {
                        dataOut[nDataReady++] = tkrData.boardHits[brd].hitList[b];
                    }
                    free(tkrData.boardHits[brd].hitList);
                    tkrData.boardHits[brd].nBytes = 0;
                }
            }
            // Four byte trailer
            dataOut[nDataReady++] = 0x46;
            dataOut[nDataReady++] = 0x49;
            dataOut[nDataReady++] = 0x4E;
            dataOut[nDataReady++] = 0x49;
            eventDataReady = true;
            adc1_sampleArray[0] = 0;
            adc1_sampleArray[1] = 0;
            adc1_sampleArray[2] = 0;
            adc2_sampleArray[0] = 0;
            adc2_sampleArray[1] = 0;
            adc2_sampleArray[2] = 0;
            InterruptState = CyEnterCriticalSection();
            for (int j=0; j<TOFMAX_EVT; ++j) {
                tofA.filled[j] = false;
                tofB.filled[j] = false;
            }
            tofA.ptr = 0;
            tofB.ptr = 0;
            CyExitCriticalSection(InterruptState);
            
            tkrData.nTkrBoards = 0;
            ch1CtrSave = Cntr8_V1_1_ReadCount();
            ch2CtrSave = Cntr8_V1_2_ReadCount();
            ch3CtrSave = Cntr8_V1_3_ReadCount();
            ch4CtrSave = Cntr8_V1_4_ReadCount();
            ch5CtrSave = Cntr8_V1_5_ReadCount();
            ch1CountSave = ch1Count;
            ch2CountSave = ch2Count;
            ch3CountSave = ch3Count;
            ch4CountSave = ch4Count;
            ch5CountSave = ch5Count;
        }
        
        // Data goes out by USBUART, for bench testing, or by SPI to the main PSOC
        // Format: 3 byte aligned packeckets with a 3 byte header (0xDC00FF) 
        //         and 3 byte EOR (0xFF00FF)
        
        if (nDataReady > 0 || cmdDone) {
            if (nDataReady > 0) {    // Send out a command echo only if there are also data to send
                dataLED(true);
                dataPacket[3] = nDataReady + nDataBytes;
                uint16 nPadding = 3 - (nDataBytes + nDataReady)%3;
                if (nPadding == 3) nPadding = 0;
                if (!cmdDone) { // Output is event data, not a command response
                    dataPacket[4] = 0xDD;
                } else {              // Output directly responding to a command
                    dataPacket[4] = command;               
                }
                dataPacket[5] = nDataBytes;
                if (outputMode == USBUART_OUTPUT) {  // Output the header
                    while(USBUART_CDCIsReady() == 0u);
                    USBUART_PutData(dataPacket, 6);  
                } else {
                    set_SPI_SSN(SSN_Main, true);
                    for (int i=0; i<6; ++i) {
                        SPIM_WriteTxData(dataPacket[i]);
                    }
                    set_SPI_SSN(0, false);
                }
                if (nDataBytes > 0) {
                    if (outputMode == USBUART_OUTPUT) {  // Output the command data echo
                        while(USBUART_CDCIsReady() == 0u);
                        USBUART_PutData(cmdData, nDataBytes);
                    } else {
                        set_SPI_SSN(SSN_Main, false);
                        for (int i=0; i<nDataBytes; ++i) {
                            SPIM_WriteTxData(cmdData[i]);
                        }
                        set_SPI_SSN(0, false);  
                    }
                }
                if (outputMode == USBUART_OUTPUT) {    // output the data 
                    uint16 bytesRemaining = nDataReady;
                    const uint16 mxSend = 64;
                    int offset = 0;
                    while (bytesRemaining > 0) {
                        while(USBUART_CDCIsReady() == 0u);
                        if (bytesRemaining > mxSend) {
                            USBUART_PutData(&dataOut[offset], mxSend);
                            offset += mxSend;
                            bytesRemaining -= mxSend;
                        } else {
                            USBUART_PutData(&dataOut[offset], bytesRemaining); 
                            bytesRemaining = 0;
                        }
                    }
                    if (nPadding > 0) {
                        while(USBUART_CDCIsReady() == 0u);
                        USBUART_PutData(Padding, nPadding);
                    }
                    while(USBUART_CDCIsReady() == 0u);
                    USBUART_PutData(&dataPacket[6], 3);  
                } else {                        
                    set_SPI_SSN(SSN_Main, false);
                    for (int i=0; i<nDataReady; ++i) {
                        SPIM_WriteTxData(dataOut[i]);
                    }
                    for (int i=0; i<nPadding; ++i) {
                        SPIM_WriteTxData(Padding[i]);
                    }
                    for (int i=6; i<9; ++i) {
                        SPIM_WriteTxData(dataPacket[i]);
                    }
                    set_SPI_SSN(0, false);
                }
                nDataReady = 0;
                if (eventDataReady) {   // re-enable the trigger after event data has been output
                    LED2_OnOff(false);
                    triggerEnable(true);
                    eventDataReady = false;
                }
                if (cmdDone) {  // The command is completely finished once the echo and data have gone out
                    nDataBytes = 0;
                    awaitingCommand = true;
                    cmdDone = false;
                }
                dataLED(false);
            } else {            // Don't send an echo if the command doesn't result in data. Command is finished.
                awaitingCommand = true;
                cmdDone = false;
            }
        }
        
        // Time-out protection in case the expected data for a command are never sent
        if (!awaitingCommand) {
            if (time() - cmdStartTime > TIMEOUT) {
                awaitingCommand = true;
                cmdDone = false;
                nDataBytes = 0;     // Hopefully this will just flush the bad command
                nCmdTimeOut++;
            }
        }        
        
        // Get a 9-byte command input from the UART or USB-UART
        // The two should not be used at the same time (no reason for that, anyway)
        int count = 0;
        if (nDataReady == 0) { // Don't interpret a new command if data are still going out
            if (USBUART_GetConfiguration() != 0u) {    // Command from USB-UART
                if (USBUART_DataIsReady() != 0u) {
                    count = USBUART_GetAll(buffer);
                }
            }
            if (count == 0 && cmdReadPtr < MX_CMDS) { // Command from UART 
                //int InterruptState = CyEnterCriticalSection();
                isr_UART_Disable();
                count = CMD_LENGTH;
                for (int i=0; i<count; ++i) {
                    buffer[i] = cmd_buffer[cmdReadPtr].buf[i];
                }
                cmd_buffer[cmdReadPtr].nBytes = 0;
                if (ptrNext(cmdReadPtr) != cmdWritePtr) {
                    cmdReadPtr = ptrNext(cmdReadPtr);
                } else {
                    cmdReadPtr = 255;
                }
                isr_UART_Enable();
                //CyExitCriticalSection(InterruptState);
            }
        }
        if (count == CMD_LENGTH) {
            bool badCMD = false;
            for (int i=0; i<9; ++i) {   // Check that all 3 command copies are identical
                if (buffer[i] != buffer[i+9] || buffer[i] != buffer[i+18]) {
                    badCMD = true;
                    break;
                }
            }  
            if (badCMD) {
                badCMD = false;
                for (int i=0; i<9; ++i) {   // Check that first two command copies are identical
                    if (buffer[i] != buffer[i+9]) {
                        badCMD = true;
                        break;
                    }
                }
                if (badCMD) {
                    badCMD = false;
                    for (int i=0; i<9; ++i) {   // Check that first and third command copies are identical
                        if (buffer[i] != buffer[i+18]) {
                            badCMD = true;
                            break;
                        }
                    }
                    if (badCMD) {
                        badCMD = false;
                        for (int i=0; i<9; ++i) {   // Check that last two command copies are identical
                            if (buffer[i+9] != buffer[i+18]) {
                                badCMD = true;
                                addError(ERR_BAD_CMD, code[buffer[i+9]], i);
                                break;
                            }
                        }
                        if (!badCMD) for (int i=0; i<9; ++i) buffer[i] = buffer[i+9];
                    }
                }
            }
            if (!badCMD && buffer[0] == 'S' && buffer[8] == 'W') {
                if (awaitingCommand) cmdCountGLB++;
                //cmdTime = time();
                uint8 nib3 = code[buffer[3]];
                uint8 nib4 = code[buffer[4]];
                uint8 addressByte = (nib3<<4) | nib4;
                uint8 PSOCaddress = (addressByte & '\x3C')>>2;
                if (PSOCaddress == eventPSOCaddress) {                    
                    uint8 nib1 = code[buffer[1]];  // No check on code. Illegal characters get translated to 0.
                    uint8 nib2 = code[buffer[2]];
                    uint8 dataByte = (nib1<<4) | nib2;
                    if (awaitingCommand) {         // This is the start of a new command
                        awaitingCommand = false;
                        cmdStartTime = time();
                        cmdCount++;
                        dCnt = 0;
                        nDataBytes = ((addressByte & '\xC0') >> 4) | (addressByte & '\x03');
                        command = dataByte;
                        if (nDataBytes == 0) cmdDone = true;
                    } else {                       // Receiving data from a command in progress
                        uint8 byteCnt = ((addressByte & '\xC0') >> 4) | (addressByte & '\x03');
                        if (byteCnt != 0) {
                            cmdData[byteCnt-1] = dataByte;
                            dCnt++;
                            if (dCnt == nDataBytes) cmdDone = true; 
                        } else {
                            addError(ERR_BAD_BYTE, command, nDataBytes);
                            badCMD = true;
                        }
                    }
                }
                if (cmdDone && badCMD) {
                    cmdDone = false;
                    awaitingCommand = true;  // Abort a bad command
                    nDataBytes = 0;
                }
                if (cmdDone && !badCMD) {
                    uint16 DACsetting12;
                    uint32 tStart;
                    uint8 DACaddress = 0;
                    uint16 thrSetting;
                    uint8 nCalClusters;
                    uint8 fpgaAddress;
                    uint8 chipAddress;
                    // If the trigger is enabled, ignore all commands besides disable trigger, 
                    // so that nothing can interrupt the readout.
                    if (command == '\x3D' || command == '\x44' || !isTriggerEnabled()) {
                        switch (command) { 
                            case '\x01':         // Load a threshold DAC setting
                                switch (cmdData[0]) {
                                    case 0x05: 
                                        thrSetting = (uint16)cmdData[1];
                                        thrSetting = (thrSetting<<8) | (uint16)cmdData[2];
                                        rc = loadDAC(I2C_Address_DAC_Ch5, thrSetting);
                                        if (rc != 0) {
                                            addError(ERR_DAC_LOAD, rc, I2C_Address_DAC_Ch5);
                                        }
                                        break;
                                    case 0x01:
                                        VDAC8_Ch1_SetValue(cmdData[1]);
                                        thrDACsettings[0] = cmdData[1];
                                        break;
                                    case 0x02:
                                        VDAC8_Ch2_SetValue(cmdData[1]);
                                        thrDACsettings[1] = cmdData[1];
                                        break;
                                    case 0x03:
                                        VDAC8_Ch3_SetValue(cmdData[1]);
                                        thrDACsettings[2] = cmdData[1];
                                        break;
                                    case 0x04:
                                        VDAC8_Ch4_SetValue(cmdData[1]);
                                        thrDACsettings[3] = cmdData[1];
                                        break;
                                }
                                break;
                            case '\x02':         // Get a threshold DAC setting
                                if (cmdData[0] == 0x05) {
                                    nDataReady = 2;
                                    rc = readDAC(I2C_Address_DAC_Ch5, &DACsetting12);
                                    if (rc != 0) {
                                        DACsetting12 = 0;
                                        addError(ERR_DAC_READ, rc, DACaddress);
                                    }
                                    dataOut[0] = (uint8)((DACsetting12 & 0xFF00)>>8);
                                    dataOut[1] = (uint8)(DACsetting12 & 0x00FF);
                                } else if (cmdData[0] < 5) {
                                    nDataReady = 1;
                                    dataOut[0] = thrDACsettings[cmdData[0]-1];
                                } else {
                                    nDataReady = 1;
                                    dataOut[0] = 0;
                                }
                                break;
                            case '\x03':         // Read back all of the accumulated error codes
                                if (nErrors == 0) {
                                    nDataReady = 3;
                                    dataOut[0] = 0x00;
                                    dataOut[1] = 0xEE;
                                    dataOut[2] = 0xFF;
                                    break;
                                }
                                nDataReady = nErrors*3;
                                for (int i=0; i<nErrors; ++i) {
                                    dataOut[i*3] = errors[i].errorCode;
                                    dataOut[i*3 + 1] = errors[i].value0;
                                    dataOut[i*3 + 2] = errors[i].value1;                                    
                                }
                                nErrors = 0;
                                break;
                            case '\x04':        // Load the TOF DACs
                                if (cmdData[0] == 1) {
                                    DACaddress = I2C_Address_TOF_DAC1;                                    
                                } else if (cmdData[0] == 2) {                                    
                                    DACaddress = I2C_Address_TOF_DAC2;
                                } else break;
                                uint16 thrSetting = (uint16)cmdData[1];                                
                                thrSetting = (thrSetting<<8) | (uint16)cmdData[2];
                                rc = loadDAC(DACaddress, thrSetting);
                                if (rc != 0) {
                                    addError(ERR_TOF_DAC_LOAD, rc, DACaddress);
                                }
                                break;
                            case '\x05':        // Read the TOF DAC settings                               
                                if (cmdData[0] == 1) {
                                    DACaddress = I2C_Address_TOF_DAC1;
                                } else if (cmdData[0] == 2) {
                                    DACaddress = I2C_Address_TOF_DAC2;
                                } else break;
                                rc = readDAC(DACaddress, &DACsetting12);
                                if (rc != 0) {
                                    DACsetting12 = 0;
                                    addError(ERR_TOF_DAC_READ, rc, DACaddress);                                         
                                }
                                
                                nDataReady = 2;
                                dataOut[0] = (uint8)((DACsetting12 & 0xFF00)>>8);
                                dataOut[1] = (uint8)(DACsetting12 & 0x00FF);
                                break;
                            case '\x06':        // Turn LED on or off, for communication test
                                if (cmdData[0] == 1) {
                                    LED2_OnOff(true);
                                } else {
                                    LED2_OnOff(false);
                                }
                                break;
                            case '\x07':        // Return the version number
                                nDataReady = 1;
                                dataOut[0] = VERSION;
                                break;
                            case '\x10':        // Send an arbitrary command to the tracker
                                tkrCmdCode = cmdData[1];
                                // Ignore commands that are supposed to be internal to the tracker,
                                // to avoid confusing the tracker logic.
                                if (tkrCmdCode == 0x52 || tkrCmdCode == 0x53) break;
                                tkrLED(true);
                                UART_TKR_PutChar(cmdData[0]);   // FPGA address
                                UART_TKR_PutChar(tkrCmdCode);
                                uint8 nDataTKR = cmdData[2];
                                UART_TKR_PutChar(nDataTKR);
                                for (int i=0; i<nDataTKR; ++i) {
                                    UART_TKR_PutChar(cmdData[3+i]);
                                }
                                
                                if (tkrCmdCode == 0x0F) {  // This command sets the number of tracker boards in the readout.
                                    numTkrBrds = cmdData[3];  // Make sure the PSOC also knows how many boards are used.
                                }
                                // Wait around for up to a second for all the data to transmit
                                tStart = time();
                                while (UART_TKR_GetTxBufferSize() > 0) {                                           
                                    if (time() - tStart > 200) {
                                        addError(ERR_TX_FAILED, tkrCmdCode, command);
                                        tkrLED(false);
                                        break;
                                    }
                                }                                
                                if (tkrCmdCode == 0x67 || tkrCmdCode == 0x6C) {
                                    tkrLED(false);
                                    break; // These commands have no echo
                                }
                                // Now look for the bytes coming back from the Tracker.
                                if (tkrCmdCode >= 0x20 && tkrCmdCode <= 0x25) {
                                    getASICdata();
                                } else if (tkrCmdCode == 0x46) {
                                    getTKRi2cData();
                                } else {
                                    rc = getTrackerData(0);
                                    if (rc != 0) {
                                        addError(ERR_GET_TKR_DATA, rc, command);
                                    }
                                }
                                tkrLED(false);
                                break;
                            case '\x41':        // Load a tracker ASIC mask register
                                tkrLED(true);
                                fpgaAddress = cmdData[0] & 0x07;
                                chipAddress = cmdData[1] & 0x1F;
                                uint8 regType = cmdData[2] & 0x03;
                                uint8 fill = cmdData[3] & 0x01;
                                nCalClusters = cmdData[4];
                                if (nCalClusters > (nDataBytes - 5)/2) {
                                    nCalClusters = (nDataBytes - 5)/2;
                                }   
                                int ptr = 5;
                                uint64 mask = 0;
                                for (int i=0; i<nCalClusters; ++i) {
                                    uint64 mask0 = 0;
                                    int nch = cmdData[ptr];
                                    int ch0 = 64-nch-cmdData[ptr+1];
                                    mask0 = mask0 +1;
                                    for (int j=1; j<nch; ++j) {
                                        mask0 = mask0<<1;
                                        mask0 = mask0 + 1;
                                    }
                                    mask0 = mask0<<ch0;
                                    mask = mask | mask0;
                                    ptr = ptr + 2;
                                }
                                if (fill) mask = ~mask;
                                if (regType == CALMASK) tkrCmdCode = '\x15';
                                else if (regType == DATAMASK) tkrCmdCode = '\x13';
                                else tkrCmdCode = '\x14';
                                UART_TKR_PutChar(fpgaAddress);
                                UART_TKR_PutChar(tkrCmdCode);  
                                UART_TKR_PutChar('\x09');
                                UART_TKR_PutChar(chipAddress);
                                uint8 bytesToSend[8];
                                for (int j=0; j<8; ++j) {
                                    bytesToSend[j] = (uint8)(mask & 0x00000000000000FF);
                                    mask = mask>>8;
                                }
                                for (int j=7; j>=0; --j) {
                                    UART_TKR_PutChar(bytesToSend[j]);
                                }
                                
                                // Wait around for up to a second for all the data to transmit
                                tStart = time();
                                while (UART_TKR_GetTxBufferSize() > 0) {                                           
                                    if (time() - tStart > 200) {
                                        addError(ERR_TX_FAILED, tkrCmdCode, command);
                                        tkrLED(false);
                                        break;
                                    }
                                }    
                                rc = getTrackerData(TKR_ECHO_DATA);  // Get the command echo
                                if (rc != 0) {
                                    addError(ERR_GET_TKR_DATA, rc, tkrCmdCode);
                                }
                                tkrLED(false);
                                break;
                            case '\x42':        // Start a tracker calibration sequence
                                // First send a calibration strobe command
                                tkrLED(true);
                                tkrCmdCode = '\x02';
                                UART_TKR_PutChar('\x00');
                                UART_TKR_PutChar(tkrCmdCode);
                                UART_TKR_PutChar('\x03');
                                UART_TKR_PutChar('\x1F');
                                uint8 FPGA = cmdData[0];
                                uint8 trgDelay = cmdData[1];
                                uint8 trgTag = cmdData[2] & 0x03;
                                uint8 byte2 = (trgDelay & 0x3f)<<2;
                                byte2 = byte2 | trgTag;
                                UART_TKR_PutChar(byte2);
                                UART_TKR_PutChar(FPGA);
                                // Wait around for up to a second for all the data to transmit
                                tStart = time();
                                while (UART_TKR_GetTxBufferSize() > 0) {                                           
                                    if (time() - tStart > 200) {
                                        addError(ERR_TX_FAILED, tkrCmdCode, command);
                                        tkrLED(false);
                                        break;
                                    }
                                }    
                                // Catch the trigger output and send back to the computer                              
                                getTrackerBoardTriggerData(FPGA);
                                tkrLED(false);
                                break;
                            case '\x43':   // Send a tracker read-event command for calibration events
                                tkrLED(true);
                                tkrCmdCode = '\x01';
                                trgTag = cmdData[0] & 0x03;
                                UART_TKR_PutChar('\x00');
                                UART_TKR_PutChar(tkrCmdCode);
                                UART_TKR_PutChar('\x01');
                                UART_TKR_PutChar(0x04 | trgTag);
                                // Wait around for up to a second for all the data to transmit
                                tStart = time();
                                while (UART_TKR_GetTxBufferSize() > 0) {                                           
                                    if (time() - tStart > 200) {
                                        addError(ERR_TX_FAILED, tkrCmdCode, command);
                                        tkrLED(false);
                                        break;
                                    }
                                }    
                                //CyDelay(1);
                                // Read the data from the tracker
                                rc = getTrackerData(TKR_EVT_DATA);
                                if (rc != 0) {
                                    addError(ERR_GET_TKR_DATA, rc, command);
                                    UART_TKR_ClearRxBuffer();
                                    resetAllTrackerLogic();
                                }
                                
                                // Then send the data out as a tracker-only event
                                dataOut[0] = 0x5A;
                                dataOut[1] = 0x45;
                                dataOut[2] = 0x52;
                                dataOut[3] = 0x4F;
                                dataOut[4] = tkrData.nTkrBoards;
                                nDataReady = 5;
                                for (int brd=0; brd<tkrData.nTkrBoards; ++brd) {
                                    if (nDataReady > MAX_DATA_OUT - (5 + tkrData.boardHits[brd].nBytes)) {
                                        addError(ERR_EVT_TOO_BIG, dataOut[6], dataOut[10]);
                                        break;
                                    }
                                    dataOut[nDataReady++] = brd;
                                    dataOut[nDataReady++] = tkrData.boardHits[brd].nBytes;
                                    for (int b=0; b<tkrData.boardHits[brd].nBytes; ++b) {
                                        dataOut[nDataReady++] = tkrData.boardHits[brd].hitList[b];
                                    }
                                    free(tkrData.boardHits[brd].hitList);
                                    tkrData.boardHits[brd].nBytes = 0;
                                }
                                dataOut[nDataReady++] = 0x46;
                                dataOut[nDataReady++] = 0x49;
                                dataOut[nDataReady++] = 0x4E;
                                dataOut[nDataReady++] = 0x49;
                                tkrLED(false);
                                break;
                            case '\x0C':        // Reset the TOF chip
                                set_SPI_SSN(SSN_TOF, true);
                                SPIM_WriteTxData(powerOnRESET);
                                set_SPI_SSN(0, false);
                                break;
                            case '\x0D':        // Modify TOF configuration (disable trigger first)
                                if (cmdData[0] < TOFSIZE) {
                                    tofConfig[cmdData[0]] = tofConfig[1];
                                    set_SPI_SSN(SSN_TOF, true);
                                    SPIM_WriteTxData(writeConfig);
                                    for (int i=0; i<TOFSIZE; ++i) {
                                        SPIM_WriteTxData(tofConfig[i]);
                                    }
                                    CyDelay(1);
                                    set_SPI_SSN(0, false);
                                }
                                break;
                            case '\x0E':        // Read the TOF IC configuration
                                SPIM_ClearRxBuffer();
                                set_SPI_SSN(SSN_TOF, true);
                                CyDelay(1);
                                SPIM_WriteTxData(readConfig);                                
                                while (SPIM_GetRxBufferSize() == 0) SPIM_WriteTxData(0x00);
                                SPIM_ReadRxData();    // The first byte read back is always garbage.
                                for (int bt=0; bt<TOFSIZE; ++bt) {
                                    while (SPIM_GetRxBufferSize() == 0) SPIM_WriteTxData(0x00);
                                    dataOut[bt] = SPIM_ReadRxData();
                                    //dataOut[bt] = tofConfig[bt];
                                }
                                nDataReady = TOFSIZE;
                                set_SPI_SSN(0, false);
                                break;
                            case '\x20':        // Read bus voltages (positive only)
                                readI2Creg(2, cmdData[0], INA226_BusV_Reg, dataOut);
                                nDataReady = 2;
                                break;
                            case '\x21':        // Read currents (Note: bit 15 is a sign bit, 2's complement)
                                readI2Creg(2, cmdData[0], INA226_ShuntV_Reg, dataOut);
                                nDataReady = 2;
                                break;
                            case '\x22':        // Read the board temperature
                                readI2Creg(2, I2C_Address_TMP100, TMP100_Temp_Reg, dataOut);
                                nDataReady = 2;
                                break;
                            case '\x23':        // Read an RTC register
                                readI2Creg(1, I2C_Address_RTC, cmdData[0], dataOut);
                                nDataReady = 1;
                                break;
                            case '\x24':        // Write an RTC register
                                loadI2Creg(I2C_Address_RTC , cmdData[0], cmdData[1]);
                                break;
                            case '\x26':       // Read a barometer register
                                readI2Creg(1, I2C_Address_Barometer, cmdData[0], dataOut);
                                nDataReady = 1;
                                break;
                            case '\x27':       // Load a barometer register
                                loadI2Creg(I2C_Address_Barometer, cmdData[0], cmdData[1]);
                                break;
                            case '\x30':       // Set the output mode
                                if (cmdData[0] == USBUART_OUTPUT || cmdData[0] == SPI_OUTPUT) {
                                    outputMode = cmdData[0];
                                }
                                break;
                            case '\x31':       // Initialize the SPI interface
                                SPIM_Init();
                                SPIM_Enable();
                                break;
                            case '\x32':       // Send TOF info to USB-UART (temporary testing)
                                outputTOF = true;                               
                                break;
                            case '\x3F':
                                outputTOF = false;
                                break;
                            case '\x34':       // Get the number of TOF events stored
                                nDataReady = 2;
                                dataOut[0] = tofA.ptr;
                                dataOut[1] = tofB.ptr;
                                break;
                            case '\x35':       // Read most recent TOF event from channel A or B (for testing)
                                nDataReady = 9;
                                int InterruptState = CyEnterCriticalSection();
                                if (cmdData[0] == 0) {                                    
                                    uint8 idx = tofA.ptr - 1;
                                    if (idx < 0) idx = idx + TOFMAX_EVT;
                                    if (tofA.filled[idx]) {
                                        uint32 AT = tofA.shiftReg[idx];
                                        uint16 stopA = (uint16)(AT & 0x0000FFFF);
                                        uint16 refA = (uint16)((AT & 0xFFFF0000)>>16);
                                        dataOut[0] = (uint8)((refA & 0xFF00)>>8);
                                        dataOut[1] = (uint8)(refA & 0x00FF);
                                        dataOut[2] = 0;
                                        dataOut[3] = (uint8)((stopA & 0xFF00)>>8);
                                        dataOut[4] = (uint8)(stopA & 0x00FF);
                                        dataOut[5] = 0;
                                        dataOut[6] = (uint8)((tofA.clkCnt[idx] & 0xFF00)>>8);
                                        dataOut[7] = (uint8)(tofA.clkCnt[idx] & 0x00FF);
                                        dataOut[8] = tofA.ptr;
                                        for (int j=0; j<TOFMAX_EVT; ++j) {
                                            tofA.filled[j] = false;
                                        }
                                        tofA.ptr = 0;
                                    } else {
                                        for (int i=0; i<8; ++i) dataOut[i] = 0;
                                        dataOut[8] = idx;
                                    }
                                } else {
                                    uint8 idx = tofB.ptr - 1;
                                    if (idx < 0) idx = idx + TOFMAX_EVT;
                                    if (tofB.filled[idx]) {
                                        uint32 BT = tofB.shiftReg[idx];
                                        uint16 stopB = (uint16)(BT & 0x0000FFFF);
                                        uint16 refB = (uint16)((BT & 0xFFFF0000)>>16);
                                        dataOut[0] = (uint8)((refB & 0xFF00)>>8);
                                        dataOut[1] = (uint8)(refB & 0x00FF);
                                        dataOut[2] = 0;
                                        dataOut[3] = (uint8)((stopB & 0xFF00)>>8);
                                        dataOut[4] = (uint8)(stopB & 0x00FF);
                                        dataOut[5] = 0;
                                        dataOut[6] = (uint8)((tofB.clkCnt[idx] & 0xFF00)>>8);
                                        dataOut[7] = (uint8)(tofB.clkCnt[idx] & 0x00FF);
                                        dataOut[8] = tofB.ptr;
                                        for (int j=0; j<TOFMAX_EVT; ++j) {
                                            tofB.filled[j] = false;
                                        }
                                        tofB.ptr = 0;
                                    } else {
                                        for (int i=0; i<8; ++i) dataOut[i] = 0;
                                        dataOut[8] = idx;
                                        for (int j=0; j<TOFMAX_EVT; ++j) {
                                            tofB.filled[j] = false;
                                            tofA.filled[j] = false;
                                        }
                                        tofB.ptr = 0;
                                        tofA.ptr = 0;
                                    }
                                }
                                CyExitCriticalSection(InterruptState);
                                break; 
                            case '\x36':     // Set a trigger mask
                                if (cmdData[0] == 1) {
                                    setTriggerMask('e', cmdData[1]);
                                } else if (cmdData[0] == 2) {
                                    setTriggerMask('p', cmdData[1]);
                                }
                                break;
                            case '\x37':    // Read a channel counter
                                nDataReady = 3;
                                switch (cmdData[0]) {
                                    case 0x01:
                                        dataOut[2] = Cntr8_V1_1_ReadCount();
                                        dataOut[1] = (uint8)(ch1Count & 0x00FF);
                                        dataOut[0] = (uint8)((ch1Count & 0xFF00)>>8);
                                        break;
                                    case 0x02:
                                        dataOut[2] = Cntr8_V1_2_ReadCount();
                                        dataOut[1] = (uint8)(ch2Count & 0x00FF);
                                        dataOut[0] = (uint8)((ch2Count & 0xFF00)>>8);
                                        break;
                                    case 0x03:
                                        dataOut[2] = Cntr8_V1_3_ReadCount();
                                        dataOut[1] = (uint8)(ch3Count & 0x00FF);
                                        dataOut[0] = (uint8)((ch3Count & 0xFF00)>>8);
                                        break;
                                    case 0x04:
                                        dataOut[2] = Cntr8_V1_4_ReadCount();
                                        dataOut[1] = (uint8)(ch4Count & 0x00FF);
                                        dataOut[0] = (uint8)((ch4Count & 0xFF00)>>8);
                                        break;
                                    case 0x05:
                                        dataOut[2] = Cntr8_V1_5_ReadCount();
                                        dataOut[1] = (uint8)(ch5Count & 0x00FF);
                                        dataOut[0] = (uint8)((ch5Count & 0xFF00)>>8);
                                        break;
                                }
                                break;
                            case '\x38':   // Reset the logic and counters, after reading back 24 bits of the clock count
                                nDataReady = 3;
                                uint32 now = time();
                                dataOut[0] = (uint8)((now & 0x00FF0000)>>16);
                                dataOut[1] = (uint8)((now & 0x0000FF00)>>8);
                                dataOut[2] = (uint8)(now & 0x000000FF);
                                logicReset();
                                break;
                            case '\x39':   // Set trigger prescales
                                if (cmdData[0] == 1) {
                                    Cntr8_V1_TKR_WritePeriod(cmdData[1]);
                                } else if (cmdData[0] == 2) {
                                    Cntr8_V1_PMT_WritePeriod(cmdData[1]);
                                }
                                break;
                            case '\x3A':   // Set trigger coincidence window
                                setCoincidenceWindow(cmdData[0]);
                                break;
                            case '\x3B':   // Enable or disable the trigger
                                if (cmdData[0] == 1) {
                                    triggerEnable(true);
                                } else if (cmdData[0] == 0) {
                                    triggerEnable(false);
                                }
                                break;
                            case '\x44':  // End a run and send out the run summary
                                triggered = false;   // this might throw out the last event
                                triggerEnable(false);
                                dataOut[0] = byte32(cntGO1, 0);
                                dataOut[1] = byte32(cntGO1, 1);
                                dataOut[2] = byte32(cntGO1, 2);
                                dataOut[3] = byte32(cntGO1, 3);
                                dataOut[4] = byte32(cntGO, 0);
                                dataOut[5] = byte32(cntGO, 1);
                                dataOut[6] = byte32(cntGO, 2);
                                dataOut[7] = byte32(cntGO, 3);
                                nDataReady = 8;
                                break;
                            case '\x3C':  // Start a run
                                InterruptState = CyEnterCriticalSection();
                                for (int j=0; j<TOFMAX_EVT; ++j) {
                                    tofA.filled[j] = false;
                                    tofB.filled[j] = false;
                                }
                                clkCnt = 0;
                                tofA.ptr = 0;
                                tofB.ptr = 0;
                                CyExitCriticalSection(InterruptState);
                                ch1Count = 0;
                                ch2Count = 0;
                                ch3Count = 0;
                                ch4Count = 0;
                                ch5Count = 0;
                                runNumber = cmdData[0];
                                runNumber = (runNumber<<8) | cmdData[1];
                                readTracker = (cmdData[2] == 1);
                                // Make sure that the TOT FIFOs are empty
                                while (ShiftReg_A_GetFIFOStatus(ShiftReg_A_OUT_FIFO) != ShiftReg_A_RET_FIFO_EMPTY) {
                                    ShiftReg_A_ReadData();
                                }
                                while (ShiftReg_B_GetFIFOStatus(ShiftReg_B_OUT_FIFO) != ShiftReg_B_RET_FIFO_EMPTY) {
                                    ShiftReg_B_ReadData();
                                }
                                
                                cntGO = 0;
                                cntGO1 = 0;
                                triggerEnable(true);
                                Control_Reg_Pls_Write(PULSE_CNTR_RST);
                                // Enable the tracker trigger
                                if (readTracker) {
                                    tkrCmdCode = 0x65;
                                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                                    UART_TKR_WriteTxData(0x00);    // Address byte
                                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                                    UART_TKR_WriteTxData(tkrCmdCode);    // Trigger enable
                                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                                    UART_TKR_WriteTxData(0x00);    // Number of data bytes
                                    uint8 rc = getTrackerData(TKR_ECHO_DATA);   // Get the echo. Note that any delay put before this results in
                                                                                // the first few bytes of the echo getting missed. Don't know why.
                                    if (rc != 0) {
                                        addError(ERR_TKR_TRG_ENABLE, dataOut[2], rc);;
                                    }
                                }
                                nDataReady = 0;  // Don't send the echo back to the UART
                                break;
                            case '\x3D':  // Return trigger enable status
                                nDataReady =1;
                                if (isTriggerEnabled()) dataOut[0] = 1;
                                else dataOut[0] = 0;
                                break;
                            case '\x3E':  // Return trigger mask register
                                nDataReady = 1;
                                uint8 reg = 0;
                                if (cmdData[0] == 1) {
                                    reg = getTriggerMask('e');
                                } else if (cmdData[0] == 2) {
                                    reg = getTriggerMask('p');
                                }
                                dataOut[0] = reg;
                                break;
                            case '\x33':    // Read a saved channel counter, from end of run
                                nDataReady = 3;
                                switch (cmdData[0]) {
                                    case 0x01:
                                        dataOut[2] = ch1CtrSave;
                                        dataOut[1] = (uint8)(ch1CountSave & 0x00FF);
                                        dataOut[0] = (uint8)((ch1CountSave & 0xFF00)>>8);
                                        break;
                                    case 0x02:
                                        dataOut[2] = ch2CtrSave;
                                        dataOut[1] = (uint8)(ch2CountSave & 0x00FF);
                                        dataOut[0] = (uint8)((ch2CountSave & 0xFF00)>>8);
                                        break;
                                    case 0x03:
                                        dataOut[2] = ch3CtrSave;
                                        dataOut[1] = (uint8)(ch3CountSave & 0x00FF);
                                        dataOut[0] = (uint8)((ch3CountSave & 0xFF00)>>8);
                                        break;
                                    case 0x04:
                                        dataOut[2] = ch4CtrSave;
                                        dataOut[1] = (uint8)(ch4CountSave & 0x00FF);
                                        dataOut[0] = (uint8)((ch4CountSave & 0xFF00)>>8);
                                        break;
                                    case 0x05:
                                        dataOut[2] = ch5CtrSave;
                                        dataOut[1] = (uint8)(ch5CountSave & 0x00FF);
                                        dataOut[0] = (uint8)((ch5CountSave & 0xFF00)>>8);
                                        break;
                                }
                                break;
                            case '\x40':  // Read all TOF data (for testing)
                                nDataReady = 3;
                                uint8 nA = 0;
                                uint8 nB = 0;
                                InterruptState = CyEnterCriticalSection();
                                for (int i=0; i<TOFMAX_EVT; ++i) {
                                    if (tofA.filled[i]) ++nA;
                                    if (tofB.filled[i]) ++nB;
                                }
                                dataOut[2] = 1;
                                if (nA > 21 || nB > 21) {
                                    dataOut[2] = 2;
                                    if (nA > 21) nA = 21;
                                    if (nB > 21) nB = 21;
                                }
                                dataOut[0] = nA;
                                dataOut[1] = nB;
                                int iptr = tofA.ptr;
                                int jptr = tofB.ptr;
                                uint8 cnt = 0;
                                for (int i=0; i<TOFMAX_EVT; ++i) {
                                    if (!tofA.filled[i]) continue;
                                    --iptr;
                                    if (iptr < 0) iptr += TOFMAX_EVT;
                                    uint32 AT = tofA.shiftReg[iptr];
                                    uint16 stopA = (uint16)(AT & 0x0000FFFF);
                                    uint16 refA = (uint16)((AT & 0xFFFF0000)>>16);
                                    dataOut[nDataReady] = byte16(refA,0);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(refA,1);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(stopA,0);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(stopA,1);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(tofA.clkCnt[iptr],0);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(tofA.clkCnt[iptr],1);
                                    ++nDataReady;
                                    ++cnt;
                                    if (cnt >= nA) break;
                                }
                                cnt = 0;
                                for (int i=0; i<TOFMAX_EVT; ++i) {
                                    if (!tofB.filled[i]) continue;
                                    --jptr;
                                    if (jptr < 0) jptr += TOFMAX_EVT;
                                    uint32 BT = tofB.shiftReg[jptr];
                                    uint16 stopB = (uint16)(BT & 0x0000FFFF);
                                    uint16 refB = (uint16)((BT & 0xFFFF0000)>>16);
                                    dataOut[nDataReady] = byte16(refB,0);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(refB,1);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(stopB,0);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(stopB,1);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(tofB.clkCnt[jptr],0);
                                    ++nDataReady;
                                    dataOut[nDataReady] = byte16(tofB.clkCnt[jptr],1);
                                    ++nDataReady;
                                    ++cnt;
                                    if (cnt >= nB) break;
                                }
                                for (int j=0; j<TOFMAX_EVT; ++j) {
                                    tofB.filled[j] = false;
                                    tofA.filled[j] = false;
                                }
                                tofB.ptr = 0;
                                tofA.ptr = 0;
                                CyExitCriticalSection(InterruptState);
                                break;
                            case '\x45': // Set the time and date of the real-time-clock
                                timeDate->Sec = cmdData[0];
                                timeDate->Min = cmdData[1];
                                timeDate->Hour = cmdData[2];
                                timeDate->DayOfWeek = cmdData[3];
                                timeDate->DayOfMonth = cmdData[4];
                                timeDate->DayOfYear = cmdData[6] + cmdData[5]*256;
                                timeDate->Month = cmdData[7];
                                timeDate->Year = cmdData[9] + cmdData[8]*256;
                                RTC_1_WriteTime(timeDate);
                                break;
                            case '\x46': // get the time and date of the real-time-clock
                                nDataReady = 10;
                                timeDate = RTC_1_ReadTime();
                                dataOut[0] = timeDate->Sec;
                                dataOut[1] = timeDate->Min;
                                dataOut[2] = timeDate->Hour;
                                dataOut[3] = timeDate->DayOfWeek;
                                dataOut[4] = timeDate->DayOfMonth;
                                dataOut[5] = timeDate->DayOfYear/256;
                                dataOut[6] = timeDate->DayOfYear%256;
                                dataOut[7] = timeDate->Month;
                                dataOut[8] = timeDate->Year/256;
                                dataOut[9] = timeDate->Year%256;
                                break;
                            case '\x47': // Reset the tracker state machines
                                resetAllTrackerLogic();
                                break;
                            case '\x48': // Calibrate the input timing on one or every Tracker FPGA board
                                if (cmdData[0] > 7) {
                                    calibrateAllInputTiming();
                                } else {
                                    calibrateInputTiming(cmdData[0]);
                                }
                                break;
                        } // End of command switch
                    } else { // Log an error if the user is sending spurious commands while the trigger is enabled
                        addError(ERR_CMD_IGNORE, command, 0);
                    }
                }
            } // End of command polling            
        }
        
        // Send out Tracker housekeeping data immediately after receiving it from the Tracker
        if (!isTriggerEnabled() && nTkrHouseKeeping>0) {
            nDataReady = nTkrHouseKeeping + 7;
            dataOut[0] = nDataReady;
            dataOut[1] = 0xC7;
            dataOut[2] = nTkrHouseKeeping;
            dataOut[3] = byte16(tkrCmdCount,0);
            dataOut[4] = byte16(tkrCmdCount,1);
            dataOut[5] = tkrHouseKeepingFPGA;
            dataOut[6] = tkrCmdCode;
            for (int i=0; i<nTkrHouseKeeping; ++i) {
                dataOut[6+i] = tkrHouseKeeping[i];
            }
            nTkrHouseKeeping = 0;
        }
        
    }
}

/* [] END OF FILE */
