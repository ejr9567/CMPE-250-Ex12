#include "common.h"
#include "Accel_Driver.h"
#include "MKL05Z4.h"
#include "UART_Driver.h"

#define ACCEL_I2C_ADDR (0b0011101) // 001110 from factory, + SA0 is pulled high
#define ACCEL_I2C_ADDR_TX ((ACCEL_I2C_ADDR) << 1)
#define ACCEL_I2C_ADDR_RX ((ACCEL_I2C_ADDR) << 1 | 0b1)

static void accel_write_byte(UInt8 address, UInt8 value); // forward decl for Accel_Get_Velocity

#define Accel_Status    (0x00)
#define Accel_X_Out_MSB (0x01)
#define Accel_X_Out_LSB (0x02)
#define Accel_Y_Out_MSB (0x03)
#define Accel_Y_Out_LSB (0x04)
#define Accel_Z_Out_MSB (0x05)
#define Accel_Z_Out_LSB (0x06)
#define Accel_Who_Am_I  (0x0D)
#define Accel_Ctrl_Reg1 (0x2A)

void Init_Accel(void) {
    // enable clock to port B
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    // select ALT2 (I2C0_SCL "serial clock") for port B pin 3
    PORTB->PCR[3] |= 2 << PORT_PCR_MUX_SHIFT;
    // select ALT2 (I2C0_SDA "serial data") for port B pin 4
    PORTB->PCR[4] |= 2 << PORT_PCR_MUX_SHIFT;

    // set pull select to pull up for port B pins 3 and 4
    PORTB->PCR[3] |= PORT_PCR_PS_MASK;
    PORTB->PCR[4] |= PORT_PCR_PS_MASK;
    // enable pull up resistor for port B pins 3 and 4
    PORTB->PCR[3] |= PORT_PCR_PE_MASK;
    PORTB->PCR[4] |= PORT_PCR_PE_MASK;

    // enable clock to I2C0
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;

    // Set I2C config register values

    /* -- REGISTER C1
        bit 7 (IICEN) is set to enable the I2C module
        bit 6 (IICIE) is cleared to disable interrupts
        bit 5 (MST)   is left alone since it triggers the start signal
        bit 4 (TX)    is left alone since it chooses tx/rx for a single transmission
        bit 3 (TXAK)  is left alone since it chooses ack/nack for a single receive
        bit 2 (RSTA)  is left alone since it triggers a repeated start signal
        bit 1 (WUEN)  is left alone since we don't care about wakeup interrupt
        bit 0 (DMAEN) is cleared to disable DMA 
    */
    #define I2C_C1_ACCEL (I2C_C1_IICEN_MASK)
    I2C0->C1 = I2C_C1_ACCEL;


    /* -- REGISTER C2
        bit 7    (GCAEN)     is cleared since we don't want to send to all peripherals
        bit 6    (ADEXT)     is cleared since the peripheral uses the 7-bit addressing scheme
        XXX bit 5    (HDRS)      is cleared because we don't think we need high drive strength (?)
            -> trying setting drive high
        bit 4    (SBRC)      is left alone because it probably doesn't matter unless we are a peripheral
        bit 3    (RMEN)      is left alone because it definitely doesn't matter unless we are a peripheral
        bits 2-0 (AD[10:8])  are left alone because we are only using 7-bit addressing
    */
    #define I2C_C2_ACCEL (I2C_C2_HDRS_MASK)
    I2C0->C2 = I2C_C2_ACCEL;


    /* -- REGISTER F
        XXX bits 7-6 (MULT)      are 00 for multiplier of 1
            -> trying mult of 10
        XXX bits 5-0 (ICR)       01 1000 for clock rate of 0x18
            --> trying ICR of 00 0000
    */
    // #define I2C_F_ACCEL (0b00011000)
    // #define I2C_F_ACCEL (0b10000000)
    #define I2C_F_ACCEL (0b00001110)
    I2C0->F = I2C_F_ACCEL;

    #define ACCEL_CTRL_REG1_ACTIVE_SHIFT (0)
    #define ACCEL_CTRL_REG1_ACTIVE_MASK (1 << ACCEL_CTRL_REG1_ACTIVE_SHIFT)
    #define ACCEL_CTRL_REG1_F_READ_SHIFT (1)
    #define ACCEL_CTRL_REG1_F_READ_MASK (1 << ACCEL_CTRL_REG1_F_READ_SHIFT)
}

#define ACCEL_STATUS_XDR_SHIFT (1)
#define ACCEL_STATUS_XDR_MASK (1 << ACCEL_STATUS_XDR_SHIFT)

UInt32 Accel_Get_Velocity(void) {
    // lol

    // accel_write_byte(&(Accel->XYZ_Data_CFG), 0b00010011);
    // return accel_read_byte(&(Accel->XYZ_Data_CFG));

    return 69;

    // // return accel_read_byte((UInt8) (UInt32) (void*) &(Accel->Who_Am_I));
}

typedef struct {
    UInt8 first;
    UInt8 second;
} TwoBytes;

static TwoBytes accel_read_two_byte(UInt8 address);

float Accel_Get_Accel_X(void) {
    TwoBytes dataHAndL = accel_read_two_byte(Accel_X_Out_MSB);
    UInt8 dataH = dataHAndL.first;
    // PutStringSB("data H: ", 100000);
    // PrintByte(dataH);
    UInt8 dataL = dataHAndL.second;
    // PutStringSB(", data L: ", 10000);
    // PrintByte(dataL);
    // PutStringSB("\r\n", 2);
    UInt16 dataShiftedUp = ((UInt16) dataH << 8) | ((UInt16) dataL);
    Int16 dataShiftedWithCorrectSign = *(Int16*) &dataShiftedUp;
    Int16 data = dataShiftedWithCorrectSign >> 2;
    float dataFinal = ((float) data * 9.81) / 16384;
    return dataFinal;
}

// Wait for transmit to complete, and hang CPU if we did not receive ACK bit
static inline void accel_i2c_verify_transmit(void) {
    // Wait until Transfer Complete Flag is set (DISABLED):
    // while (!(I2C0->S & I2C_S_TCF_MASK));

    // For whatever reason, the it seems like the "transfer complete flag"
    // is set too soon. The I2C interrupt is the only reliable way to check
    // if the transfer has been completed.
    while (!(I2C0->S & I2C_S_IICIF_MASK));
    I2C0->S |= I2C_S_IICIF_MASK;

    if (I2C0->S & I2C_S_RXAK_MASK) { // No acknowledge signal was sent by peripheral
        while (1); // hang CPU in error case
    }
}

static TwoBytes accel_read_two_byte(UInt8 address) {
    // Ref: MMA8451Q datasheet, page 19 "I2C sequence diagrams"

    // Write start signal
    I2C0->C1 |= I2C_C1_MST_MASK;
    // Wait until start signal is detected
    while (!(I2C0->S & I2C_S_BUSY_MASK));
    // Tell the I2C module that we are performing a transmit
    I2C0->C1 |= I2C_C1_TX_MASK;
    // Write device address + W bit cleared
    I2C0->D = ACCEL_I2C_ADDR_TX;
    accel_i2c_verify_transmit();

    // Write register address
    I2C0->D = address;
    accel_i2c_verify_transmit();

    // Perform repeated start
    I2C0->C1 |= I2C_C1_RSTA_MASK;
    // Write device address + W bit set
    I2C0->D = ACCEL_I2C_ADDR_RX;
    accel_i2c_verify_transmit();

    // Tell the I2C module that we are performing a receive
    I2C0->C1 &= ~I2C_C1_TX_MASK;
    // Tell the I2C module to send an ACK after the following receive operation
    I2C0->C1 &= ~I2C_C1_TXAK_MASK;
    // Initiate read of data byte
    volatile /* don't optimize out :( */ UInt8 _ = I2C0->D;
    // Wait until interrupt condition is met (see accel_i2c_verify_transmit comment)
    while (!(I2C0->S & I2C_S_IICIF_MASK));
    I2C0->S |= I2C_S_IICIF_MASK;
    // Load data byte from I2C shift register
    UInt8 dataFirst = I2C0->D;

    // Tell the I2C module that we are performing a receive
    I2C0->C1 &= ~I2C_C1_TX_MASK;
    // Tell the I2C module to send a NACK after the following receive operation
    I2C0->C1 |= I2C_C1_TXAK_MASK;
    // Initiate read of data byte
    _ = I2C0->D;
    // Wait until interrupt condition is met (see accel_i2c_verify_transmit comment)
    while (!(I2C0->S & I2C_S_IICIF_MASK));
    I2C0->S |= I2C_S_IICIF_MASK;
    // Load data byte from I2C shift register
    UInt8 dataSecond = I2C0->D;

    // Write stop signal
    I2C0->C1 &= ~I2C_C1_MST_MASK;

    // The line stays busy? Don't bother infinite-looping,
    // while (I2C0->S & I2C_S_BUSY_MASK);
    // get your easy solution today! on sale for only 14,999 clock cycles
    // turn off module
    I2C0->C1 &= ~I2C_C1_IICEN_MASK;
    // turn on module
    I2C0->C1 |= I2C_C1_IICEN_MASK;

    TwoBytes ret = {.first = dataFirst, .second = dataSecond};

    return ret;
}

// static void accel_write_byte(UInt8 address, UInt8 value) {
//     // The waits in the conditions of write byte are
//     // specifically waiting for the acknowledge bit from peripheral.

//     // Tell I2C module we are transmitting
//     I2C0->C1 |= I2C_C1_TX_MASK;
//     // Send start signal
//     I2C0->C1 |= I2C_C1_MST_MASK;
//     // Transmit 7 bit device address + W bit
//     I2C0->D = ACCEL_I2C_ADDR_TX;
//     accel_i2c_verify_transmit();

//     // Transmit 8-bit address of register to write to 
//     I2C0->D = address; 
//     accel_i2c_verify_transmit();

//     // Transmit value to write to the register
//     I2C0->D = value;
//     accel_i2c_verify_transmit();
    
//     // Send stop signal (set back to peripheral mode)
//     I2C0->C1 &= ~I2C_C1_MST_MASK;

//     // The line stays busy? Don't bother infinite-looping,
//     // while (I2C0->S & I2C_S_BUSY_MASK);
//     // get your easy solution today! on sale for only 14,999 clock cycles
//     // turn off module
//     I2C0->C1 &= ~I2C_C1_IICEN_MASK;
//     // turn on module
//     I2C0->C1 |= I2C_C1_IICEN_MASK;
// }
