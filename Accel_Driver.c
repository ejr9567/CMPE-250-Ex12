#include "common.h"
#include "Accel_Driver.h"
#include "MKL05Z4.h"
#include "UART_Driver.h"
#include <math.h> // sqrt

#define ACCEL_I2C_ADDR (0b0011101) // 001110 from factory, + SA0 is pulled high
#define ACCEL_I2C_ADDR_TX ((ACCEL_I2C_ADDR) << 1)
#define ACCEL_I2C_ADDR_RX ((ACCEL_I2C_ADDR) << 1 | 0b1)

static void accel_read_bytes(UInt8 address, UInt8 numBytes, UInt8 *out); // forward decl for Accel_Get_Velocity
static UInt8 accel_read_byte(UInt8 address);
static void accel_write_byte(UInt8 address, UInt8 value); // forward decl for Accel_Get_Velocity

#define Accel_Status    (0x00)
#define Accel_X_Out_MSB (0x01)
#define Accel_X_Out_LSB (0x02)
#define Accel_Y_Out_MSB (0x03)
#define Accel_Y_Out_LSB (0x04)
#define Accel_Z_Out_MSB (0x05)
#define Accel_Z_Out_LSB (0x06)
#define Accel_Who_Am_I  (0x0D)
#define Accel_XYZ_Data_CFG (0x0E)
#define Accel_Ctrl_Reg1 (0x2A)
#define Accel_Ctrl_Reg2 (0x2B)

#define ACCEL_STATUS_XDR_SHIFT (1)
#define ACCEL_STATUS_XDR_MASK (1 << ACCEL_STATUS_XDR_SHIFT)

#define ACCEL_CTRL_REG1_ACTIVE_SHIFT (0)
#define ACCEL_CTRL_REG1_ACTIVE_MASK (1 << ACCEL_CTRL_REG1_ACTIVE_SHIFT)
#define ACCEL_CTRL_REG1_F_READ_SHIFT (1)
#define ACCEL_CTRL_REG1_F_READ_MASK (1 << ACCEL_CTRL_REG1_F_READ_SHIFT)

#define ACCEL_XYZ_DATA_CFG_FS_SHIFT (0)
#define ACCEL_XYZ_DATA_CFG_FS_MASK (0b11 << ACCEL_XYZ_DATA_CFG_FS_SHIFT)


#define FLOAT_NUM_AFTER_DECIMAL (4) // Number of digits to print of fractional part of float
#define FLOAT_POWER_AFTER_DECIMAL (10000) // 2 ** FLOAT_NUM_AFTER_DECIMAL
// Print floating-point value over UART.
void PutFloat(float val) {
    if (val < 0) {
        PutChar('-');
        val = val * ((float) -1);
    } else {
        PutChar('+');
    }
    UInt32 wholePart = val;
    PutNumU(wholePart);
    PutChar('.');
    float fractionalPart = val - (float) wholePart;
    float fractionalPartAboveRadixPoint = fractionalPart * (FLOAT_POWER_AFTER_DECIMAL);
    UInt32 fractionalPartAsInt = fractionalPartAboveRadixPoint;
    PutNumU(fractionalPartAsInt);
}

typedef struct {
    float x;
    float y;
    float z;
} Vec3;
Vec3 Vec3_add(Vec3 lhs, Vec3 rhs) {
    Vec3 result = {.x = lhs.x + rhs.x, .y = lhs.y + rhs.y, .z = lhs.z + rhs.z};
    return result;
}
Vec3 Vec3_scale(Vec3 vec, float fac) {
    Vec3 result = {.x = vec.x * fac, .y = vec.y * fac, .z = vec.z * fac};
    return result;
}
void Vec3_print(Vec3 vec) {
    PutFloat(vec.x);
    PutChar('\t');
    PutChar('\t');
    PutFloat(vec.y);
    PutChar('\t');
    PutChar('\t');
    PutFloat(vec.z);
}
static const Vec3 Vec3_zero = {.x = 0, .y = 0, .z = 0};

static Vec3 vel;
Vec3 accel_get_accel(void);

// Offset for acceleration due to Earth gravity.
// Set in Init_Accel
const Vec3 accel_initial_bias = {.x = 0, .y = 0, .z = 0};
Vec3 accel_bias;

// Board should not be moving!
void Init_Accel(void) {
    vel = Vec3_zero;

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
        bit 5    (HDRS)      is cleared because we don't need high drive strength
        bit 4    (SBRC)      is left alone because it probably doesn't matter unless we are a peripheral
        bit 3    (RMEN)      is left alone because it definitely doesn't matter unless we are a peripheral
        bits 2-0 (AD[10:8])  are left alone because we are only using 7-bit addressing
    */
    #define I2C_C2_ACCEL (0)
    I2C0->C2 = I2C_C2_ACCEL;


    /* -- REGISTER F
        bits 7-6 (MULT)      are 00 for multiplier of 1
        bits 5-0 (ICR)       are 01 1110 for clock rate of 0x14
    */
    #define I2C_F_ACCEL (0b00001110)
    I2C0->F = I2C_F_ACCEL;


    // Set accelerometer into standby
    accel_write_byte(Accel_Ctrl_Reg1, accel_read_byte(Accel_Ctrl_Reg1) & ~ACCEL_CTRL_REG1_ACTIVE_MASK);

    // Set full scale to +/- 4 G's
    #define ACCEL_XYZ_DATA_CFG_FS_4G (0b01 << ACCEL_XYZ_DATA_CFG_FS_SHIFT)
    accel_write_byte(Accel_XYZ_Data_CFG, accel_read_byte(Accel_XYZ_Data_CFG) & ~ACCEL_XYZ_DATA_CFG_FS_MASK | ACCEL_XYZ_DATA_CFG_FS_4G);

    // Set accelerometer back into active
    accel_write_byte(Accel_Ctrl_Reg1, accel_read_byte(Accel_Ctrl_Reg1) | ACCEL_CTRL_REG1_ACTIVE_MASK);
}

static volatile UInt8 calibrating = 0;
static volatile UInt32 calibrationSamplesCount = 0;
static volatile Vec3 accelBiasTotal = {.x = 0, .y = 0, .z = 0};

// Calculate acceleration bias by sampling acceleration due to Earth gravity
// Bias is added to every subsequent data point, so it should be the opposite of the current gravity
// to serve as a correction factor
void Accel_Calibrate(void) {
    PutStringSB("starting calibration, bias is ", 100);
    Vec3_print(accel_bias);
    PutStringSB(", total bias is ", 100);
    Vec3_print(accelBiasTotal);
    PutStringSB("\r\n", 2);
    calibrating = 1;
    while (calibrating);
//     accel_bias = accel_initial_bias;

//     Vec3 accel_bias_total = Vec3_zero;
//     // Sample acceleration repeatedly and take the average to get a more accurate estimate
#define BIAS_SAMPLE_CNT (100)
//     for (int biasSample = 0; biasSample < BIAS_SAMPLE_CNT; biasSample++) {
//         Vec3 sample = accel_get_accel();
//         accel_bias_total = Vec3_add(accel_bias_total, sample);
//     }
}


float Accel_Get_Velocity_X_Y(void) {
    return sqrt((vel.x * vel.x) + (vel.y * vel.y));
}

UInt32 Accel_Get_Velocity(void) {
    // lol

    // accel_write_byte(&(Accel->XYZ_Data_CFG), 0b00010011);
    // return accel_read_byte(&(Accel->XYZ_Data_CFG));

    return 69;

    // // return accel_read_byte((UInt8) (UInt32) (void*) &(Accel->Who_Am_I));
}

float accel_interpret_data_sample(UInt8 high, UInt8 low);

Vec3 accel_get_accel(void) {
    // Three axes, two bytes per axis = six bytes total

    UInt8 rawVec3[6];
    accel_read_bytes(Accel_X_Out_MSB, 6, rawVec3);

    Vec3 data;
    data.x = accel_interpret_data_sample(rawVec3[0], rawVec3[1]);
    data.y = accel_interpret_data_sample(rawVec3[2], rawVec3[3]);
    data.z = accel_interpret_data_sample(rawVec3[4], rawVec3[5]);

    Vec3 correctedData = Vec3_add(data, accel_bias);

    return correctedData;
}

#define VEL_UPDATE_TIMESTEP ((float) 0.01)
// #define VEL_UPDATE_TIMESTEP ((float) 0.001)

// should be called once every 0.01s by PIT driver.
void Accel_Update_Velocity(void) {
    if (calibrating) {
        Vec3 sample = accel_get_accel();
        accelBiasTotal = Vec3_add(accelBiasTotal, sample);
        calibrationSamplesCount++;

        if (calibrationSamplesCount == BIAS_SAMPLE_CNT) {
            accel_bias = Vec3_scale(accelBiasTotal, (float) (((float) -1) / ((float) BIAS_SAMPLE_CNT)));
            accelBiasTotal = Vec3_zero;
            calibrationSamplesCount = 0;

            PutStringSB("calibration just finished, total bias is ", 100);
            Vec3_print(accelBiasTotal);
            PutStringSB(", bias is ", 100);
            Vec3_print(accel_bias);
            PutStringSB("\r\n", 2);

            calibrating = 0;
        }
    } else {
        Vec3 currentAccel = accel_get_accel();
        Vec3 deltaVel = Vec3_scale(currentAccel, VEL_UPDATE_TIMESTEP);
        vel = Vec3_add(vel, deltaVel);
    }
}

void Accel_Reset_Velocity(void) {
    __asm(" CPSID I");
    vel = Vec3_zero;
    __asm(" CPSIE I");
}

// Unit conversion for G to m/s^2
#define m_s_2_per_g ((float) 9.81)
// We use the accelerometer in +/- 4 G mode
#define accel_range_G ((float) 4)

// returns resultant acceleration in m/s^2
float accel_interpret_data_sample(UInt8 high, UInt8 low) {
    // high gives !!!!!! !!------ of sample
    // all bits are used (!!!!!!!!)

    // low gives ------ --!!!!!! of sample
    // ignore bottom two bits (!!!!!!xx)

    // put sample into format: !!!!!!!! !!!!!!xx
    UInt16 dataShiftedUp = ((UInt16) high << 8) | ((UInt16) low);
    // reinterpret as signed integer (MSB should be used as sign bit)
    Int16 dataShiftedWithCorrectSign = *(Int16*) &dataShiftedUp;
    // sign-extended right shift to format: SS!!!!!! !!!!!!!!
    Int16 data = dataShiftedWithCorrectSign >> 2;

    return ((float) data * (m_s_2_per_g * accel_range_G)) / ((float) 8192) /* expected range of signed value, 2 ** 13 */;
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

static inline void accel_i2c_wait_for_transfer_end(void) {
    // Not sure why this doesn't work to wait until the line is no longer busy.
    // while (I2C0->S & I2C_S_BUSY_MASK);

    // Stopgap solution is to reset the module:
    // turn off module
    I2C0->C1 &= ~I2C_C1_IICEN_MASK;
    // turn on module
    I2C0->C1 |= I2C_C1_IICEN_MASK;
}

static void accel_read_bytes(UInt8 address, UInt8 numBytes, UInt8 *out) {
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

    for (UInt8 byteIndex = 0; byteIndex < numBytes; byteIndex++) {
        // Tell the I2C module that we are performing a receive
        I2C0->C1 &= ~I2C_C1_TX_MASK;

        // Last byte being read?
        if (byteIndex == (numBytes - 1)) {
            // Send a NACK

            // Tell the I2C module to send a NACK after the following receive operation
            I2C0->C1 |= I2C_C1_TXAK_MASK;
        } else {
            // Send an ACK

            // Tell the I2C module to send an ACK after the following receive operation
            I2C0->C1 &= ~I2C_C1_TXAK_MASK;
        }

        // Initiate read of data byte
        volatile /* don't optimize out :( */ UInt8 _ = I2C0->D;
        // Wait until interrupt condition is met (see accel_i2c_verify_transmit comment)
        while (!(I2C0->S & I2C_S_IICIF_MASK));
        I2C0->S |= I2C_S_IICIF_MASK;
        // Load data byte from I2C shift register
        UInt8 data = I2C0->D;

        out[byteIndex] = data;
    }

    // Write stop signal
    I2C0->C1 &= ~I2C_C1_MST_MASK;

    // Wait until lines go high
    accel_i2c_wait_for_transfer_end();
}

// Read a single byte from the accelerometer
static UInt8 accel_read_byte(UInt8 address) {
    UInt8 byte[1];
    accel_read_bytes(address, 1, byte);
    return byte[0];
}

static void accel_write_byte(UInt8 address, UInt8 value) {
    // The waits in the conditions of write byte are
    // specifically waiting for the acknowledge bit from peripheral.

    // Tell I2C module we are transmitting
    I2C0->C1 |= I2C_C1_TX_MASK;
    // Send start signal
    I2C0->C1 |= I2C_C1_MST_MASK;
    // Transmit 7 bit device address + W bit
    I2C0->D = ACCEL_I2C_ADDR_TX;
    accel_i2c_verify_transmit();

    // Transmit 8-bit address of register to write to 
    I2C0->D = address; 
    accel_i2c_verify_transmit();

    // Transmit value to write to the register
    I2C0->D = value;
    accel_i2c_verify_transmit();
    
    // Send stop signal (set back to peripheral mode)
    I2C0->C1 &= ~I2C_C1_MST_MASK;

    // Wait until lines go high
    accel_i2c_wait_for_transfer_end();
}
