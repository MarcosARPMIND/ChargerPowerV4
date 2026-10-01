#ifndef INC_ENERGY_METER_H_
#define INC_ENERGY_METER_H_


/* --- Standard Includes --- */
#include <stdint.h>
#include <stdbool.h>

/* --- Hardware Abstraction Includes --- */
#include "spi.h"
#include "board_io.h"
#include "comms_manager.h"
#include "tim.h"


/* --- Protocol Constants --- */
/**
 * @brief SPI Timeout definition.
 * Calculation: 1.5 Mbps = ~5.3us/byte. Max packet (7 bytes) = ~38us.
 * 2ms provides ample margin for OS interrupts and HAL overhead.
 */
#define SPI_TIMEOUT_MS 2

#define ADE_WRITE_FLAG          0x00
#define ADE_READ_FLAG           0x80

#define WINDOW_SIZE_SHIFT 4
#define WINDOW_SIZE       (1 << WINDOW_SIZE_SHIFT)


/* --- Calibration factors, one per ADE7953 (device1 = line 1) ---
 * VRMS / IRMS: Q16.16 multipliers (result = raw * factor >> 16), in mV / mA.
 * POWER:       plain multiplier (result = raw * factor), in mW.
 * 65536u marks a channel that is NOT calibrated yet (unity Q16.16 scale, or
 * a placeholder for power) -- its readings are not meaningful. */
#define VRMS_LSB_FP16_device3  2427u
#define VRMS_LSB_FP16_device2  2427u
#define VRMS_LSB_FP16_device1  2427u

#define IRMS_LSB_mA_device3     65536u   // not calibrated
#define IRMS_LSB_mA_device2     65536u   // not calibrated
#define IRMS_LSB_mA_device1     874u     // calibrated against a clamp meter (~9 A)

#define POWER_LSB_mW_device3    65536u   // not calibrated
#define POWER_LSB_mW_device2    65536u   // not calibrated
#define POWER_LSB_mW_device1    65536u   // not calibrated

/* =================================================================================
 * REGISTER MAP
 * Based on ADE7953 Datasheet Rev. C
 * ================================================================================= */

/* --- Configuration & Control (8-bit / 16-bit) --- */
#define ADDR_SAGCYC             0x0000  // Sag line cycles (8-bit)
#define ADDR_DISNOLOAD          0x0001  // No-load detection disable (8-bit) [cite: 2850]
#define ADDR_LCYCMODE           0x0004  // Line cycle accumulation mode (8-bit)
#define ADDR_PGA_V              0x0007  // Gain for Voltage channel (8-bit)
#define ADDR_PGA_IA             0x0008  // Gain for Current channel A (8-bit)
#define ADDR_PGA_IB             0x0009  // Gain for Current channel B (8-bit)
#define ADDR_WRITE_PROTECT      0x0040  // Write protection bits (8-bit)
#define ADDR_UNLOCK_RESERVED    0x00FE  // Address to write 0xAD to unlock reg 0x120 (8-bit)
#define ADDR_CONFIG_UNLOCK      0x0120  // Special config register (16-bit) -> Set to 0x0030
#define ADDR_CONFIG             0x0102  // Main configuration register (16-bit)
#define ADDR_VERSION            0x0702  // Silicon Version (8-bit, read-only)

/* --- Accumulation Mode & No-Load (24/32-bit) --- */
#define ADDR_ACCMODE            0x0201  // Accumulation mode (24-bit)
#define ADDR_AP_NOLOAD          0x0203  // Active Power No-Load Threshold (24-bit) [cite: 2842]

/* --- Calibration (24-bit) --- */
// Using 0x2xx addresses is standard for calibration registers
#define ADDR_AIGAIN             0x0280  // Current channel A gain
#define ADDR_AVGAIN             0x0281  // Voltage channel gain
#define ADDR_AWGAIN             0x0282  // Active power gain A
#define ADDR_PHCALA             0x0108  // Phase calibration A (16-bit)

/* --- Interrupts & Status (24-bit/32-bit) --- */
#define ADDR_IRQENA             0x032C  // Interrupt Enable A (32-bit) [cite: 2881]
#define ADDR_IRQSTATA           0x032D  // Interrupt Status A (32-bit) [cite: 2883]
#define ADDR_RSTIRQSTATA        0x032E  // Reset Interrupt Status A (32-bit, Read-only)
#define ADDR_IRQENB             0x032F  // Interrupt Enable B (32-bit) [cite: 2891]
#define ADDR_IRQSTATB           0x0330  // Interrupt Status B (32-bit) [cite: 2892]
#define ADDR_RSTIRQSTATB        0x0331  // Reset Interrupt Status B (32-bit, Read-only)

/* =================================================================================
 * MEASUREMENT REGISTERS
 * NOTE: We use 0x3xx addresses (32-bit) for signed values.
 * This allows the ADE7953 to perform automatic Sign Extension.
 * Reading these into an int32_t variable yields the correct negative values immediately.
 * ================================================================================= */

/* --- RMS & Instantaneous Power (32-bit) --- */
#define ADDR_IRMSA              0x031A  // IRMS A (32-bit, unsigned) [cite: 2842]
#define ADDR_IRMSB              0x031B  // IRMS B (32-bit, unsigned)
#define ADDR_VRMS               0x031C  // VRMS (32-bit, unsigned)

#define ADDR_AWATT              0x0312  // Instantaneous Active Power A (32-bit, signed)
#define ADDR_BWATT              0x0313  // Instantaneous Active Power B (32-bit, signed)
#define ADDR_AVAR               0x0314  // Instantaneous Reactive Power A (32-bit, signed)
#define ADDR_BVAR               0x0315  // Instantaneous Reactive Power B (32-bit, signed)
#define ADDR_AVA                0x0310  // Instantaneous Apparent Power A (32-bit, signed)
#define ADDR_BVA                0x0311  // Instantaneous Apparent Power B (32-bit, signed)

/* --- Power Factor & Angle (16-bit) --- */
#define ADDR_PFA                0x010A  // Power factor A (16-bit, signed)
#define ADDR_ANGLE_A            0x010C  // Angle A (16-bit, signed)

/* --- Peaks (32-bit) --- */
#define ADDR_VPEAK              0x0326  // Voltage Peak (32-bit, unsigned)
#define ADDR_IAPEAK             0x0328  // Current A Peak (32-bit, unsigned)

/* --- Energy Accumulation (32-bit) --- */
#define ADDR_AENERGYA           0x031E  // Active Energy A (32-bit, signed)
#define ADDR_AENERGYB           0x031F  // Active Energy B (32-bit, signed)
#define ADDR_RENERGYA           0x0320  // Reactive Energy A (32-bit, signed)
#define ADDR_RENERGYB           0x0321  // Reactive Energy B (32-bit, signed)
#define ADDR_APENERGYA          0x0322  // Apparent Energy A (32-bit, signed)

/* --- Period (16-bit) --- */
#define ADDR_PERIOD             0x010E  // Line Period (16-bit, unsigned)

#define EMA_ALPHA_SHIFT  3

/* --- Device Definitions --- */
typedef enum {
    ADE_DEVICE_1 = 0,
    ADE_DEVICE_2,
    ADE_DEVICE_3
} ADE_DEVICE;

typedef struct {
	uint32_t vrms_mV;
	uint32_t active_power_W;
	uint32_t session_energy_Wh;
	uint32_t current_mA;
	uint16_t freq_dec_Hz;
}Energy_Data_t;

typedef struct {
    uint32_t buffer[WINDOW_SIZE];
    uint32_t sum;
    uint8_t  index;
    uint8_t  count; // 0 until the first sample seeds the whole window
} SlidingWindow_t;

/* =================================================================================
 * PUBLIC FUNCTIONS
 * ================================================================================= */

// Initialization
HAL_StatusTypeDef ADE7953_INIT(ADE_DEVICE device);

// Low-Level Access
HAL_StatusTypeDef ade7953_Write_Reg(ADE_DEVICE dev, uint16_t addr, uint32_t data, uint8_t num_bytes);
uint32_t ade7953_Read_Reg(ADE_DEVICE dev, uint16_t addr, uint8_t num_bytes);

// Readings
uint32_t 	ade7953_get_vrms_mV(ADE_DEVICE device);
uint32_t 	ade7953_get_irms_mA(ADE_DEVICE device);
int32_t 	ade7953_get_active_power_mW(ADE_DEVICE device);
int32_t 	ade7953_get_reactive_power_mVAR(ADE_DEVICE device);
int32_t 	ade7953_get_apparent_power_mVA(ADE_DEVICE device);
int32_t  	ade7953_get_active_energy_a(ADE_DEVICE device);
uint16_t    ade7953_get_line_frequency(ADE_DEVICE device);

// Status Checks
bool     ade7953_is_zero_crossing_v(void);

void ade7953_enable_reset_on_read(ADE_DEVICE device);

// Utilities
void delay_us(void);
void Board_Set_SPI_CS(ADE_DEVICE device, PIN_STATE state);
void Board_ADE_Reset(ADE_DEVICE device, PIN_STATE state);
bool ade7953_calibrate_vrms(ADE_DEVICE device, uint16_t num_samples, uint32_t *result);

#endif /* INC_ENERGY_METER_H_ */
