/*
 * energy_meter.c
 *
 * Created on: Dec 9, 2025
 * Author: ARPMindTech
 *
 * Description:
 * Driver implementation for the Analog Devices ADE7953 Single-Phase Energy Metering IC.
 * This driver handles SPI communication, initialization sequences, and register read/write operations.
 * It includes specific handling for the ADE7953 "Communication Locking" mechanism required at startup.
 */

#include "energy_meter.h"
#include "stm32c0xx_hal.h"

static SlidingWindow_t vrms_window[3] = {0};
static uint32_t process_sliding_window(SlidingWindow_t *win, uint32_t new_sample);
/**
 * @brief  Initializes the ADE7953 device via SPI.
 * @note   This function performs a critical startup sequence mandated by the ADE7953 datasheet:
 * 1. Locks the communication interface to SPI mode.
 * 2. Unlocks internal reserved registers.
 * 3. Writes specific optimization values to reserved registers.
 * 4. Verifies communication by reading the silicon version.
 *
 * @param  dev: The specific ADE device instance (ADE_DEVICE_1, etc.) to initialize.
 * @return HAL_OK if initialization and version check are successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ADE7953_INIT(ADE_DEVICE dev) {
    uint8_t tx_buff[5]; // Buffer for manual packet construction (2 addr + 1 cmd + 2 data)

    // ---------------------------------------------------------
    // STEP 1: Lock SPI Mode and Enable HPF
    // Target Register: CONFIG (0x102) | Value: 0x0004
    // ---------------------------------------------------------
    // The datasheet requires a specific timing sequence to "lock" the communication interface
    // to SPI mode. We manually construct the packet to control the Chip Select (CS) timing precisely.

    tx_buff[0] = (ADDR_CONFIG >> 8) & 0xFF; // Address High Byte
    tx_buff[1] = ADDR_CONFIG & 0xFF;        // Address Low Byte
    tx_buff[2] = ADE_WRITE_FLAG;            // Write Command (0x00)
    tx_buff[3] = (0x0004 >> 8) & 0xFF;      // Data MSB (0x00)
    tx_buff[4] = 0x04;                      // Data LSB (0x04) - Bit 2 (HPFEN) enabled, Bit 15 (COMM_LOCK) cleared

    Board_Set_SPI_CS(dev, CMD_ACTIVATE);

    if (HAL_SPI_Transmit(&hspi1, tx_buff, 5, SPI_TIMEOUT_MS) != HAL_OK) {
        Board_Set_SPI_CS(dev, CMD_DEACTIVATE);
        return HAL_ERROR;
    }

    // --- CRITICAL DELAY FOR LOCKING ---
    // According to ADE7953 Datasheet (SPI Interface Timing):
    // Parameter t_SFS_LK (CS High after SCLK) must be >= 1.2 us to successfully lock the port.
    // We use a precision microsecond delay here to ensure this requirement is met before toggling CS.
    delay_us();
    // Note: HAL_Delay(1) could also be used here since 1ms >> 1.2us, providing a safe margin during init.

    Board_Set_SPI_CS(dev, CMD_DEACTIVATE);

    // Small stabilization delay after locking the interface
    HAL_Delay(1);

    // ---------------------------------------------------------
    // STEP 2: Unlock Reserved Register 0x120
    // Operation: Write 0xAD to Register 0xFE
    // ---------------------------------------------------------
    // The datasheet specifies that register 0x120 is password-protected.
    // We must write the key 0xAD to address 0xFE to unlock it.
    ade7953_Write_Reg(dev, ADDR_UNLOCK_RESERVED, 0xAD, 1);

    // ---------------------------------------------------------
    // STEP 3: Internal Optimization Configuration
    // Operation: Write 0x0030 to Register 0x120
    // ---------------------------------------------------------
    // This value is required for optimal performance as per the datasheet specifications.
    ade7953_Write_Reg(dev, ADDR_CONFIG_UNLOCK, 0x0030, 2);

    // ---------------------------------------------------------
    // STEP 4: Communication Verification
    // Operation: Read Silicon Version (Register 0x702)
    // ---------------------------------------------------------
    // We read the 8-bit version register to verify that SPI communication is bidirectional
    // and the chip is responding correctly.
    uint8_t version = (uint8_t)ade7953_Read_Reg(dev, ADDR_VERSION, 1);

    // Check for known silicon revisions (typically 0x02 or 0x03)
    if (version == 0x02 || version == 0x03) {
        return HAL_OK;
    }

    return HAL_ERROR; // Communication failed or unknown silicon version
}

/**
 * @brief  Reads data from an ADE7953 register via SPI.
 * @param  dev: The specific ADE device instance.
 * @param  addr: The 16-bit register address to read from.
 * @param  num_bytes: The number of bytes to read (1, 2, or 4).
 * @return The value read from the register (up to 32 bits).
 */
uint32_t ade7953_Read_Reg(ADE_DEVICE dev, uint16_t addr, uint8_t num_bytes){

    uint8_t tx_buff[3];       // Buffer for header (Addr High, Addr Low, Read Cmd)
    uint8_t rx_buff[4] = {0}; // Buffer for received data (max 32 bits)
    uint32_t read_value = 0;

    // 1. Prepare Header (Big Endian format required by ADE7953)
    tx_buff[0] = (addr >> 8) & 0xFF; // Address High Byte
    tx_buff[1] = addr & 0xFF;        // Address Low Byte
    tx_buff[2] = ADE_READ_FLAG;      // Command: Read (0x80)

    // 2. Start Transaction
    Board_Set_SPI_CS(dev, CMD_ACTIVATE);

    // 3. Send Header
    if(HAL_SPI_Transmit(&hspi1, tx_buff, 3, SPI_TIMEOUT_MS) == HAL_OK)
    {
        // 4. Receive Data Payload
        // The ADE7953 clocks out data MSB first immediately after the header.
        HAL_SPI_Receive(&hspi1, rx_buff, num_bytes, SPI_TIMEOUT_MS);
    }

    // 5. End Transaction
    Board_Set_SPI_CS(dev, CMD_DEACTIVATE);

    // 6. Reconstruct the 32-bit value from the received bytes (MSB at index 0)
    for(uint8_t i = 0; i < num_bytes; i++){
        read_value = (read_value << 8) | rx_buff[i];
    }

    return read_value;
}

/**
 * @brief  Writes data to an ADE7953 register via SPI.
 * @param  dev: The specific ADE device instance.
 * @param  addr: The 16-bit register address to write to.
 * @param  data: The data value to write (up to 32 bits).
 * @param  num_bytes: The number of bytes to write (1, 2, or 4).
 * @return HAL Status of the SPI transmission.
 */
HAL_StatusTypeDef ade7953_Write_Reg(ADE_DEVICE dev, uint16_t addr, uint32_t data, uint8_t num_bytes){

    uint8_t tx_buff[7]; // Buffer size: 3 bytes header + max 4 bytes data
    uint8_t id = 0;

    // 1. Construct Header
    tx_buff[id++] = (addr >> 8) & 0xFF; // Address High Byte [cite: 2507]
    tx_buff[id++] = addr & 0xFF;        // Address Low Byte [cite: 2507]
    tx_buff[id++] = 0x00;               // Command: Write (0x00) [cite: 2516]

    // 2. Construct Data Payload
    // Data must be sent MSB first. We iterate backwards from the most significant byte.
    // NOTE: 'i' must be a signed type (int8_t) to allow the loop condition (i >= 0) to terminate correctly.
    for(int8_t i = num_bytes - 1; i >= 0; i--){
        tx_buff[id++] = (uint8_t)((data >> (i * 8)) & 0xFF);
    }

    // 3. Perform SPI Transaction
    Board_Set_SPI_CS(dev, CMD_ACTIVATE);

    // Transmit Header + Data in a single continuous burst
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, tx_buff, id, SPI_TIMEOUT_MS);

    Board_Set_SPI_CS(dev, CMD_DEACTIVATE);

    return status;
}

/**
 * @brief  Blocking microsecond delay using a hardware timer (TIM14).
 * @note   This function assumes TIM14 is initialized and running.
 * It handles timer overflow automatically via unsigned arithmetic.
 */
void delay_us(void){

    // Capture starting count
    uint16_t start = (uint16_t)TIM14->CNT;

    // Define target ticks for ~1.25us delay
    // Assuming Timer Clock = 12MHz (approx 83ns per tick) -> 16 ticks ~= 1.33us
    // Value set to 20 for a safe margin (~1.6us).
    const uint16_t Ticks_delay = 20;

    // Blocking wait loop
    // (Current - Start) handles unsigned overflow naturally
    while( (uint16_t)(TIM14->CNT - start) < Ticks_delay ) {
        __NOP();
    }
}

/**
 * @brief  Controls the Chip Select (CS) pin for the specified ADE device.
 * @param  device: The target ADE device (1, 2, or 3).
 * @param  state: CMD_ACTIVATE (Low) or CMD_DEACTIVATE (High).
 */
void Board_Set_SPI_CS(ADE_DEVICE device, PIN_STATE state){

    switch(device){
        case ADE_DEVICE_1:
            Board_Set_SPI_CS_1(state);
            break;

        case ADE_DEVICE_2:
            Board_Set_SPI_CS_2(state);
            break;

        case ADE_DEVICE_3:
            Board_Set_SPI_CS_3(state);
            break;
    }
}

/**
 * @brief  Controls the Hardware Reset pin for the specified ADE device.
 * @param  device: The target ADE device (1, 2, or 3).
 * @param  state: CMD_ACTIVATE (Low=Reset) or CMD_DEACTIVATE (High=Run).
 */
void Board_ADE_Reset(ADE_DEVICE device, PIN_STATE state){

    switch(device){
        case ADE_DEVICE_1:
            Board_ADE_Reset_1(state);
            break;

        case ADE_DEVICE_2:
            Board_ADE_Reset_2(state);
            break;

        case ADE_DEVICE_3:
            Board_ADE_Reset_3(state);
            break;
    }
}

/**
 * @brief  Enables "Reset on Read" mode for energy registers.
 * @param  device Target ADE device.
 * @note   When enabled, reading an energy register (like AENERGYA) automatically
 *         clears its value to 0. This is useful for accumulating energy intervals.
 */
void ade7953_enable_reset_on_read(ADE_DEVICE device) {
    // Bit 6 (RSTREAD) = 1: Energy registers reset to 0 after read.
    uint8_t lcycmode = 0x40;
    ade7953_Write_Reg(device, ADDR_LCYCMODE, lcycmode, 1);
}

/**
 * @brief  Reads the RMS Voltage (Line 1).
 * @param  device Target ADE device.
 * @return 32-bit register value (needs conversion factor to get mVolts).
 */
/*
uint32_t ade7953_get_vrms_mV(ADE_DEVICE device) {
    uint32_t raw = ade7953_Read_Reg(device, ADDR_VRMS,4);
    // raw * 10027 >> 16 ≈ raw * 0.153
    return (uint32_t)(((uint64_t)raw * VRMS_LSB_FP16) >> 16);
}
*/
uint32_t ade7953_get_vrms_mV(ADE_DEVICE device) {
    uint32_t raw = ade7953_Read_Reg(device, ADDR_VRMS, 4);

    uint16_t VRMS_FACTOR = 0;
    switch(device){

    case ADE_DEVICE_1:

    	VRMS_FACTOR = VRMS_LSB_FP16_device1;

    	break;

    case ADE_DEVICE_2:

    	VRMS_FACTOR = VRMS_LSB_FP16_device2;
    	break;

    case ADE_DEVICE_3:

    	VRMS_FACTOR = VRMS_LSB_FP16_device3;
    	break;

    default:
    	break;

    }

    // (Ainda usando a tua macro antiga ou as calibrações dinâmicas que falámos antes)
    uint32_t new_sample = (uint32_t)(((uint64_t)raw * VRMS_FACTOR) >> 16);
    uint8_t idx = (uint8_t)device;
    // Processar o filtro de média móvel
    return process_sliding_window(&vrms_window[idx], new_sample);
}

/**
 * @brief  Reads the RMS Current (Phase A).
 * @param  device Target ADE device.
 * @return 32-bit register value (needs conversion factor to get mAmps).
 */
uint32_t ade7953_get_irms_mA(ADE_DEVICE device) {
    uint32_t raw = ade7953_Read_Reg(device, ADDR_IRMSA,4);



    uint32_t IRMS_FACTOR = 0;


    switch(device){

    case ADE_DEVICE_1:

    	IRMS_FACTOR = IRMS_LSB_mA_device1;

    	break;

    case ADE_DEVICE_2:

    	IRMS_FACTOR = IRMS_LSB_mA_device2;
    	break;

    case ADE_DEVICE_3:

    	IRMS_FACTOR = IRMS_LSB_mA_device3;
    	break;

    default:
    	break;

    }
    uint32_t new_sample = (uint32_t)(((uint64_t)raw * IRMS_FACTOR) >> 16);
    return new_sample;
}

/**
 * @brief  Reads the Active Power (Phase A).
 * @param  device Target ADE device.
 * @return Signed 32-bit value (Positive = Import, Negative = Export).
 */

int32_t ade7953_get_active_power_mW(ADE_DEVICE device) {
    int32_t raw = (int32_t)ade7953_Read_Reg(device, ADDR_AWATT, 4);
    uint32_t pwr_factor = 0;
    
    switch(device) {
        case ADE_DEVICE_1:
            pwr_factor = POWER_LSB_mW_device1;
            break;
        case ADE_DEVICE_2:
            pwr_factor = POWER_LSB_mW_device2;
            break;
        case ADE_DEVICE_3:
            pwr_factor = POWER_LSB_mW_device3;
            break;
        default:
            pwr_factor = 1; // Fallback to avoid mult by 0
            break;
    }
    return (int32_t)(raw * pwr_factor);
}

/**
 * @brief  Reads the Reactive Power (Phase A).
 * @param  device Target ADE device.
 * @return Signed 32-bit value (Positive = Import, Negative = Export).
 */
int32_t ade7953_get_reactive_power_mVAR(ADE_DEVICE device) {
    int32_t raw = (int32_t)ade7953_Read_Reg(device, ADDR_AVAR, 4);
    uint32_t pwr_factor = 0;
    
    switch(device) {
        case ADE_DEVICE_1:
            pwr_factor = POWER_LSB_mW_device1;
            break;
        case ADE_DEVICE_2:
            pwr_factor = POWER_LSB_mW_device2;
            break;
        case ADE_DEVICE_3:
            pwr_factor = POWER_LSB_mW_device3;
            break;
        default:
            pwr_factor = 1;
            break;
    }
    return (int32_t)(raw * pwr_factor);
}

/**
 * @brief  Reads the Apparent Power (Phase A).
 * @param  device Target ADE device.
 * @return Signed 32-bit value (Positive = Import, Negative = Export).
 */
int32_t ade7953_get_apparent_power_mVA(ADE_DEVICE device) {
    int32_t raw = (int32_t)ade7953_Read_Reg(device, ADDR_AVA, 4);
    uint32_t pwr_factor = 0;
    
    switch(device) {
        case ADE_DEVICE_1:
            pwr_factor = POWER_LSB_mW_device1;
            break;
        case ADE_DEVICE_2:
            pwr_factor = POWER_LSB_mW_device2;
            break;
        case ADE_DEVICE_3:
            pwr_factor = POWER_LSB_mW_device3;
            break;
        default:
            pwr_factor = 1;
            break;
    }
    return (int32_t)(raw * pwr_factor);
}

/**
 * @brief  Reads the Active Energy accumulation (Phase A).
 * @param  device Target ADE device.
 * @return Signed 32-bit energy accumulator value.
 */
int32_t ade7953_get_active_energy_a(ADE_DEVICE device){
	return (int32_t)ade7953_Read_Reg(device, ADDR_AENERGYA,4 );
}

/**
 * @brief  Reads the Line Frequency.
 * @param  device Target ADE device.
 * @return Floating point frequency (Hz).
 * @todo   Implement calculation using PERIOD register.
 */
uint16_t ade7953_get_line_frequency(ADE_DEVICE device){

	uint16_t raw_period = ade7953_Read_Reg(device, ADDR_PERIOD, 2);

	if(raw_period > 0){
		return (uint16_t)( 2237500UL / raw_period);
	}
	else{
		return 0;
	}
}

static uint32_t process_sliding_window(SlidingWindow_t *win, uint32_t new_sample) {
    // Se for a primeiríssima amostra (count == 0), preenchemos tudo com ela.
    // Assim eliminamos o tempo de aquecimento e as transições mortas.
    if (win->count == 0) {
        for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
            win->buffer[i] = new_sample;
        }
        win->sum = new_sample * WINDOW_SIZE; // ou (new_sample << WINDOW_SIZE_SHIFT)
        win->index = 0;
        win->count = WINDOW_SIZE; // Marca como permanentemente cheio

        return new_sample;
    }
    // Código ultra-rápido SEM divisões:
    // 1. Envolve o índice
    if (win->index >= WINDOW_SIZE) win->index = 0;
    // 2. Remove a amostra mais velha do somatório
    win->sum -= win->buffer[win->index];

    // 3. Guarda a amostra mais recente
    win->buffer[win->index] = new_sample;

    // 4. Adiciona a nova ao somatório
    win->sum += new_sample;
    win->index++;
    // 5. Retorna a média usando bit shift (Ex: >> 4 equivale a dividir por 16)
    return (win->sum >> WINDOW_SIZE_SHIFT);
}



/**
 * @brief  Non-blocking calibration. Call once per 100ms flag.
 * @param  device Target ADE device.
 * @param  num_samples Total samples to average.
 * @param  result Pointer to store the final average raw value.
 * @return true when calibration is complete, false while still collecting.
 */
bool ade7953_calibrate_vrms(ADE_DEVICE device, uint16_t num_samples, uint32_t *result) {
    static uint64_t accumulator = 0;
    static uint16_t count = 0;
    accumulator += ade7953_Read_Reg(device, ADDR_VRMS, 4);
    count++;
    if (count >= num_samples) {
        *result = (uint32_t)(accumulator / num_samples);
        // Reset para eventual reutilização
        accumulator = 0;
        count = 0;
        return true;   // acabou
    }
    return false;  // ainda a recolher
}
