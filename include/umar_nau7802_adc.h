

// Using legacy I2C driver (driver/i2c.h) as my controller requires it. It can be updated (driver/i2c_master.h) easily.
#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c.h>


/* Bits within PU_CTRL register */
typedef enum {
    NAU7802_PU_CTRL_RR    = 1,
    NAU7802_PU_CTRL_PUD   = 2,
    NAU7802_PU_CTRL_PUA   = 4,
    NAU7802_PU_CTRL_PUR   = 8,
    NAU7802_PU_CTRL_CS    = 16,
    NAU7802_PU_CTRL_CR    = 32,
    NAU7802_PU_CTRL_OSCS  = 64,
    NAU7802_PU_CTRL_AVDDS = 128,
} PU_CTRL_Bits;

/* Bits within PGA register */
typedef enum {
    NAU7802_PGA_CHPDIS  = 1,   // chopper disable
    NAU7802_PGA_RES1    = 2,   // reserved
    NAU7802_PGA_RES2    = 4,   // reserved
    NAU7802_PGA_PGAINV  = 8,   // invert pga phase
    NAU7802_PGA_BYPASS  = 16,  // pga bypass
    NAU7802_PGA_DISBUF  = 32,  // disable pga output buffer
    NAU7802_PGA_LDOMODE = 64,  // pga ldomode
    NAU7802_PGA_OTPSEL  = 128, // output select
} PGA_Bits;

/* NAU7802 register addresses */
typedef enum {
    NAU7802_PU_CTRL = 0x00,
    NAU7802_CTRL1,
    NAU7802_CTRL2,
    NAU7802_OCAL1_B2,
    NAU7802_OCAL1_B1,
    NAU7802_OCAL1_B0,
    NAU7802_GCAL1_B3,
    NAU7802_GCAL1_B2,
    NAU7802_GCAL1_B1,
    NAU7802_GCAL1_B0,
    NAU7802_OCAL2_B2,
    NAU7802_OCAL2_B1,
    NAU7802_OCAL2_B0,
    NAU7802_GCAL2_B3,
    NAU7802_GCAL2_B2,
    NAU7802_GCAL2_B1,
    NAU7802_GCAL2_B0,
    NAU7802_I2C_CONTROL,
    NAU7802_ADCO_B2,
    NAU7802_ADCO_B1,
    NAU7802_ADCO_B0,
    NAU7802_ADC = 0x15, // Shared ADC and OTP 32:24
    NAU7802_OTP_B1,     // OTP 23:16 or 7:0?
    NAU7802_OTP_B0,     // OTP 15:8
    NAU7802_PGA = 0x1B,
    NAU7802_PGA_PWR = 0x1C,
    NAU7802_DEVICE_REV = 0x1F,
} registers;

// these need to map to exactly the VLDO values for CTRL1.
typedef enum {
    NAU7802_LDO_45V = 0,
    NAU7802_LDO_42V = 1,
    NAU7802_LDO_39V = 2,
    NAU7802_LDO_36V = 3,
    NAU7802_LDO_33V = 4,
    NAU7802_LDO_30V = 5,
    NAU7802_LDO_27V = 6,
    NAU7802_LDO_24V = 7,
} nau7802_ldo_level;


// Lightweight device handle to replace i2c_master_dev_handle_t
typedef struct {
    i2c_port_t port;   // I2C controller port (e.g., I2C_NUM_0)
    uint8_t addr;      // 7-bit I2C address (ADR pin selects 0x2A or 0x2B)
    
// Optional scale state.
    float      calibration_factor; // default 1.0
    int32_t    zero_offset;        // default 0
    uint8_t    avg_samples;        // default 20
    uint32_t   timeout_ms;         // default 1000 (see begin)
    uint32_t   ldo_ramp_ms;        // default 250

} nau7802_handle_t;


// Probe the given I2C port for an NAU7802 (0x2A/0x2B); fill handle on success.
int nau7802_detect(i2c_port_t port, nau7802_handle_t* i2cnau);

// Issue a device reset to restart the NAU7802.
int nau7802_reset(const nau7802_handle_t* i2c);

// Power up analog/digital blocks, start conversions, and run internal offset calibration.
int nau7802_poweron(const nau7802_handle_t* i2c);

// Set PGA gain (power-of-two 1..128, or 0 for bypass) and re-calibrate.
int nau7802_set_gain(const nau7802_handle_t* i2c, unsigned gain);

// Set conversion rate (SPS: 10/20/40/80/320) and re-calibrate.
int nau7802_set_sample_rate(const nau7802_handle_t* i2c, unsigned rate);

// Disable internal LDO; use external AVDD pin for the analog rail.
int nau7802_disable_ldo(const nau7802_handle_t* i2c);

// Enable internal LDO at the requested VLDO level; set PGA LDOMODE and re-calibrate.
int nau7802_enable_ldo(const nau7802_handle_t* i2c, nau7802_ldo_level level,
                       bool pga_ldomode);

// Enable/disable PGA_CAP for single‑channel builds with Vin2P/Vin2N capacitor; re-calibrate.
int nau7802_set_pga_cap(const nau7802_handle_t* i2c, bool enabled);

// Read a single 24‑bit signed ADC sample if data is ready (non‑blocking).
int nau7802_read(const nau7802_handle_t* i2c, int32_t* val);

// Read ADC sample and return it scaled to the provided full‑scale units.
int nau7802_read_scaled(const nau7802_handle_t* i2c, float* val, uint32_t scale);

// Enable/disable internal temperature sensor reads instead of VIN.
int nau7802_set_therm(const nau7802_handle_t* i2c, bool enabled);

// disable or enable the bandgap chopper. it is enabled by default.
int nau7802_set_bandgap_chop(const nau7802_handle_t* i2c, bool enabled);

// Enter or exit deep power‑down (analog off) to save power.
int nau7802_set_deepsleep(const nau7802_handle_t* i2c, bool powerdown);

// You can pass true to export the clock being used, false to re-establish the default behavior of indicating data readiness.
int nau7802_export_clock(const nau7802_handle_t* i2c, bool clock);


// Channel select (false = Channel 1, true = Channel 2). Triggers calibration.
int nau7802_set_channel(const nau7802_handle_t* i2c, bool ch2);

// Data-ready check: sets *ready = true if PU_CTRL.CR is set.
int nau7802_available(const nau7802_handle_t* i2c, bool* ready);

// INT/CRDY polarity: high-active (default true) or low-active.
int nau7802_set_int_polarity(const nau7802_handle_t* i2c, bool high_active);

// powerUp()/powerDown().
int nau7802_power_up(const nau7802_handle_t* i2c);
int nau7802_power_down(const nau7802_handle_t* i2c);

// Revision code (DEVICE_REV & 0x0F)
int nau7802_get_revision(const nau7802_handle_t* i2c, uint8_t* rev_low4);

// Calibration modes and status
typedef enum {
    NAU7802_CALMOD_INTERNAL = 0, // internal offset
    NAU7802_CALMOD_OFFSET   = 2, // system offset
    NAU7802_CALMOD_GAIN     = 3  // system gain
} nau7802_cal_mode_t;

typedef enum {
    NAU7802_CAL_SUCCESS     = 0,
    NAU7802_CAL_IN_PROGRESS = 1,
    NAU7802_CAL_FAILURE     = 2
} nau7802_cal_status_t;

// Begin calibration (async), query status, wait, or do blocking calibration.
int nau7802_begin_calibrate(const nau7802_handle_t* i2c, nau7802_cal_mode_t mode);
int nau7802_calibration_status(const nau7802_handle_t* i2c, nau7802_cal_status_t* st);
int nau7802_wait_calibrate(const nau7802_handle_t* i2c, uint32_t timeout_ms);
int nau7802_calibrate(const nau7802_handle_t* i2c, nau7802_cal_mode_t mode, uint32_t timeout_ms);

// Raw register helpers
int nau7802_get_register(const nau7802_handle_t* i2c, uint8_t reg, uint8_t* val);
int nau7802_set_register(const nau7802_handle_t* i2c, uint8_t reg, uint8_t val);
int nau7802_get_24(const nau7802_handle_t* i2c, uint8_t reg_msb, int32_t* value);
int nau7802_set_24(const nau7802_handle_t* i2c, uint8_t reg_msb, int32_t value);
int nau7802_get_32(const nau7802_handle_t* i2c, uint8_t reg_msb, uint32_t* value);
int nau7802_set_32(const nau7802_handle_t* i2c, uint8_t reg_msb, uint32_t value);

// Bit helpers
int nau7802_set_bit(const nau7802_handle_t* i2c, uint8_t bitNumber, uint8_t reg);
int nau7802_clear_bit(const nau7802_handle_t* i2c, uint8_t bitNumber, uint8_t reg);
int nau7802_get_bit(const nau7802_handle_t* i2c, uint8_t bitNumber, uint8_t reg, bool* set);



// Stateless tare/cal/weight helpers (no change to handle struct)
int nau7802_calculate_zero_offset(const nau7802_handle_t* i2c,
                                  uint8_t average, uint32_t timeout_ms,
                                  int32_t* zero_out);
int nau7802_calculate_calibration_factor(const nau7802_handle_t* i2c,
                                         float known_weight,
                                         uint8_t average, uint32_t timeout_ms,
                                         int32_t zero_offset,
                                         float* cal_out);
int nau7802_get_weight(const nau7802_handle_t* i2c,
                       int32_t zero_offset, float cal_factor,
                       uint8_t samples, uint32_t timeout_ms,
                       bool allow_negative, float* weight_out);


// state getters / setters ---
int nau7802_set_calibration_factor(nau7802_handle_t* i2c, float cf);
int nau7802_get_calibration_factor(const nau7802_handle_t* i2c, float* cf);
int nau7802_set_channel1_offset(nau7802_handle_t* i2c, int32_t offset);
int nau7802_get_channel1_offset(const nau7802_handle_t* i2c, int32_t* offset);

// Channel 1 gain/offset helpers
int nau7802_get_channel1_gain(const nau7802_handle_t* i2c, uint32_t* gain);
int nau7802_set_channel1_gain(const nau7802_handle_t* i2c, uint32_t gain);




// --- Convenience reads 
int nau7802_get_reading(const nau7802_handle_t* i2c, int32_t* reading);
int nau7802_get_average(const nau7802_handle_t* i2c, uint8_t samples, uint32_t timeout_ms, int32_t* avg);


int nau7802_start(nau7802_handle_t* i2c, i2c_port_t port, uint8_t addr);

// --- LDO ramp delay control ---
int nau7802_set_ldo_ramp_delay(nau7802_handle_t* i2c, uint32_t delay_ms);

// --- I2C_CONTROL bit helpers (Advanced I2C parity) ---
int nau7802_set_crsd   (const nau7802_handle_t* i2c, bool enabled);
int nau7802_set_fdr    (const nau7802_handle_t* i2c, bool enabled);
int nau7802_set_wpd    (const nau7802_handle_t* i2c, bool enabled); // a.k.a. SPE/WPD
int nau7802_set_si     (const nau7802_handle_t* i2c, bool enabled);
int nau7802_set_bopga  (const nau7802_handle_t* i2c, bool enabled);
