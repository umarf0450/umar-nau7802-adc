
#include "umar_nau7802_adc.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#define TIMEOUT_MS   500 
#define TIMEOUT_TICKS pdMS_TO_TICKS(TIMEOUT_MS)
static const char* TAG = "nau";


// SparkFun-parity defaults
#define NAU7802_DEFAULT_CAL_FACTOR   1.0f
#define NAU7802_DEFAULT_ZERO_OFFSET  0
#define NAU7802_DEFAULT_AVG_SAMPLES  20
#define NAU7802_DEFAULT_TIMEOUT_MS   1000
#define NAU7802_DEFAULT_LDO_RAMP_MS  250


/* --- Legacy-driver helpers ------------------------------------------------ */
static int nau7802_xmit(const nau7802_handle_t* i2c, const void* buf, size_t blen)
{
    esp_err_t e = i2c_master_write_to_device(i2c->port, i2c->addr,
                                             (const uint8_t*)buf, blen, TIMEOUT_TICKS);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "error (%s) transmitting %zuB via I2C",
                 esp_err_to_name(e), blen);
        return -1;
    }
    return 0;
}
// get the single byte of some register
static inline esp_err_t
nau7802_readreg(const nau7802_handle_t* i2c, registers reg,
                const char* regname, uint8_t* val)
{
    uint8_t r = (uint8_t)reg;
    esp_err_t e = i2c_master_write_read_device(i2c->port, i2c->addr,
                                               &r, 1, val, 1, TIMEOUT_TICKS);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "error (%s) requesting %s via I2C", esp_err_to_name(e), regname);
        return e;
    }
    ESP_LOGD(TAG, "got %s: 0x%02x", regname, *val);
    return ESP_OK;
}
static inline esp_err_t
nau7802_pu_ctrl(const nau7802_handle_t* i2c, uint8_t* val)
{
    return nau7802_readreg(i2c, NAU7802_PU_CTRL, "PU_CTRL", val);
}
static inline esp_err_t
nau7802_ctrl1(const nau7802_handle_t* i2c, uint8_t* val)
{
    return nau7802_readreg(i2c, NAU7802_CTRL1, "CTRL1", val);
}
static inline esp_err_t
nau7802_ctrl2(const nau7802_handle_t* i2c, uint8_t* val)
{
    return nau7802_readreg(i2c, NAU7802_CTRL2, "CTRL2", val);
}
static inline esp_err_t
nau7802_pga(const nau7802_handle_t* i2c, uint8_t* val)
{
    return nau7802_readreg(i2c, NAU7802_PGA, "PGA", val);
}

/* --- Public API (legacy driver) ------------------------------------------ */
int nau7802_detect(i2c_port_t port, nau7802_handle_t* i2cnau)
{
    const uint8_t candidates[] = { 0x2A, 0x2B };
    uint8_t reg = NAU7802_DEVICE_REV, val = 0;
    for (size_t i = 0; i < sizeof(candidates); ++i) {
        esp_err_t e = i2c_master_write_read_device(port, candidates[i],
                                                   &reg, 1, &val, 1, TIMEOUT_TICKS);
        if (e == ESP_OK) {
            i2cnau->port = port;
            i2cnau->addr = candidates[i];
            ESP_LOGI(TAG, "successfully detected NAU7802 at 0x%02x", candidates[i]);
            return 0;
        } else {
            ESP_LOGD(TAG, "probe miss at 0x%02x (%s)", candidates[i], esp_err_to_name(e));
        }
    }
    ESP_LOGE(TAG, "error detecting NAU7802 on I2C port %d", port);
    return -1;
}
int nau7802_reset(const nau7802_handle_t* i2c)
{
    const uint8_t buf[] = { NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR };
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    ESP_LOGI(TAG, "reset NAU7802");
    return 0;
}
static int nau7802_internal_calibrate(const nau7802_handle_t* i2c)
{
    uint8_t r;
    if (nau7802_ctrl2(i2c, &r)) {
        return -1;
    }
    // queue the internal calibration: set CALS (bit 2), zero out CALMOD (bits 1:0)
    const uint8_t buf[] = {
        NAU7802_CTRL2,
        (uint8_t)(((r | 0x04) & 0xFC))
    };
    ESP_LOGI(TAG, "running internal offset calibration");
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    // wait until CALS clears
    do {
        if (nau7802_ctrl2(i2c, &r)) {
            return -1;
        }
    } while (r & 0x04);
    bool failed = (r & 0x08) != 0;
    ESP_LOGI(TAG, "completed internal offset calibration with%s error", failed ? "" : "out");
    return failed ? -1 : 0;
}
// the power on sequence is:
// * send a reset
// * set PUD and PUA in PU_CTRL
// * check for PUR bit in PU_CTRL after short delay
// * set CS in PU_CTRL
// * set 0x30 in ADC_CTRL (REG_CHPS)
// * run an internal offset calibration (CALS/CALMOD)
//
// we ought also "wait through six cycles of data conversion" (1.14), but are not yet doing so.
int nau7802_poweron(const nau7802_handle_t* i2c)
{
    uint8_t buf[] = {
        NAU7802_PU_CTRL,
        (uint8_t)(NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA)
    };
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    // the data sheet says we must allow up to 200 microseconds for the device to power up.
    // we'll allow it a full millisecond.
    vTaskDelay(pdMS_TO_TICKS(1));
    if (nau7802_pu_ctrl(i2c, &buf[1])) {
        return -1;
    }
    if (!(buf[1] & NAU7802_PU_CTRL_PUR)) {
        ESP_LOGE(TAG, "didn't see powered-on bit");
        return -1;
    }
    buf[1] |= NAU7802_PU_CTRL_CS;
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    // Set REG_CHPS (bits in ADC register): write back 0x30 to the ADC register
    buf[0] = NAU7802_ADC;
    if (nau7802_readreg(i2c, buf[0], "ADC", &buf[1])) {
        return -1;
    }
    buf[1] = 0x30;
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    // Read device revision
    uint8_t devrev_reg = NAU7802_DEVICE_REV, devrev_val = 0;
    esp_err_t e = i2c_master_write_read_device(i2c->port, i2c->addr,
                                               &devrev_reg, 1, &devrev_val, 1, TIMEOUT_TICKS);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "error %d reading device revision code", e);
        return -1;
    }
    devrev_val &= 0x0F;
    if (devrev_val != 0x0F) {
        ESP_LOGW(TAG, "unexpected revision id 0x%x", devrev_val);
    } else {
        ESP_LOGI(TAG, "device revision code: 0x%x", devrev_val);
    }
    if (nau7802_internal_calibrate(i2c)) {
        return -1;
    }
    return 0;
}
// FIXME also have to cut PGA <= 2
int nau7802_set_therm(const nau7802_handle_t* i2c, bool enabled)
{
    uint8_t buf[] = { NAU7802_I2C_CONTROL, 0x00 };
    if (nau7802_readreg(i2c, buf[0], "I2C_CONTROL", &buf[1])) {
        return -1;
    }
    if (enabled) {
        buf[1] |= 0x02; // set TS bit
    } else {
        buf[1] &= (uint8_t)~0x02; // clear TS bit
    }
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    ESP_LOGI(TAG, "set temperature sensor bit");
    // shouldn't need to calibrate here
    return 0;
}
int nau7802_set_bandgap_chop(const nau7802_handle_t* i2c, bool enabled)
{
    uint8_t buf[] = { NAU7802_I2C_CONTROL, 0x00 };
    if (nau7802_readreg(i2c, buf[0], "I2C_CONTROL", &buf[1])) {
        return -1;
    }
    // enabled => clear disable bit; disabled => set disable bit
    if (enabled) {
        buf[1] &= (uint8_t)~NAU7802_PGA_CHPDIS; // clear BGPCP (disable) bit
    } else {
        buf[1] |= NAU7802_PGA_CHPDIS; // set BGPCP (disable) bit
    }
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    ESP_LOGI(TAG, "set bandgap chopper bit");
    // shouldn't need to calibrate here
    return 0;
}
int nau7802_set_pga_cap(const nau7802_handle_t* i2c, bool enabled)
{
    uint8_t buf[] = { NAU7802_PGA_PWR, 0x00 };
    if (nau7802_readreg(i2c, buf[0], "PGA_PWR", &buf[1])) {
        return -1;
    }
    if (enabled) {
        buf[1] |= 0x80; // set PGA_CAP_EN
    } else {
        buf[1] &= 0x7F; // clear PGA_CAP_EN
    }
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    ESP_LOGI(TAG, "set pga cap bit");
    if (nau7802_internal_calibrate(i2c)) {
        return -1;
    }
    return 0;
}
static int nau7802_set_pgabypass(const nau7802_handle_t* i2c, bool bypass)
{
    uint8_t buf[] = { NAU7802_PGA, 0xFF };
    if (nau7802_pga(i2c, &buf[1])) {
        return -1;
    }
    if (bypass) {
        buf[1] |= NAU7802_PGA_BYPASS;
    } else {
        buf[1] &= (uint8_t)~NAU7802_PGA_BYPASS;
    }
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    return 0;
}
int nau7802_set_gain(const nau7802_handle_t* i2c, unsigned gain)
{
    uint8_t rbuf;
    // gain must be power of two (1..128) or 0 (bypass)
    if (gain > 128 || (gain != 0 && (gain & (gain - 1)))) {
        ESP_LOGE(TAG, "illegal gain value %u", gain);
        return -1;
    }
    if (gain == 0) {
        return nau7802_set_pgabypass(i2c, true);
    }
    // explicitly disable bypass before setting gain
    if (nau7802_set_pgabypass(i2c, false)) {
        return -1;
    }
    uint8_t buf[] = { NAU7802_CTRL1, 0xFF };
    if (nau7802_ctrl1(i2c, &buf[1])) {
        return -1;
    }
    // clear gain bits (2:0)
    buf[1] &= 0xF8;
    ESP_LOGI(TAG, "read ctrl1 0x%02x", buf[1]);
    // map gain to field (datasheet: bits 2..0). Keep your mapping logic:
    if (gain >= 16) {
        buf[1] |= 0x04;
    }
    if (gain > 32 || (gain < 16 && gain > 2)) {
        buf[1] |= 0x02;
    }
    if (gain == 128 || gain == 32 || gain == 8 || gain == 2) {
        buf[1] |= 0x01;
    }
    ESP_LOGI(TAG, "writing ctrl1 with 0x%02x", buf[1]);
    // write the new value then read back to confirm
    uint8_t rd = NAU7802_CTRL1;
    esp_err_t e = i2c_master_write_read_device(i2c->port, i2c->addr,
                                               buf, 2, &rbuf, 1, TIMEOUT_TICKS);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "error %s writing CTRL1", esp_err_to_name(e));
        return -1;
    }
    if (rbuf != buf[1]) {
        ESP_LOGE(TAG, "CTRL1 reply 0x%02x didn't match 0x%02x", rbuf, buf[1]);
        return -1;
    }
    ESP_LOGI(TAG, "set gain");
    if (nau7802_internal_calibrate(i2c)) {
        return -1;
    }
    return 0;
}
int nau7802_set_sample_rate(const nau7802_handle_t* i2c, unsigned rate)
{
    uint8_t rbuf;
    if (rate != 10 && rate != 20 && rate != 40 && rate != 80 && rate != 320) {
        ESP_LOGE(TAG, "illegal rate value %u", rate);
        return -1;
    }
    uint8_t buf[] = { NAU7802_CTRL2, 0xFF };
    if (nau7802_ctrl2(i2c, &buf[1])) {
        return -1;
    }
    // Clear FS bits (CTRL2 bits 6:4)
    buf[1] &= (uint8_t)~0x70;
    if (rate == 10) { buf[1] |= (0b000 << 4); }
    else if (rate == 20) { buf[1] |= (0b001 << 4); }
    else if (rate == 40) { buf[1] |= (0b010 << 4); }
    else if (rate == 80) { buf[1] |= (0b011 << 4); }
    else if (rate == 320){ buf[1] |= (0b111 << 4); }
    ESP_LOGI(TAG, "writing ctrl2 with 0x%02x", buf[1]);
    // write the new value then read back to confirm
    uint8_t rd = NAU7802_CTRL2;
    esp_err_t e = i2c_master_write_read_device(i2c->port, i2c->addr,
                                               buf, 2, &rbuf, 1, TIMEOUT_TICKS);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "error %s writing CTRL2", esp_err_to_name(e));
        return -1;
    }
    if (rbuf != buf[1]) {
        ESP_LOGE(TAG, "CTRL2 reply 0x%02x didn't match 0x%02x", rbuf, buf[1]);
        return -1;
    }
    ESP_LOGI(TAG, "set rate");
    if (nau7802_internal_calibrate(i2c)) {
        return -1;
    }
    return 0;
}
// set the PGA LDOMODE (*not* the master AVDDS/LDO switch)
static int nau7802_set_pgaldomode(const nau7802_handle_t* i2c, bool ldomode)
{
    uint8_t buf[] = { NAU7802_PGA, 0xFF };
    if (nau7802_pga(i2c, &buf[1])) {
        return -1;
    }
    if (ldomode) {
        buf[1] |= NAU7802_PGA_LDOMODE;
    } else {
        buf[1] &= (uint8_t)~NAU7802_PGA_LDOMODE;
    }
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    return 0;
}
// select whether AVDD is sourced from the internal LDO, or the AVDD pin input (default configuration).
// false for AVDD pin.
static int nau7802_set_ldo(const nau7802_handle_t* i2c, bool ldomode)
{
    uint8_t buf[] = { NAU7802_PU_CTRL, 0xFF };
    if (nau7802_pu_ctrl(i2c, &buf[1])) {
        return -1;
    }
    if (ldomode) {
        buf[1] |= NAU7802_PU_CTRL_AVDDS;
    } else {
        buf[1] &= (uint8_t)~NAU7802_PU_CTRL_AVDDS;
    }
    if (nau7802_xmit(i2c, buf, sizeof(buf))) {
        return -1;
    }
    return 0;
}
int nau7802_disable_ldo(const nau7802_handle_t* i2c)
{
    if (nau7802_set_ldo(i2c, false)) {
        return -1;
    }
    ESP_LOGI(TAG, "enabled avdd pin input");
    if (nau7802_internal_calibrate(i2c)) {
        return -1;
    }
    return 0;
}
int nau7802_enable_ldo(const nau7802_handle_t* i2c, nau7802_ldo_level mode, bool pga_ldomode)
{
    if (mode > NAU7802_LDO_24V || mode < NAU7802_LDO_45V) {
        ESP_LOGW(TAG, "illegal LDO mode %d", mode);
        return -1;
    }
    uint8_t buf[] = { NAU7802_CTRL1, 0xFF };
    uint8_t rbuf;
    if (nau7802_ctrl1(i2c, &rbuf)) {
        return -1;
    }
    // VLDO is bits 5..3
    buf[1] = (uint8_t)(rbuf & 0xC7);
    buf[1] |= (uint8_t)(mode << 3);
    ESP_LOGI(TAG, "requesting VLDO mode 0x%02x (0x%02x -> 0x%02x)",
             (rbuf & 0xC7), (mode << 3), buf[1]);
    // write then read back to confirm
    esp_err_t e = i2c_master_write_read_device(i2c->port, i2c->addr,
                                               buf, 2, &rbuf, 1, TIMEOUT_TICKS);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "error (%s) writing CTRL1", esp_err_to_name(e));
        return -1;
    }
    if (rbuf != buf[1]) {
        ESP_LOGE(TAG, "CTRL1 reply 0x%02x didn't match 0x%02x", rbuf, buf[1]);
        return -1;
    }
    if (nau7802_set_pgaldomode(i2c, pga_ldomode)) {
        return -1;
    }
    if (nau7802_set_ldo(i2c, true)) {
        return -1;
    }
  


    ESP_LOGI(TAG, "enabled internal ldo");

// LDO ramp delay (SparkFun v1.0.5 defaults to ~250 ms)
    uint32_t ramp = i2c->ldo_ramp_ms ? i2c->ldo_ramp_ms : 250;
    vTaskDelay(pdMS_TO_TICKS(ramp));

    if (nau7802_internal_calibrate(i2c)) {
    return -1;
    }   






    return 0;
}
int nau7802_read_scaled(const nau7802_handle_t* i2c, float* val, uint32_t scale)
{
    int32_t v;
    if (nau7802_read(i2c, &v)) {
        return -1;
    }
    const float ADCMAX = (float)(1u << 23u); // can be represented perfectly in 32-bit float
    const float adcper = ADCMAX / scale;
    *val = v / adcper;
    ESP_LOGD(TAG, "converted raw %ld to %f", (long)v, *val);
    return 0;
}
static esp_err_t
nau7802_read_internal(const nau7802_handle_t* i2c, int32_t* val, bool lognodata)
{
    uint8_t r0, r1, r2;
    esp_err_t e;
    if ((e = nau7802_pu_ctrl(i2c, &r0)) != ESP_OK) {
        return e;
    }
    if (!(r0 & NAU7802_PU_CTRL_CR)) {
        if (lognodata) {
            ESP_LOGE(TAG, "data not yet ready at ADC (0x%02x)", r0);
        }
        return ESP_ERR_NOT_FINISHED;
    }
    if ((e = nau7802_readreg(i2c, NAU7802_ADCO_B2, "ADCO_B2", &r2)) != ESP_OK) {
        return e;
    }
    if ((e = nau7802_readreg(i2c, NAU7802_ADCO_B1, "ADCO_B1", &r1)) != ESP_OK) {
        return e;
    }
    if ((e = nau7802_readreg(i2c, NAU7802_ADCO_B0, "ADCO_B0", &r0)) != ESP_OK) {
        return e;
    }
    // FIXME: chop to noise_free_bits according to AVDD and PGA. We never have more than 20 NF bits nor less than 16.
    const int32_t mask = 0xF0;
    *val = ((int32_t)r2 << 16)
         | ((int32_t)r1 << 8)
         | (int32_t)(r0 & mask);
    // Sign-extend if MSB of 24-bit value is set
    if (*val & 0x800000) {
        *val |= 0xFF000000;
    }
    ESP_LOGD(TAG, "ADC reads: %u %u %u full %" PRId32, r0, r1, r2, *val);
    return ESP_OK;
}
int nau7802_read(const nau7802_handle_t* i2c, int32_t* val)
{
    return (nau7802_read_internal(i2c, val, true) == ESP_OK) ? 0 : -1;
}
// (optional) helper that wasn’t in the header; keeping it here for your usage
esp_err_t nau7802_multisample(const nau7802_handle_t* i2c, float* val, unsigned n)
{
    float sum = 0;
    for (unsigned z = 0; z < n; ++z) {
        int32_t v;
        esp_err_t e;
        do {
            e = nau7802_read_internal(i2c, &v, false);
            if (e != ESP_OK && e != ESP_ERR_NOT_FINISHED) {
                return e;
            }
        } while (e != ESP_OK);
        // accumulating into 32-bit float can result in inaccurate maths if we go beyond 1<<24u
        sum += (float)v;
    }
    *val = sum / n;
    return ESP_OK;
}
int nau7802_set_deepsleep(const nau7802_handle_t* i2c, bool powerdown)
{
    uint8_t buf[] = { NAU7802_PU_CTRL, 0xFF };
    if (nau7802_pu_ctrl(i2c, &buf[1])) {
        return -1;
    }
    const uint8_t mask = (uint8_t)(NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
    if (powerdown) {
        if (buf[1] & mask) {
            buf[1] &= (uint8_t)~mask;
            if (nau7802_xmit(i2c, buf, sizeof(buf))) {
                return -1;
            }
            // we ought also "wait through six cycles of data conversion" (1.14), but are not yet doing so.
        } else {
            ESP_LOGE(TAG, "analog is already powered down");
        }
    } else {
        if ((buf[1] & mask) != mask) {
            buf[1] |= mask;
            if (nau7802_xmit(i2c, buf, sizeof(buf))) {
                return -1;
            }
        } else {
            ESP_LOGE(TAG, "analog is already powered up");
        }
    }
    return 0;
}

/* ==========================================================================
 *      ADDITIONAL API (SparkFun-compat style) — Implementations
 * ==========================================================================*/

int nau7802_set_channel(const nau7802_handle_t* i2c, bool ch2)
{
    uint8_t v;
    if (nau7802_ctrl2(i2c, &v)) return -1;
    if (ch2) v |= 0x80; else v &= (uint8_t)~0x80; // CHS bit (bit 7)
    uint8_t buf[2] = { NAU7802_CTRL2, v };
    if (nau7802_xmit(i2c, buf, sizeof buf)) return -1;
    // Per SparkFun guidance: re-calibrate after channel changes
    return nau7802_internal_calibrate(i2c);
}

int nau7802_available(const nau7802_handle_t* i2c, bool* ready)
{
    if (!ready) return -1;
    uint8_t v;
    if (nau7802_pu_ctrl(i2c, &v)) return -1;
    *ready = (v & NAU7802_PU_CTRL_CR) != 0;
    return 0;
}

int nau7802_set_int_polarity(const nau7802_handle_t* i2c, bool high_active)
{
    uint8_t v;
    if (nau7802_ctrl1(i2c, &v)) return -1;
    if (high_active) {
        v &= (uint8_t)~0x80; // clear CRP => high-active
    } else {
        v |= 0x80;           // set CRP   => low-active
    }
    uint8_t buf[2] = { NAU7802_CTRL1, v };
    return nau7802_xmit(i2c, buf, sizeof buf);
}

int nau7802_power_up(const nau7802_handle_t* i2c)
{
    uint8_t v;
    // set PUD and PUA
    if (nau7802_pu_ctrl(i2c, &v)) return -1;
    v |= (NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
    uint8_t buf[2] = { NAU7802_PU_CTRL, v };
    if (nau7802_xmit(i2c, buf, sizeof buf)) return -1;

    // wait for PUR (up to ~100ms)
    uint32_t ticks0 = xTaskGetTickCount();
    do {
        if (nau7802_pu_ctrl(i2c, &v)) return -1;
        if (v & NAU7802_PU_CTRL_PUR) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((xTaskGetTickCount() - ticks0) < pdMS_TO_TICKS(100));
    if (!(v & NAU7802_PU_CTRL_PUR)) {
        ESP_LOGE(TAG, "power_up timeout waiting PUR");
        return -1;
    }

    // set CS
    v |= NAU7802_PU_CTRL_CS;
    buf[1] = v;
    if (nau7802_xmit(i2c, buf, sizeof buf)) return -1;
    return 0;
}

int nau7802_power_down(const nau7802_handle_t* i2c)
{
    uint8_t v;
    if (nau7802_pu_ctrl(i2c, &v)) return -1;
    v &= (uint8_t)~(NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
    uint8_t buf[2] = { NAU7802_PU_CTRL, v };
    return nau7802_xmit(i2c, buf, sizeof buf);
}

int nau7802_get_revision(const nau7802_handle_t* i2c, uint8_t* rev_low4)
{
    if (!rev_low4) return -1;
    uint8_t v;
    if (nau7802_readreg(i2c, NAU7802_DEVICE_REV, "DEVICE_REV", &v)) return -1;
    *rev_low4 = (uint8_t)(v & 0x0F);
    return 0;
}

/* --- Calibration control -------------------------------------------------- */
int nau7802_begin_calibrate(const nau7802_handle_t* i2c, nau7802_cal_mode_t mode)
{
    uint8_t v;
    if (nau7802_ctrl2(i2c, &v)) return -1;
    v &= (uint8_t)~0x03;                 // clear CALMOD[1:0]
    v |= (uint8_t)(mode & 0x03);         // set mode
    v |= 0x04;                           // set CALS
    uint8_t buf[2] = { NAU7802_CTRL2, v };
    return nau7802_xmit(i2c, buf, sizeof buf);
}

int nau7802_calibration_status(const nau7802_handle_t* i2c, nau7802_cal_status_t* st)
{
    if (!st) return -1;
    uint8_t v;
    if (nau7802_ctrl2(i2c, &v)) return -1;
    if (v & 0x04) { *st = NAU7802_CAL_IN_PROGRESS; return 0; } // CALS
    if (v & 0x08) { *st = NAU7802_CAL_FAILURE;     return 0; } // CAL_ERROR
    *st = NAU7802_CAL_SUCCESS;
    return 0;
}

int nau7802_wait_calibrate(const nau7802_handle_t* i2c, uint32_t timeout_ms)
{
    uint32_t t0 = xTaskGetTickCount();
    for (;;) {
        nau7802_cal_status_t s;
        if (nau7802_calibration_status(i2c, &s)) return -1;
        if (s != NAU7802_CAL_IN_PROGRESS) {
            return (s == NAU7802_CAL_SUCCESS) ? 0 : -1;
        }
        if (timeout_ms && (xTaskGetTickCount() - t0) > pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "wait_calibrate timeout");
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int nau7802_calibrate(const nau7802_handle_t* i2c, nau7802_cal_mode_t mode, uint32_t timeout_ms)
{
    if (nau7802_begin_calibrate(i2c, mode)) return -1;
    return nau7802_wait_calibrate(i2c, timeout_ms);
}

/* --- Register helpers ----------------------------------------------------- */
int nau7802_get_register(const nau7802_handle_t* i2c, uint8_t reg, uint8_t* val)
{
    if (!val) return -1;
    return (nau7802_readreg(i2c, (registers)reg, "GEN", val) == ESP_OK) ? 0 : -1;
}

int nau7802_set_register(const nau7802_handle_t* i2c, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return nau7802_xmit(i2c, buf, sizeof buf);
}

int nau7802_get_24(const nau7802_handle_t* i2c, uint8_t reg_msb, int32_t* value)
{
    if (!value) return -1;
    uint8_t b2, b1, b0;
    if (nau7802_readreg(i2c, (registers)reg_msb,     "24_B2", &b2)) return -1;
    if (nau7802_readreg(i2c, (registers)(reg_msb+1), "24_B1", &b1)) return -1;
    if (nau7802_readreg(i2c, (registers)(reg_msb+2), "24_B0", &b0)) return -1;
    int32_t v = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | (int32_t)b0;
    if (v & 0x00800000) v |= 0xFF000000; // sign-extend 24-bit
    *value = v;
    return 0;
}

int nau7802_set_24(const nau7802_handle_t* i2c, uint8_t reg_msb, int32_t value)
{
    uint32_t u = (uint32_t)value;
    uint8_t buf[4] = {
        reg_msb,
        (uint8_t)((u >> 16) & 0xFF),
        (uint8_t)((u >>  8) & 0xFF),
        (uint8_t)( u        & 0xFF)
    };
    return nau7802_xmit(i2c, buf, sizeof buf);
}

int nau7802_get_32(const nau7802_handle_t* i2c, uint8_t reg_msb, uint32_t* value)
{
    if (!value) return -1;
    uint8_t b3,b2,b1,b0;
    if (nau7802_readreg(i2c, (registers)reg_msb,     "32_B3", &b3)) return -1;
    if (nau7802_readreg(i2c, (registers)(reg_msb+1), "32_B2", &b2)) return -1;
    if (nau7802_readreg(i2c, (registers)(reg_msb+2), "32_B1", &b1)) return -1;
    if (nau7802_readreg(i2c, (registers)(reg_msb+3), "32_B0", &b0)) return -1;
    *value = ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
    return 0;
}

int nau7802_set_32(const nau7802_handle_t* i2c, uint8_t reg_msb, uint32_t value)
{
    uint8_t buf[5] = {
        reg_msb,
        (uint8_t)((value >> 24) & 0xFF),
        (uint8_t)((value >> 16) & 0xFF),
        (uint8_t)((value >>  8) & 0xFF),
        (uint8_t)( value        & 0xFF)
    };
    return nau7802_xmit(i2c, buf, sizeof buf);
}

/* --- Bit helpers ---------------------------------------------------------- */
int nau7802_set_bit(const nau7802_handle_t* i2c, uint8_t bitNumber, uint8_t reg)
{
    uint8_t v;
    if (nau7802_get_register(i2c, reg, &v)) return -1;
    v |= (uint8_t)(1u << bitNumber);
    return nau7802_set_register(i2c, reg, v);
}
int nau7802_clear_bit(const nau7802_handle_t* i2c, uint8_t bitNumber, uint8_t reg)
{
    uint8_t v;
    if (nau7802_get_register(i2c, reg, &v)) return -1;
    v &= (uint8_t)~(1u << bitNumber);
    return nau7802_set_register(i2c, reg, v);
}
int nau7802_get_bit(const nau7802_handle_t* i2c, uint8_t bitNumber, uint8_t reg, bool* set)
{
    if (!set) return -1;
    uint8_t v;
    if (nau7802_get_register(i2c, reg, &v)) return -1;
    *set = (v & (1u << bitNumber)) != 0;
    return 0;
}

/* --- Stateless tare/calibration/weight helpers --------------------------- */
int nau7802_calculate_zero_offset(const nau7802_handle_t* i2c,
                                  uint8_t average, uint32_t timeout_ms,
                                  int32_t* zero_out)
{
    if (!zero_out) return -1;
    uint32_t t0 = xTaskGetTickCount();
    float avg;
    for (;;) {
        if (nau7802_multisample(i2c, &avg, average) == ESP_OK) break;
        if (timeout_ms && (xTaskGetTickCount() - t0) > pdMS_TO_TICKS(timeout_ms)) return -1;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    *zero_out = (int32_t)avg;
    return 0;
}

int nau7802_calculate_calibration_factor(const nau7802_handle_t* i2c,
                                         float known_weight,
                                         uint8_t average, uint32_t timeout_ms,
                                         int32_t zero_offset,
                                         float* cal_out)
{
    if (!cal_out || known_weight == 0.0f) return -1;
    int32_t onScale;
    {
        uint32_t t0 = xTaskGetTickCount();
        float avg;
        for (;;) {
            if (nau7802_multisample(i2c, &avg, average) == ESP_OK) {
                onScale = (int32_t)avg; break;
            }
            if (timeout_ms && (xTaskGetTickCount() - t0) > pdMS_TO_TICKS(timeout_ms)) return -1;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    *cal_out = (float)(onScale - zero_offset) / known_weight;
    return 0;
}

int nau7802_get_weight(const nau7802_handle_t* i2c,
                       int32_t zero_offset, float cal_factor,
                       uint8_t samples, uint32_t timeout_ms,
                       bool allow_negative, float* weight_out)
{
    if (!weight_out || cal_factor == 0.0f) return -1;
    int32_t onScale;
    {
        uint32_t t0 = xTaskGetTickCount();
        float avg;
        for (;;) {
            if (nau7802_multisample(i2c, &avg, samples) == ESP_OK) {
                onScale = (int32_t)avg; break;
            }
            if (timeout_ms && (xTaskGetTickCount() - t0) > pdMS_TO_TICKS(timeout_ms)) return -1;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    if (!allow_negative && onScale < zero_offset) onScale = zero_offset;
      *weight_out = (float)(onScale - zero_offset) / cal_factor;
    return 0;
}







/* --- Channel 1 offset/gain helpers --------------------------------------- */

int nau7802_set_channel1_offset(nau7802_handle_t* i2c, int32_t offset) {
    if (!i2c) return -1;
    i2c->zero_offset = offset;
    // Also write the device OCAL1 register (SparkFun Example7 behavior)
    return nau7802_set_24(i2c, NAU7802_OCAL1_B2, offset);
}
int nau7802_get_channel1_offset(const nau7802_handle_t* i2c, int32_t* offset) {
    if (!i2c || !offset) return -1;
    *offset = i2c->zero_offset;
    return 0;
}

int nau7802_get_channel1_gain(const nau7802_handle_t* i2c, uint32_t* gain)
{
    return nau7802_get_32(i2c, NAU7802_GCAL1_B3, gain);
}
int nau7802_set_channel1_gain(const nau7802_handle_t* i2c, uint32_t gain)
{
    return nau7802_set_32(i2c, NAU7802_GCAL1_B3, gain);
}


int nau7802_set_calibration_factor(nau7802_handle_t* i2c, float cf) {
    if (!i2c || cf == 0.0f) return -1;
    i2c->calibration_factor = cf;
    return 0;
}
int nau7802_get_calibration_factor(const nau7802_handle_t* i2c, float* cf) {
    if (!i2c || !cf) return -1;
    *cf = (i2c->calibration_factor == 0.0f) ? NAU7802_DEFAULT_CAL_FACTOR : i2c->calibration_factor;
    return 0;
}









int nau7802_get_reading(const nau7802_handle_t* i2c, int32_t* reading) {
    if (!i2c || !reading) return -1;
    bool ready = false;
    if (nau7802_available(i2c, &ready) || !ready) return -1;
    return nau7802_read(i2c, reading);
}

int nau7802_get_average(const nau7802_handle_t* i2c, uint8_t samples, uint32_t timeout_ms, int32_t* avg_out) {
    if (!i2c || !avg_out || samples == 0) return -1;
    if (timeout_ms == 0) timeout_ms = NAU7802_DEFAULT_TIMEOUT_MS;
    uint32_t t0 = xTaskGetTickCount();
    double sum = 0.0;
    uint8_t got = 0;
    while (got < samples) {
        bool ready=false;
        if (nau7802_available(i2c, &ready)) return -1;
        if (ready) {
            int32_t v;
            if (nau7802_read(i2c, &v)) return -1;
            sum += (double)v;
            got++;
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (timeout_ms && (xTaskGetTickCount() - t0) > pdMS_TO_TICKS(timeout_ms)) return -1;
    }
    *avg_out = (int32_t)(sum / samples);
    return 0;
}




int nau7802_set_ldo_ramp_delay(nau7802_handle_t* i2c, uint32_t delay_ms) {
    if (!i2c) return -1;
    i2c->ldo_ramp_ms = delay_ms;
    return 0;
}






int nau7802_start(nau7802_handle_t* i2c, i2c_port_t port, uint8_t addr)
{
    if (!i2c) return -1;

    // Set handle (default to 0x2A if addr == 0)
    i2c->port = port;
    i2c->addr = addr ? addr : 0x2A;

    // Light presence check via device revision
    uint8_t rev;
    if (nau7802_get_revision(i2c, &rev) != 0) return -1;

    // Power-on (already performs internal offset calibration)
    if (nau7802_poweron(i2c) != 0) return -1;

    // ---- LDO configuration (internal AVDD source) ----
    // For DVDD = 3.3 V, NAU7802_LDO_30V is valid (LDO ≤ DVDD - 0.3 V).
    // pga_ldomode=false => higher DC gain (expects low-ESR AVDD cap).
    if (nau7802_enable_ldo(i2c, NAU7802_LDO_30V, false) != 0) return -1;

    // Optional: brief settle time for the analog rail
    vTaskDelay(pdMS_TO_TICKS(2));

    // Defaults like SparkFun examples: 40 SPS, gain 16
    if (nau7802_set_sample_rate(i2c, 80) != 0) return -1;
    if (nau7802_set_gain(i2c, 128) != 0) return -1;

    // No need for an extra internal calibration: the driver already does it
    // after poweron/ldo/gain/rate changes.

    return 0;
}






// I2C_CONTROL masks (see datasheet register summary)
#define NAU7802_I2C_CRSD     (1u << 7)
#define NAU7802_I2C_FDR      (1u << 6)
#define NAU7802_I2C_WPD      (1u << 5) // a.k.a SPE/WPD
#define NAU7802_I2C_SI       (1u << 4)
#define NAU7802_I2C_BOPGA    (1u << 3)
// bit0 TS/BGPCP is already handled in your nau7802_set_therm / set_bandgap_chop

static int i2c_ctrl_update(const nau7802_handle_t* i2c, uint8_t mask, bool set) {
    uint8_t v;
    if (nau7802_readreg(i2c, NAU7802_I2C_CONTROL, "I2C_CONTROL", &v)) return -1;
    if (set) v |= mask; else v &= (uint8_t)~mask;
    uint8_t buf[2] = { NAU7802_I2C_CONTROL, v };
    return nau7802_xmit(i2c, buf, sizeof buf);
}

int nau7802_set_crsd  (const nau7802_handle_t* i2c, bool en){ return i2c_ctrl_update(i2c, NAU7802_I2C_CRSD,  en); }
int nau7802_set_fdr   (const nau7802_handle_t* i2c, bool en){ return i2c_ctrl_update(i2c, NAU7802_I2C_FDR,   en); }
int nau7802_set_wpd   (const nau7802_handle_t* i2c, bool en){ return i2c_ctrl_update(i2c, NAU7802_I2C_WPD,   en); }
int nau7802_set_si    (const nau7802_handle_t* i2c, bool en){ return i2c_ctrl_update(i2c, NAU7802_I2C_SI,    en); }
int nau7802_set_bopga (const nau7802_handle_t* i2c, bool en){ return i2c_ctrl_update(i2c, NAU7802_I2C_BOPGA, en); }
