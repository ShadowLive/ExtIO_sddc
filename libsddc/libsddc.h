/*
 * libsddc - low level functions for wideband SDR receivers like
 *           BBRF103, RX-666, RX888, HF103, etc
 *
 * Copyright (C) 2020 by Franco Venturi
 *
 * this program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * this program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef __LIBSDDC_H
#define __LIBSDDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct sddc sddc_t;

struct sddc_device_info {
  const char *manufacturer;
  const char *product;
  const char *serial_number;
};

enum SDDCStatus {
  SDDC_STATUS_OFF,
  SDDC_STATUS_READY,
  SDDC_STATUS_STREAMING,
  SDDC_STATUS_FAILED = 0xff
};

enum SDDCHWModel {
  HW_NORADIO,
  HW_BBRF103,
  HW_HF103,
  HW_RX888,
  HW_RX888R2,
  HW_RX999,
  HW_RX888R3,
};

enum RFMode {
  NO_RF_MODE,
  HF_MODE,
  VHF_MODE
};

enum LEDColors {
  YELLOW_LED = 0x01,
  RED_LED    = 0x02,
  BLUE_LED   = 0x04
};

/** @defgroup basic Basic Functions
 *  Device enumeration, opening, and status functions
 *  @{
 */

/**
 * @brief Get the number of connected SDDC devices
 *
 * Scans the USB bus for compatible FX3-based SDR devices (BBRF103, HF103,
 * RX888, RX888R2, RX888R3, RX999).
 *
 * @return Number of devices found, or negative error code on failure
 *
 * @note This function initializes libusb internally
 * @note Thread-safe: Yes
 *
 * @see sddc_get_device_info()
 */
int sddc_get_device_count();

/**
 * @brief Get detailed information about all connected devices
 *
 * Retrieves manufacturer, product name, and serial number for each device.
 * The caller must free the returned array using sddc_free_device_info().
 *
 * @param[out] sddc_device_infos Pointer to receive allocated device info array
 *                               (NULL-terminated)
 *
 * @return Number of devices found, or negative error code on failure:
 *         -1: Invalid parameter (sddc_device_infos is NULL)
 *         -2: Failed to enumerate USB devices
 *
 * @note Memory is allocated by this function and must be freed with
 *       sddc_free_device_info()
 * @note Thread-safe: Yes
 * @note The returned array is NULL-terminated for iteration convenience
 *
 * @code
 * struct sddc_device_info *infos;
 * int count = sddc_get_device_info(&infos);
 * if (count > 0) {
 *     for (int i = 0; i < count; i++) {
 *         printf("Device %d: %s %s (SN: %s)\n", i,
 *                infos[i].manufacturer,
 *                infos[i].product,
 *                infos[i].serial_number);
 *     }
 *     sddc_free_device_info(infos);
 * }
 * @endcode
 *
 * @see sddc_get_device_count()
 * @see sddc_free_device_info()
 */
int sddc_get_device_info(struct sddc_device_info **sddc_device_infos);

/**
 * @brief Free device info array allocated by sddc_get_device_info()
 *
 * @param sddc_device_infos Device info array to free (may be NULL)
 *
 * @return 0 on success
 *
 * @note Thread-safe: Yes
 * @note Safe to call with NULL pointer
 *
 * @see sddc_get_device_info()
 */
int sddc_free_device_info(struct sddc_device_info *sddc_device_infos);

/**
 * @brief Open an SDDC device by index
 *
 * Opens the specified device and loads firmware if needed. The device
 * must be closed with sddc_close() when done.
 *
 * @param index Device index (0-based, from sddc_get_device_count())
 * @param imagefile Path to FX3 firmware image file (usually "SDDC_FX3.img")
 *
 * @return Device handle on success, NULL on failure
 *
 * @note This function may take several seconds if firmware loading is required
 * @note Use sddc_get_last_error() to get detailed error information
 * @note Thread-safe: Yes (different devices)
 * @note Only one handle should be open per physical device
 *
 * @see sddc_close()
 * @see sddc_get_last_error()
 */
sddc_t *sddc_open(int index, const char* imagefile);

/**
 * @brief Close an SDDC device
 *
 * Stops streaming if active and releases all resources associated with
 * the device handle.
 *
 * @param t Device handle (may be NULL)
 *
 * @note Thread-safe: No (don't call from multiple threads on same handle)
 * @note Safe to call with NULL pointer
 * @note Automatically stops streaming if active
 *
 * @see sddc_open()
 */
void sddc_close(sddc_t *t);

/**
 * @brief Get current device status
 *
 * @param t Device handle
 *
 * @return Current status:
 *         - SDDC_STATUS_OFF: Device not initialized
 *         - SDDC_STATUS_READY: Device ready, not streaming
 *         - SDDC_STATUS_STREAMING: Device actively streaming
 *         - SDDC_STATUS_FAILED: Device in error state
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_start_streaming()
 * @see sddc_stop_streaming()
 */
enum SDDCStatus sddc_get_status(sddc_t *t);

/**
 * @brief Get hardware model
 *
 * @param t Device handle
 *
 * @return Hardware model enum (HW_BBRF103, HW_HF103, HW_RX888, etc.)
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_get_hw_model_name()
 */
enum SDDCHWModel sddc_get_hw_model(sddc_t *t);

/**
 * @brief Get hardware model name as string
 *
 * @param t Device handle
 *
 * @return Model name string ("BBRF103", "HF103", "RX888", etc.)
 *
 * @note Thread-safe: Yes (read-only)
 * @note Returned string is static, do not free
 *
 * @see sddc_get_hw_model()
 */
const char *sddc_get_hw_model_name(sddc_t *t);

/**
 * @brief Get firmware version
 *
 * @param t Device handle
 *
 * @return Firmware version number
 *
 * @note Thread-safe: Yes (read-only)
 */
uint16_t sddc_get_firmware(sddc_t *t);

/**
 * @brief Get supported frequency range
 *
 * @param t Device handle
 *
 * @return Pointer to two-element array [min_freq_hz, max_freq_hz]
 *
 * @note Thread-safe: Yes (read-only)
 * @note Returned array is static, do not free
 * @note Range depends on hardware model and RF mode
 */
const double *sddc_get_frequency_range(sddc_t *t);

/**
 * @brief Get current RF mode
 *
 * @param t Device handle
 *
 * @return Current RF mode:
 *         - NO_RF_MODE: No RF frontend
 *         - HF_MODE: HF direct sampling mode (0-32 MHz)
 *         - VHF_MODE: VHF tuner mode (24 MHz - 1.8 GHz)
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_rf_mode()
 */
enum RFMode sddc_get_rf_mode(sddc_t *t);

/**
 * @brief Set RF mode
 *
 * Switches between HF direct sampling and VHF tuner modes.
 *
 * @param t Device handle
 * @param rf_mode Desired RF mode (HF_MODE or VHF_MODE)
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Must not be called while streaming
 * @note Switching modes resets tuner/attenuator settings
 * @note Thread-safe: No
 *
 * @see sddc_get_rf_mode()
 */
int sddc_set_rf_mode(sddc_t *t, enum RFMode rf_mode);

/** @} */ /* end of basic group */


/** @defgroup led LED Functions
 *  Control status LEDs on the device
 *  @{
 */

/**
 * @brief Turn on LED(s)
 *
 * @param t Device handle
 * @param led_pattern Bitmask of LEDs to turn on (YELLOW_LED, RED_LED, BLUE_LED)
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Multiple LEDs can be combined with bitwise OR
 * @note Thread-safe: No
 *
 * @code
 * sddc_led_on(dev, YELLOW_LED | BLUE_LED);  // Turn on yellow and blue
 * @endcode
 *
 * @see sddc_led_off()
 * @see sddc_led_toggle()
 */
int sddc_led_on(sddc_t *t, uint8_t led_pattern);

/**
 * @brief Turn off LED(s)
 *
 * @param t Device handle
 * @param led_pattern Bitmask of LEDs to turn off
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 *
 * @see sddc_led_on()
 * @see sddc_led_toggle()
 */
int sddc_led_off(sddc_t *t, uint8_t led_pattern);

/**
 * @brief Toggle LED(s)
 *
 * @param t Device handle
 * @param led_pattern Bitmask of LEDs to toggle
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 *
 * @see sddc_led_on()
 * @see sddc_led_off()
 */
int sddc_led_toggle(sddc_t *t, uint8_t led_pattern);

/** @} */ /* end of led group */


/** @defgroup adc ADC Functions
 *  Configure ADC dithering and randomization
 *  @{
 */

/**
 * @brief Get ADC dither state
 *
 * Dithering adds small random noise to improve ADC linearity and reduce
 * quantization artifacts.
 *
 * @param t Device handle
 *
 * @return 1 if dither enabled, 0 if disabled, negative on error
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_adc_dither()
 */
int sddc_get_adc_dither(sddc_t *t);

/**
 * @brief Enable/disable ADC dither
 *
 * @param t Device handle
 * @param dither 1 to enable, 0 to disable
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 * @note Can be changed during streaming
 *
 * @see sddc_get_adc_dither()
 */
int sddc_set_adc_dither(sddc_t *t, int dither);

/**
 * @brief Get ADC randomizer state
 *
 * Randomizer XORs ADC output with pseudo-random sequence to decorrelate
 * spurious tones.
 *
 * @param t Device handle
 *
 * @return 1 if randomizer enabled, 0 if disabled, negative on error
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_adc_random()
 */
int sddc_get_adc_random(sddc_t *t);

/**
 * @brief Enable/disable ADC randomizer
 *
 * @param t Device handle
 * @param random 1 to enable, 0 to disable
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 * @note Can be changed during streaming
 *
 * @see sddc_get_adc_random()
 */
int sddc_set_adc_random(sddc_t *t, int random);

/** @} */ /* end of adc group */


/** @defgroup pga PGA Functions
 *  Programmable Gain Amplifier control
 *  @{
 */

/**
 * @brief Get PGA enable state
 *
 * @param t Device handle
 *
 * @return 1 if PGA enabled, 0 if disabled, negative on error
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_pga()
 */
int sddc_get_pga(sddc_t *t);

/**
 * @brief Enable/disable PGA
 *
 * @param t Device handle
 * @param enable 1 to enable, 0 to disable
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 *
 * @see sddc_get_pga()
 */
int sddc_set_pga(sddc_t *t, int enable);

/** @} */ /* end of pga group */

/** @defgroup vga VGA Functions
 *  AD8370 Variable Gain Amplifier control
 *  @{
 */

/**
 * @brief Get VGA gain setting
 *
 * The gain value is an 8-bit code:
 * - Bit 7 (MSB): 0 = low gain mode, 1 = high gain mode
 * - Bits 0-6: Gain code (0-127)
 *
 * Gain ranges:
 * - High gain mode (bit 7 = 1): 6 dB to 34 dB
 * - Low gain mode (bit 7 = 0): -11 dB to 17 dB
 *
 * @param t Device handle
 *
 * @return Current gain code (0x00-0xFF)
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_vga_gain()
 */
uint8_t sddc_get_vga_gain(sddc_t *t);

/**
 * @brief Set VGA gain
 *
 * @param t Device handle
 * @param gain 8-bit gain code:
 *             - 0x00: Minimum gain (-11 dB, low mode)
 *             - 0x7F: Maximum low mode gain (17 dB)
 *             - 0x80: Minimum high mode gain (6 dB)
 *             - 0xFF: Maximum gain (34 dB, high mode)
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 * @note Can be changed during streaming
 *
 * @code
 * // Set maximum gain
 * sddc_set_vga_gain(dev, 0xFF);
 *
 * // Set mid-range high mode gain (~20 dB)
 * sddc_set_vga_gain(dev, 0x80 | 64);
 * @endcode
 *
 * @see sddc_get_vga_gain()
 */
int sddc_set_vga_gain(sddc_t *t, uint8_t gain);

/** @} */ /* end of vga group */


/** @defgroup hf HF Block Functions
 *  HF direct sampling mode controls (0-32 MHz)
 *  @{
 */

/**
 * @brief Get HF attenuation setting
 *
 * @param t Device handle
 *
 * @return Current attenuation in dB, or negative on error
 *
 * @note Only valid in HF_MODE
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_hf_attenuation()
 */
double sddc_get_hf_attenuation(sddc_t *t);

/**
 * @brief Set HF attenuation
 *
 * @param t Device handle
 * @param attenuation Desired attenuation in dB (hardware-dependent steps)
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Only valid in HF_MODE
 * @note Actual attenuation may be quantized to hardware-supported values
 * @note Thread-safe: No
 *
 * @see sddc_get_hf_attenuation()
 */
int sddc_set_hf_attenuation(sddc_t *t, double attenuation);

/**
 * @brief Get HF bias-T state
 *
 * @param t Device handle
 *
 * @return 1 if bias-T enabled, 0 if disabled, negative on error
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_hf_bias()
 */
int sddc_get_hf_bias(sddc_t *t);

/**
 * @brief Enable/disable HF bias-T
 *
 * Provides DC power on the antenna connector for active antennas or LNAs.
 *
 * @param t Device handle
 * @param bias 1 to enable, 0 to disable
 *
 * @return 0 on success, negative error code on failure
 *
 * @warning Only enable bias-T when connected to bias-T compatible equipment!
 * @note Thread-safe: No
 *
 * @see sddc_get_hf_bias()
 */
int sddc_set_hf_bias(sddc_t *t, int bias);

/** @} */ /* end of hf group */


/** @defgroup vhf VHF Block and Tuner Functions
 *  VHF/UHF tuner mode controls (24 MHz - 1.8 GHz)
 *  @{
 */

/**
 * @brief Get tuner frequency
 *
 * @param t Device handle
 *
 * @return Current tuner frequency in Hz, or negative on error
 *
 * @note Only valid in VHF_MODE
 * @note This is the tuner LO frequency, not the actual RF frequency
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_tuner_frequency()
 * @see sddc_get_tuner_if_frequency()
 */
double sddc_get_tuner_frequency(sddc_t *t);

/**
 * @brief Set tuner frequency
 *
 * @param t Device handle
 * @param frequency Desired RF frequency in Hz (24 MHz - 1.8 GHz)
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Only valid in VHF_MODE
 * @note Actual frequency may differ slightly due to tuner PLL granularity
 * @note Thread-safe: No
 *
 * @see sddc_get_tuner_frequency()
 */
int sddc_set_tuner_frequency(sddc_t *t, double frequency);

/**
 * @brief Get available RF attenuation values
 *
 * @param t Device handle
 * @param[out] attenuations Pointer to receive array of attenuation values (dB)
 *
 * @return Number of available attenuation values, negative on error
 *
 * @note Returned array is static, do not free
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_tuner_rf_attenuation()
 */
int sddc_get_tuner_rf_attenuations(sddc_t *t, const double *attenuations[]);

/**
 * @brief Get current RF attenuation
 *
 * @param t Device handle
 *
 * @return Current RF attenuation in dB, or negative on error
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_tuner_rf_attenuation()
 */
double sddc_get_tuner_rf_attenuation(sddc_t *t);

/**
 * @brief Set RF attenuation
 *
 * @param t Device handle
 * @param attenuation Desired attenuation in dB
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Value will be clamped to nearest supported attenuation
 * @note Thread-safe: No
 *
 * @see sddc_get_tuner_rf_attenuation()
 * @see sddc_get_tuner_rf_attenuations()
 */
int sddc_set_tuner_rf_attenuation(sddc_t *t, double attenuation);

/**
 * @brief Get available IF attenuation values
 *
 * @param t Device handle
 * @param[out] attenuations Pointer to receive array of attenuation values (dB)
 *
 * @return Number of available attenuation values, negative on error
 *
 * @note Returned array is static, do not free
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_tuner_if_attenuation()
 */
int sddc_get_tuner_if_attenuations(sddc_t *t, const double *attenuations[]);

/**
 * @brief Get current IF attenuation
 *
 * @param t Device handle
 *
 * @return Current IF attenuation in dB, or negative on error
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_tuner_if_attenuation()
 */
double sddc_get_tuner_if_attenuation(sddc_t *t);

/**
 * @brief Set IF attenuation
 *
 * @param t Device handle
 * @param attenuation Desired attenuation in dB
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Value will be clamped to nearest supported attenuation
 * @note Thread-safe: No
 *
 * @see sddc_get_tuner_if_attenuation()
 * @see sddc_get_tuner_if_attenuations()
 */
int sddc_set_tuner_if_attenuation(sddc_t *t, double attenuation);

/**
 * @brief Get VHF bias-T state
 *
 * @param t Device handle
 *
 * @return 1 if bias-T enabled, 0 if disabled, negative on error
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_vhf_bias()
 */
int sddc_get_vhf_bias(sddc_t *t);

/**
 * @brief Enable/disable VHF bias-T
 *
 * Provides DC power on the VHF antenna connector for active antennas or LNAs.
 *
 * @param t Device handle
 * @param bias 1 to enable, 0 to disable
 *
 * @return 0 on success, negative error code on failure
 *
 * @warning Only enable bias-T when connected to bias-T compatible equipment!
 * @note Thread-safe: No
 *
 * @see sddc_get_vhf_bias()
 */
int sddc_set_vhf_bias(sddc_t *t, int bias);

/** @} */ /* end of vhf group */


/** @defgroup streaming Streaming Functions
 *  Data acquisition and streaming control
 *  @{
 */

/**
 * @brief Streaming callback function type
 *
 * Called from sddc_handle_events() when new IQ data is available.
 *
 * @param data_size Number of bytes in data buffer
 * @param data Pointer to IQ sample data (32-bit float, interleaved I/Q pairs)
 * @param context User context pointer (from sddc_set_async_params())
 *
 * @note Callback must process data quickly to avoid buffer overflows
 * @note Do not call libsddc functions from within the callback
 * @note Data format: float[2*N] where N = sample count, layout = [I0, Q0, I1, Q1, ...]
 */
typedef void (*sddc_read_async_cb_t)(uint32_t data_size, uint8_t *data,
                                      void *context);

/**
 * @brief Get current sample rate
 *
 * @param t Device handle
 *
 * @return Current sample rate in samples/second, or negative on error
 *
 * @note Valid sample rates: 2, 4, 8, 16, 32 Msps
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_sample_rate()
 */
double sddc_get_sample_rate(sddc_t *t);

/**
 * @brief Set sample rate
 *
 * Configures the output IQ sample rate by adjusting decimation.
 *
 * @param t Device handle
 * @param sample_rate Desired sample rate in Hz
 *                    Valid values: 2e6, 4e6, 8e6, 16e6, 32e6
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Must be set before starting streaming
 * @note Changing sample rate while streaming requires stop/restart
 * @note Thread-safe: No
 *
 * @see sddc_get_sample_rate()
 */
int sddc_set_sample_rate(sddc_t *t, double sample_rate);

/**
 * @brief Configure asynchronous streaming parameters
 *
 * Must be called before sddc_start_streaming().
 *
 * @param t Device handle
 * @param frame_size Size of each callback buffer (currently ignored, reserved)
 * @param num_frames Number of buffers (currently ignored, reserved)
 * @param callback Function to call with IQ data
 * @param callback_context User pointer passed to callback
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 * @note frame_size and num_frames parameters are reserved for future use
 *
 * @see sddc_start_streaming()
 * @see sddc_read_async_cb_t
 */
int sddc_set_async_params(sddc_t *t, uint32_t frame_size,
                          uint32_t num_frames, sddc_read_async_cb_t callback,
                          void *callback_context);

/**
 * @brief Start IQ data streaming
 *
 * Begins continuous data acquisition. Call sddc_handle_events() in a loop
 * to receive callbacks with IQ data.
 *
 * @param t Device handle
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Must call sddc_set_async_params() first
 * @note Device status changes to SDDC_STATUS_STREAMING
 * @note Thread-safe: No
 *
 * @see sddc_stop_streaming()
 * @see sddc_handle_events()
 * @see sddc_set_async_params()
 */
int sddc_start_streaming(sddc_t *t);

/**
 * @brief Process USB events and invoke callbacks
 *
 * Must be called repeatedly while streaming to receive data callbacks.
 * This function blocks until a USB event occurs or timeout.
 *
 * @param t Device handle
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Blocks for up to 1 second waiting for USB events
 * @note Call in a loop while streaming
 * @note Thread-safe: No (don't call from multiple threads on same handle)
 *
 * @code
 * sddc_start_streaming(dev);
 * while (streaming) {
 *     sddc_handle_events(dev);
 * }
 * sddc_stop_streaming(dev);
 * @endcode
 *
 * @see sddc_start_streaming()
 */
int sddc_handle_events(sddc_t *t);

/**
 * @brief Stop IQ data streaming
 *
 * @param t Device handle
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Safe to call even if not streaming
 * @note Device status changes to SDDC_STATUS_READY
 * @note Thread-safe: No
 *
 * @see sddc_start_streaming()
 */
int sddc_stop_streaming(sddc_t *t);

/**
 * @brief Reset device status after error
 *
 * Clears error state and returns device to SDDC_STATUS_READY.
 *
 * @param t Device handle
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 *
 * @see sddc_get_status()
 */
int sddc_reset_status(sddc_t *t);

/**
 * @brief Synchronous read (blocking)
 *
 * Reads IQ data synchronously without using callbacks.
 *
 * @param t Device handle
 * @param[out] data Buffer to receive IQ data
 * @param length Maximum bytes to read
 * @param[out] transferred Actual bytes read (may be NULL)
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Streaming must be started with sddc_start_streaming()
 * @note This function blocks until data is available
 * @note Alternative to async callback model
 * @note Thread-safe: No
 *
 * @see sddc_start_streaming()
 */
int sddc_read_sync(sddc_t *t, uint8_t *data, int length, int *transferred);

/** @} */ /* end of streaming group */


/** @defgroup extended Extended API Functions
 *  Advanced features and low-level hardware access
 *  @{
 */

/**
 * @defgroup error Error Handling
 * Thread-local error reporting for detailed diagnostics
 * @{
 */

/**
 * @brief Get last error code
 *
 * Returns the error code from the most recent failed API call on this thread.
 *
 * @return Error code (negative), or 0 if no error
 *
 * @note Error state is thread-local
 * @note Thread-safe: Yes
 *
 * @see sddc_get_last_error()
 */
int sddc_get_last_error_code(void);

/**
 * @brief Get last error message
 *
 * Returns a human-readable description of the most recent error.
 *
 * @return Error message string, or empty string if no error
 *
 * @note Error state is thread-local
 * @note Returned string is valid until next API call
 * @note Thread-safe: Yes
 *
 * @see sddc_get_last_error_code()
 */
const char* sddc_get_last_error(void);

/** @} */ /* end of error group */

/**
 * @defgroup tuner_if Tuner IF Frequency
 * Configure the intermediate frequency used by VHF tuner
 * @{
 */

/**
 * @brief Get tuner IF frequency
 *
 * The R820T2/R828D tuner downconverts RF to an intermediate frequency (IF).
 * Actual RF frequency = tuner_frequency + IF_frequency
 *
 * @param t Device handle
 *
 * @return IF frequency in Hz (default: 4.57 MHz)
 *
 * @note Only relevant in VHF_MODE
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_set_tuner_if_frequency()
 */
double sddc_get_tuner_if_frequency(sddc_t *t);

/**
 * @brief Set tuner IF frequency
 *
 * Advanced users can adjust IF frequency for specific applications.
 *
 * @param t Device handle
 * @param frequency IF frequency in Hz
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Only relevant in VHF_MODE
 * @note Default is 4.57 MHz - change only if you know what you're doing
 * @note Thread-safe: No
 *
 * @see sddc_get_tuner_if_frequency()
 */
int sddc_set_tuner_if_frequency(sddc_t *t, double frequency);

/** @} */ /* end of tuner_if group */

/**
 * @defgroup gpio GPIO Control
 * Low-level GPIO access for advanced hardware control
 * @{
 */

/**
 * @brief Set GPIO pins
 *
 * Sets specified GPIO bits to given values.
 *
 * @param t Device handle
 * @param value Value to set (1 bits)
 * @param mask Which bits to modify
 *
 * @return 0 on success, negative error code on failure
 *
 * @note This is a low-level function - prefer high-level functions when available
 * @note Thread-safe: No
 *
 * @code
 * // Turn on yellow LED only
 * uint32_t led_mask = sddc_gpio_led_yellow() | sddc_gpio_led_red() | sddc_gpio_led_blue();
 * sddc_gpio_set(dev, sddc_gpio_led_yellow(), led_mask);
 * @endcode
 *
 * @see sddc_gpio_on()
 * @see sddc_gpio_off()
 */
int sddc_gpio_set(sddc_t *t, uint32_t value, uint32_t mask);

/**
 * @brief Set GPIO pins high
 *
 * @param t Device handle
 * @param mask Bitmask of pins to set high
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 *
 * @see sddc_gpio_set()
 * @see sddc_gpio_off()
 */
int sddc_gpio_on(sddc_t *t, uint32_t mask);

/**
 * @brief Set GPIO pins low
 *
 * @param t Device handle
 * @param mask Bitmask of pins to set low
 *
 * @return 0 on success, negative error code on failure
 *
 * @note Thread-safe: No
 *
 * @see sddc_gpio_set()
 * @see sddc_gpio_on()
 */
int sddc_gpio_off(sddc_t *t, uint32_t mask);

/**
 * @brief Read current GPIO state
 *
 * @param t Device handle
 *
 * @return Current GPIO register value (32-bit)
 *
 * @note Thread-safe: Yes (read-only)
 *
 * @see sddc_gpio_set()
 */
uint32_t sddc_gpio_get(sddc_t *t);

/* GPIO bit masks - use these helper functions for portability */

/** @brief Get VHF antenna path enable bit (RX888R2+) */
uint32_t sddc_gpio_vhf_en(void);

/** @brief Get HF bias-T enable bit */
uint32_t sddc_gpio_bias_hf(void);

/** @brief Get VHF bias-T enable bit */
uint32_t sddc_gpio_bias_vhf(void);

/** @brief Get ADC dither enable bit */
uint32_t sddc_gpio_dither(void);

/** @brief Get ADC randomizer enable bit */
uint32_t sddc_gpio_random(void);

/** @brief Get attenuator select bit 0 */
uint32_t sddc_gpio_att_sel0(void);

/** @brief Get attenuator select bit 1 */
uint32_t sddc_gpio_att_sel1(void);

/** @brief Get yellow LED bit */
uint32_t sddc_gpio_led_yellow(void);

/** @brief Get red LED bit */
uint32_t sddc_gpio_led_red(void);

/** @brief Get blue LED bit */
uint32_t sddc_gpio_led_blue(void);

/** @} */ /* end of gpio group */

/** @} */ /* end of extended group */

#ifdef __cplusplus
}
#endif

#endif /* __LIBSDDC_H */