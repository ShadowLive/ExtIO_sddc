#include "libsddc.h"
#include "config.h"
#include "RadioHandler.h"
#include <cmath>
#include <cstring>
#include <cstdarg>

// R820T2/R828D tuner IF frequency (4.57 MHz)
#define TUNER_IF_FREQUENCY 4570000.0

// Sample rates indexed by samplerateidx
// RadioHandler::Start sets: decimate = 4 - srate_idx
// R2IQ mdecimation table for 64 MSPS ADC (from r2iq.h):
//   mdecimation=0 → 32 MSPS (ratio=2)
//   mdecimation=1 → 16 MSPS (ratio=4)
//   mdecimation=2 → 8 MSPS (ratio=8)
//   mdecimation=3 → 4 MSPS (ratio=16)
//   mdecimation=4 → 2 MSPS (ratio=32)
// So srate_idx maps to output rate:
static const double sample_rates[] = {
    2000000.0,   // idx 0: decimate=4 → mdecimation=4 → 2 MSPS
    4000000.0,   // idx 1: decimate=3 → mdecimation=3 → 4 MSPS
    8000000.0,   // idx 2: decimate=2 → mdecimation=2 → 8 MSPS
    16000000.0,  // idx 3: decimate=1 → mdecimation=1 → 16 MSPS
    32000000.0,  // idx 4: decimate=0 → mdecimation=0 → 32 MSPS
};
static const int num_sample_rates = sizeof(sample_rates) / sizeof(sample_rates[0]);

// Error handling
static thread_local char last_error[256] = {0};
static thread_local int last_error_code = 0;

static void set_error(int code, const char* fmt, ...) {
    last_error_code = code;
    va_list args;
    va_start(args, fmt);
    vsnprintf(last_error, sizeof(last_error), fmt, args);
    va_end(args);
    fprintf(stderr, "libsddc error: %s\n", last_error);
}

static void clear_error() {
    last_error_code = 0;
    last_error[0] = '\0';
}

struct sddc
{
    SDDCStatus status;
    RadioHandlerClass* handler;
    fx3class* fx3;
    uint8_t led;
    int samplerateidx;
    double freq;
    double tuner_if_freq;  // Tuner IF frequency (configurable)

    sddc_read_async_cb_t callback;
    void *callback_context;
};

sddc_t *current_running;

// Ring buffer for synchronous reads
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <chrono>
#include <algorithm>

static std::mutex sync_mutex;
static std::condition_variable sync_cv;
static uint8_t sync_buffer[1024 * 1024];  // 1MB buffer
static size_t sync_write_pos = 0;
static size_t sync_read_pos = 0;
static size_t sync_available = 0;


static void Callback(void* context, const float* data, uint32_t len)
{
    if (!current_running) return;

    // Forward to user callback if set
    if (current_running->callback) {
        // Convert float samples to int16 for the callback
        // The callback expects uint8_t* but data is float*
        // For now, pass as-is since the callback format isn't well-defined
        current_running->callback(len * sizeof(float), (uint8_t*)data,
                                  current_running->callback_context);
    }

    // Store in sync buffer for sddc_read_sync
    // len = number of complex samples, each complex = 2 floats (I + Q)
    const size_t bytes = len * 2 * sizeof(float);
    const uint8_t* src = (const uint8_t*)data;
    constexpr size_t buffer_size = sizeof(sync_buffer);

    {
        std::lock_guard<std::mutex> lock(sync_mutex);

        // Calculate contiguous space from write position to end of buffer
        size_t to_end = buffer_size - sync_write_pos;

        if (bytes <= to_end) {
            // Data fits in one contiguous block
            memcpy(&sync_buffer[sync_write_pos], src, bytes);
            sync_write_pos = (sync_write_pos + bytes) % buffer_size;
        } else {
            // Data wraps around - copy in two parts
            memcpy(&sync_buffer[sync_write_pos], src, to_end);
            memcpy(sync_buffer, src + to_end, bytes - to_end);
            sync_write_pos = bytes - to_end;
        }

        // Update available count, handling overflow
        if (sync_available + bytes <= buffer_size) {
            sync_available += bytes;
        } else {
            // Buffer overflow - discard old data
            size_t overflow = (sync_available + bytes) - buffer_size;
            sync_read_pos = (sync_read_pos + overflow) % buffer_size;
            sync_available = buffer_size;
        }
    }
    sync_cv.notify_one();
}

int sddc_get_device_count()
{
    return 1;
}

int sddc_get_device_info(struct sddc_device_info **sddc_device_infos)
{
    auto ret = new sddc_device_info();
    const char *todo = "TODO";
    ret->manufacturer = todo;
    ret->product = todo;
    ret->serial_number = todo;

    *sddc_device_infos = ret;

    return 1;
}

int sddc_free_device_info(struct sddc_device_info *sddc_device_infos)
{
    delete sddc_device_infos;
    return 0;
}

sddc_t *sddc_open(int index, const char* imagefile)
{
    clear_error();
    auto ret_val = new sddc_t();

    fx3class *fx3 = CreateUsbHandler();
    if (fx3 == nullptr)
    {
        set_error(-1, "Failed to create USB handler");
        delete ret_val;
        return nullptr;
    }

    // open the firmware
    unsigned char* res_data;
    uint32_t res_size;

    FILE *fp = fopen(imagefile, "rb");
    if (fp == nullptr)
    {
        set_error(-2, "Failed to open firmware file: %s", imagefile);
        delete fx3;
        delete ret_val;
        return nullptr;
    }

    fseek(fp, 0, SEEK_END);
    res_size = ftell(fp);
    res_data = (unsigned char*)malloc(res_size);
    fseek(fp, 0, SEEK_SET);
    if (fread(res_data, 1, res_size, fp) != res_size) {
        set_error(-3, "Failed to read firmware file");
        fclose(fp);
        free(res_data);
        delete fx3;
        delete ret_val;
        return nullptr;
    }
    fclose(fp);

    bool openOK = fx3->Open();
    if (!openOK) {
        set_error(-4, "Failed to open FX3 USB device");
        free(res_data);
        delete fx3;
        delete ret_val;
        return nullptr;
    }

    ret_val->handler = new RadioHandlerClass();
    ret_val->fx3 = fx3;
    ret_val->tuner_if_freq = TUNER_IF_FREQUENCY;  // Default 4.57 MHz

    if (ret_val->handler->Init(fx3, Callback, nullptr))
    {
        ret_val->status = SDDC_STATUS_READY;
        ret_val->samplerateidx = 4;  // Default to 32 MSPS
    }

    free(res_data);
    return ret_val;
}

void sddc_close(sddc_t *that)
{
    if (that->handler)
        delete that->handler;
    delete that;
}

enum SDDCStatus sddc_get_status(sddc_t *t)
{
    return t->status;
}

enum SDDCHWModel sddc_get_hw_model(sddc_t *t)
{
    switch(t->handler->getModel())
    {
        case RadioModel::BBRF103:
            return HW_BBRF103;
        case RadioModel::HF103:
            return HW_HF103;
        case RadioModel::RX888:
            return HW_RX888;
        case RadioModel::RX888r2:
            return HW_RX888R2;
        case RadioModel::RX888r3:
            return HW_RX888R3;
        case RadioModel::RX999:
            return HW_RX999;
        default:
            return HW_NORADIO;
    }
}

const char *sddc_get_hw_model_name(sddc_t *t)
{
    return t->handler->getName();
}

uint16_t sddc_get_firmware(sddc_t *t)
{
    return t->handler->GetFirmware();
}

const double *sddc_get_frequency_range(sddc_t *t)
{
    return nullptr;
}

enum RFMode sddc_get_rf_mode(sddc_t *t)
{
    switch(t->handler->GetmodeRF())
    {
        case HFMODE:
            return RFMode::HF_MODE;
        case VHFMODE:
            return RFMode::VHF_MODE;
        default:
            return RFMode::NO_RF_MODE;
    }
}

int sddc_set_rf_mode(sddc_t *t, enum RFMode rf_mode)
{
    switch (rf_mode)
    {
    case VHF_MODE:
        fprintf(stderr, "libsddc: Setting VHF mode (VHF antenna port)\n");
        t->handler->UpdatemodeRF(VHFMODE);
        break;
    case HF_MODE:
        fprintf(stderr, "libsddc: Setting HF mode (HF antenna port)\n");
        t->handler->UpdatemodeRF(HFMODE);
        break;
    default:
        fprintf(stderr, "libsddc: Unknown RF mode %d\n", rf_mode);
        return -1;
    }

    return 0;
}

/* LED functions */
int sddc_led_on(sddc_t *t, uint8_t led_pattern)
{
    if (led_pattern & YELLOW_LED)
        t->handler->uptLed(0, true);
    if (led_pattern & RED_LED)
        t->handler->uptLed(1, true);
    if (led_pattern & BLUE_LED)
        t->handler->uptLed(2, true);

    t->led |= led_pattern;

    return 0;
}

int sddc_led_off(sddc_t *t, uint8_t led_pattern)
{
    if (led_pattern & YELLOW_LED)
        t->handler->uptLed(0, false);
    if (led_pattern & RED_LED)
        t->handler->uptLed(1, false);
    if (led_pattern & BLUE_LED)
        t->handler->uptLed(2, false);

    t->led &= ~led_pattern;

    return 0;
}

int sddc_led_toggle(sddc_t *t, uint8_t led_pattern)
{
    t->led = t->led ^ led_pattern;
    if (t->led & YELLOW_LED)
        t->handler->uptLed(0, false);
    if (t->led & RED_LED)
        t->handler->uptLed(1, false);
    if (t->led & BLUE_LED)
        t->handler->uptLed(2, false);

    return 0;
}


/* ADC functions */
int sddc_get_adc_dither(sddc_t *t)
{
    return t->handler->GetDither();
}

int sddc_set_adc_dither(sddc_t *t, int dither)
{
    t->handler->UptDither(dither != 0);
    return 0;
}

int sddc_get_adc_random(sddc_t *t)
{
    return t->handler->GetRand();
}

int sddc_set_adc_random(sddc_t *t, int random)
{
    t->handler->UptRand(random != 0);
    return 0;
}

/* PGA (Programmable Gain Amplifier) functions */
int sddc_get_pga(sddc_t *t)
{
    return t->handler->GetPga();
}

int sddc_set_pga(sddc_t *t, int enable)
{
    t->handler->UptPga(enable != 0);
    return 0;
}

/* AD8370 VGA (Variable Gain Amplifier) functions */
uint8_t sddc_get_vga_gain(sddc_t *t)
{
    return t->handler->GetVgaGain();
}

int sddc_set_vga_gain(sddc_t *t, uint8_t gain)
{
    t->handler->SetVgaGain(gain);
    return 0;
}

/* HF block functions */
double sddc_get_hf_attenuation(sddc_t *t)
{
    return 0;
}

int sddc_set_hf_attenuation(sddc_t *t, double attenuation)
{
    return 0;
}

int sddc_get_hf_bias(sddc_t *t)
{
    return t->handler->GetBiasT_HF();
}

int sddc_set_hf_bias(sddc_t *t, int bias)
{
    t->handler->UpdBiasT_HF(bias != 0);
    return 0;
}


/* VHF block and VHF/UHF tuner functions */
double sddc_get_tuner_frequency(sddc_t *t)
{
    return t->freq;
}

int sddc_set_tuner_frequency(sddc_t *t, double frequency)
{
    t->freq = t->handler->TuneLO((uint64_t)frequency);

    return 0;
}

// Static storage for step values converted to double
static double rf_steps_double[64];
static double if_steps_double[64];
static int current_rf_idx = 0;
static int current_if_idx = 0;

int sddc_get_tuner_rf_attenuations(sddc_t *t, const double *attenuations[])
{
    const float *steps;
    int count = t->handler->GetRFAttSteps(&steps);
    if (count > 64) count = 64;
    for (int i = 0; i < count; i++) {
        rf_steps_double[i] = steps[i];
    }
    *attenuations = rf_steps_double;
    return count;
}

double sddc_get_tuner_rf_attenuation(sddc_t *t)
{
    const float *steps;
    int count = t->handler->GetRFAttSteps(&steps);
    if (current_rf_idx >= 0 && current_rf_idx < count) {
        return steps[current_rf_idx];
    }
    return 0;
}

int sddc_set_tuner_rf_attenuation(sddc_t *t, double attenuation)
{
    const float *steps;
    int count = t->handler->GetRFAttSteps(&steps);
    if (count <= 0) return -1;

    // Find closest step
    int best_idx = 0;
    double best_diff = fabs(steps[0] - attenuation);
    for (int i = 1; i < count; i++) {
        double diff = fabs(steps[i] - attenuation);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx = i;
        }
    }

    current_rf_idx = best_idx;
    t->handler->UpdateattRF(best_idx);
    return 0;
}

int sddc_get_tuner_if_attenuations(sddc_t *t, const double *attenuations[])
{
    const float *steps;
    int count = t->handler->GetIFGainSteps(&steps);
    if (count > 64) count = 64;
    for (int i = 0; i < count; i++) {
        if_steps_double[i] = steps[i];
    }
    *attenuations = if_steps_double;
    return count;
}

double sddc_get_tuner_if_attenuation(sddc_t *t)
{
    const float *steps;
    int count = t->handler->GetIFGainSteps(&steps);
    if (current_if_idx >= 0 && current_if_idx < count) {
        return steps[current_if_idx];
    }
    return 0;
}

int sddc_set_tuner_if_attenuation(sddc_t *t, double attenuation)
{
    const float *steps;
    int count = t->handler->GetIFGainSteps(&steps);
    if (count <= 0) return -1;

    // Find closest step
    int best_idx = 0;
    double best_diff = fabs(steps[0] - attenuation);
    for (int i = 1; i < count; i++) {
        double diff = fabs(steps[i] - attenuation);
        if (diff < best_diff) {
            best_diff = diff;
            best_idx = i;
        }
    }

    current_if_idx = best_idx;
    t->handler->UpdateIFGain(best_idx);
    return 0;
}

int sddc_get_vhf_bias(sddc_t *t)
{
    return t->handler->GetBiasT_VHF();
}

int sddc_set_vhf_bias(sddc_t *t, int bias)
{
    t->handler->UpdBiasT_VHF(bias != 0);
    return 0;
}

double sddc_get_sample_rate(sddc_t *t)
{
    if (t->samplerateidx >= 0 && t->samplerateidx < num_sample_rates) {
        return sample_rates[t->samplerateidx];
    }
    return 0;
}

int sddc_set_sample_rate(sddc_t *t, double sample_rate)
{
    // R2IQ mdecimation maps to output rate (for 64 MSPS ADC):
    //   srate_idx=0 → decimate=4 → mdecimation=4 → 2 MSPS
    //   srate_idx=1 → decimate=3 → mdecimation=3 → 4 MSPS
    //   srate_idx=2 → decimate=2 → mdecimation=2 → 8 MSPS
    //   srate_idx=3 → decimate=1 → mdecimation=1 → 16 MSPS
    //   srate_idx=4 → decimate=0 → mdecimation=0 → 32 MSPS
    int64_t rate = (int64_t)sample_rate;
    switch(rate)
    {
        case 32000000:
            t->samplerateidx = 4;  // mdecimation=0, output=32 MSPS
            break;
        case 16000000:
            t->samplerateidx = 3;  // mdecimation=1, output=16 MSPS
            break;
        case 8000000:
            t->samplerateidx = 2;  // mdecimation=2, output=8 MSPS
            break;
        case 4000000:
            t->samplerateidx = 1;  // mdecimation=3, output=4 MSPS
            break;
        case 2000000:
            t->samplerateidx = 0;  // mdecimation=4, output=2 MSPS
            break;
        default:
            set_error(-1, "Unsupported sample rate: %.0f Hz", sample_rate);
            return -1;
    }
    return 0;
}

int sddc_set_async_params(sddc_t *t, uint32_t frame_size, 
                          uint32_t num_frames, sddc_read_async_cb_t callback,
                          void *callback_context)
{
    // TODO: ignore frame_size, num_frames
    t->callback = callback;
    t->callback_context = callback_context;
    return 0;
}

int sddc_start_streaming(sddc_t *t)
{
    current_running = t;
    t->handler->Start(t->samplerateidx);
    return 0;
}

int sddc_handle_events(sddc_t *t)
{
    return 0;
}

int sddc_stop_streaming(sddc_t *t)
{
    t->handler->Stop();
    current_running = nullptr;
    return 0;
}

int sddc_reset_status(sddc_t *t)
{
    return 0;
}

int sddc_read_sync(sddc_t *t, uint8_t *data, int length, int *transferred)
{
    std::unique_lock<std::mutex> lock(sync_mutex);

    // Wait for data with timeout
    if (sync_available == 0) {
        sync_cv.wait_for(lock, std::chrono::milliseconds(1000),
                         []{ return sync_available > 0; });
    }

    if (sync_available == 0) {
        *transferred = 0;
        return 0;  // Timeout, no data
    }

    // Read available data using memcpy
    size_t to_read = std::min((size_t)length, sync_available);
    constexpr size_t buffer_size = sizeof(sync_buffer);

    // Calculate contiguous data from read position to end of buffer
    size_t to_end = buffer_size - sync_read_pos;

    if (to_read <= to_end) {
        // Data is in one contiguous block
        memcpy(data, &sync_buffer[sync_read_pos], to_read);
        sync_read_pos = (sync_read_pos + to_read) % buffer_size;
    } else {
        // Data wraps around - copy in two parts
        memcpy(data, &sync_buffer[sync_read_pos], to_end);
        memcpy(data + to_end, sync_buffer, to_read - to_end);
        sync_read_pos = to_read - to_end;
    }
    sync_available -= to_read;

    *transferred = (int)to_read;
    return 0;
}

/* ============================================================================
 * NEW API FUNCTIONS
 * ============================================================================ */

/* Error handling functions */
int sddc_get_last_error_code(void)
{
    return last_error_code;
}

const char* sddc_get_last_error(void)
{
    return last_error[0] ? last_error : nullptr;
}

/* Tuner IF frequency functions */
double sddc_get_tuner_if_frequency(sddc_t *t)
{
    return t->tuner_if_freq;
}

int sddc_set_tuner_if_frequency(sddc_t *t, double frequency)
{
    if (frequency < 0 || frequency > 10000000) {  // Sanity check: 0-10 MHz
        set_error(-10, "Invalid IF frequency: %.0f Hz (must be 0-10 MHz)", frequency);
        return -1;
    }
    t->tuner_if_freq = frequency;
    return 0;
}

/* GPIO control functions */
int sddc_gpio_set(sddc_t *t, uint32_t value, uint32_t mask)
{
    if (!t || !t->fx3) {
        set_error(-11, "Invalid device handle for GPIO control");
        return -1;
    }

    // Use the FX3 Control function to send GPIO command
    // The GPIOFX3 command (0xAD) takes the full GPIO state as a 32-bit value
    // We need to read current state, modify with mask, and write back

    // Unfortunately RadioHandler doesn't expose the current GPIO state directly,
    // so we'll track it ourselves
    static uint32_t gpio_state = 0;

    gpio_state = (gpio_state & ~mask) | (value & mask);

    bool ok = t->fx3->Control(GPIOFX3, gpio_state);
    if (!ok) {
        set_error(-12, "GPIO control command failed");
        return -1;
    }

    return 0;
}

int sddc_gpio_on(sddc_t *t, uint32_t mask)
{
    return sddc_gpio_set(t, mask, mask);
}

int sddc_gpio_off(sddc_t *t, uint32_t mask)
{
    return sddc_gpio_set(t, 0, mask);
}

uint32_t sddc_gpio_get(sddc_t *t)
{
    // Return the current tracked GPIO state
    // Note: This doesn't read from hardware, just returns last set value
    static uint32_t gpio_state = 0;
    return gpio_state;
}

/* GPIO bit definitions - expose via API */
uint32_t sddc_gpio_vhf_en(void) { return VHF_EN; }
uint32_t sddc_gpio_bias_hf(void) { return BIAS_HF; }
uint32_t sddc_gpio_bias_vhf(void) { return BIAS_VHF; }
uint32_t sddc_gpio_dither(void) { return DITH; }
uint32_t sddc_gpio_random(void) { return RANDO; }
uint32_t sddc_gpio_att_sel0(void) { return ATT_SEL0; }
uint32_t sddc_gpio_att_sel1(void) { return ATT_SEL1; }
uint32_t sddc_gpio_led_yellow(void) { return LED_YELLOW; }
uint32_t sddc_gpio_led_red(void) { return LED_RED; }
uint32_t sddc_gpio_led_blue(void) { return LED_BLUE; }
