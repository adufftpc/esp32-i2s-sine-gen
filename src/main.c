#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

#include <driver/gpio.h>
// #include <driver/i2s.h>
#include "driver/i2s_std.h"

#include <math.h>


#define LED_HB_DURATION_MS      ( 100 )

#define BUFF_SIZE               ( 800 )

#define I2S_BCLK_IO1            GPIO_NUM_4      // I2S bit clock io number
#define I2S_WS_IO1              GPIO_NUM_5      // I2S word select io number
#define I2S_DOUT_IO1            GPIO_NUM_18     // I2S data out io number

#define I2S_SAMPLE_RATE         ( 44100 )
#define I2S_DATA_SIZE           ( I2S_DATA_BIT_WIDTH_32BIT )
#define I2S_WS_SIZE             ( I2S_DATA_BIT_WIDTH_32BIT )

#define SINE_FREQ_HZ            ( 440.0f)

#define TWOPI                   ( 6.28318531f )

static i2s_chan_handle_t        tx_chan;        // I2S tx channel handler


size_t generateSineWave(float freq, uint16_t sampleRate, uint32_t *dst, size_t size) {
    float p = 0.;
    float samp = 0.;
    uint32_t val;
    size_t periodSize = 0;
    float phaseInc = (float)freq * TWOPI / (float)(sampleRate);

    for (ssize_t i=0; i < size; i += 2) {
        samp = (sinf(p) + 1.0f) * .5f;
        val = samp * 0x00FFFFFF;
        // printf("%6d: phase: %8.4f ratio: %f val: %u \n", i, p, samp, val);
        dst[i] = val;
        dst[i+1] = val;
        p += phaseInc;
        periodSize++;
        // Generate 1 period only
        if (p >= TWOPI)
            break;
    }
    return periodSize << 1;
}

static void i2s_write_task(void *args)
{
    uint8_t *w_buf = (uint8_t *)calloc(sizeof(uint32_t), BUFF_SIZE);
    assert(w_buf); // Check if w_buf allocation success

    size_t w_bytes = BUFF_SIZE;
    size_t generated = generateSineWave(SINE_FREQ_HZ, I2S_SAMPLE_RATE, (uint32_t *)w_buf, BUFF_SIZE);

    w_bytes *= sizeof(uint32_t);
    while (w_bytes == BUFF_SIZE) {
        i2s_channel_preload_data(tx_chan, w_buf, BUFF_SIZE, &w_bytes);
    }

    // ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    while (1) {
        /* Write i2s data */
        if (i2s_channel_write(tx_chan, w_buf, BUFF_SIZE, &w_bytes, 1000) == ESP_OK) {
            // if (gpio_get_level(GPIO_NUM_2))
            //     gpio_set_level(GPIO_NUM_2, 0);
            // else
            //     gpio_set_level(GPIO_NUM_2, 1);
            // printf("Write Task: i2s write %d bytes\n", w_bytes);
        } else {
            // gpio_set_level(GPIO_NUM_2, 0);
            // gpio_set_level(GPIO_NUM_2, 1);
            // gpio_set_level(GPIO_NUM_2, 0);
            // gpio_set_level(GPIO_NUM_2, 1);
            // gpio_set_level(GPIO_NUM_2, 0);
            // while(1);
            // gpio_get_level(GPIO_NUM_2) ? gpio_set_level(GPIO_NUM_2, 0) : gpio_set_level(GPIO_NUM_2, 1);
            // printf("Write Task: i2s write failed\n");
        }
        // gpio_set_level(GPIO_NUM_2, 1);
        // vTaskDelay(pdMS_TO_TICKS(200));
        // gpio_set_level(GPIO_NUM_2, 0);
    }
    free(w_buf);
    vTaskDelete(NULL);
}

static void i2s_std_init(void)
{
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL);

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        // .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_SIZE,
            .slot_bit_width = I2S_WS_SIZE,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_WS_SIZE,
            .ws_pol = false,
            .bit_shift = false,
            .msb_right = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_IO1,
            .ws   = I2S_WS_IO1,
            .dout = I2S_DOUT_IO1,
            .din  = -1,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };


    i2s_channel_init_std_mode(tx_chan, &tx_std_cfg);
}


void led_blinker_task(void *pvParameter)
{
    while (1) {
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_HB_DURATION_MS));

        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(pdMS_TO_TICKS(LED_HB_DURATION_MS));

        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_HB_DURATION_MS));

        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(pdMS_TO_TICKS(1000 - LED_HB_DURATION_MS * 3));
    }
}

void app_main()
{
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    i2s_std_init();

    xTaskCreatePinnedToCore(i2s_write_task,     "i2s_writer",   4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(led_blinker_task,   "LEDS",         2048, NULL, 1, NULL, 1);

    while(1);
}
