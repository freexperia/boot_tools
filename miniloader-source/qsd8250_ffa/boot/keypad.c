/* Copyright 2007, Google Inc. */

#include <boot/boot.h>
#include <boot/gpio_keypad.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static unsigned int halibut_row_gpios[] = { 31, 32, 33, 34, 35, 41 };
static unsigned int halibut_col_gpios[] = { 36, 37, 38, 39, 40 };

static unsigned int halibut_key_map[] = {
    [11] = BOOT_KEY_CONTINUE_BOOT,      /* FA on SURF, B on FFA */
    [23] = BOOT_KEY_STOP_BOOT,          /* FB on SURF */
    [27] = BOOT_KEY_STOP_BOOT,          /* 2 on FFA */
    [25] = BOOT_KEY_STOP_BOOT,          /* top left key on 8K FFA */
};

static gpio_keypad_info halibut_keypad = {
    .output_gpios = halibut_row_gpios,
    .input_gpios = halibut_col_gpios,
    .noutputs = ARRAY_SIZE(halibut_row_gpios),
    .ninputs = ARRAY_SIZE(halibut_col_gpios),
    .key_map = halibut_key_map,
    .settle_time = 5000,
    .polarity = 0,
    .drive_inactive_outputs = 1
};

static void keypad_poll()
{
    static int skip = 0;
    skip++;
    //increased the counter so we have breathing time for usb_poll in fastboot mode 
    if(skip > KEYPAD_DELAY) { 
        gpio_keypad_scan_keys(&halibut_keypad);
        skip = 0;
    }
}

void keypad_init(void)
{
    gpio_keypad_init(&halibut_keypad);
    boot_register_poll_func(keypad_poll);
}
