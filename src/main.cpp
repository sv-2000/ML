#include <version.h>
#if (KERNEL_VERSION_MAJOR > 3) || ((KERNEL_VERSION_MAJOR == 3) && (KERNEL_VERSION_MINOR >= 1))
#include <zephyr/kernel.h>
#else
#include <zephyr.h>
#endif
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <stdio.h>
#include <string.h>

#define START_MARKER_1 0xAA
#define START_MARKER_2 0x55
#define IMAGE_WIDTH 200
#define IMAGE_HEIGHT 200
#define IMAGE_SIZE (IMAGE_WIDTH * IMAGE_HEIGHT)

uint8_t image_raw[IMAGE_SIZE];
float image_normalized[IMAGE_SIZE];

bool receive_uart_image(void) {
    const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return false;
    }

    uint8_t byte;
    uint8_t marker1 = 0, marker2 = 0;

    printk("Waiting for image marker...\n");

    // Wait for START_MARKER_1
    do {
        while (uart_poll_in(uart_dev, &marker1) < 0) {
            k_msleep(1);
        }
    } while (marker1 != START_MARKER_1);

    // Wait for START_MARKER_2
    while (uart_poll_in(uart_dev, &marker2) < 0) {
        k_msleep(1);
    }

    if (marker2 != START_MARKER_2) {
        printk("Invalid start marker!\n");
        return false;
    }

    printk("Start marker received. Receiving image data...\n");

    for (int i = 0; i < IMAGE_SIZE; i++) {
        while (uart_poll_in(uart_dev, &image_raw[i]) < 0) {
            k_msleep(1);
        }
    }

    for (int i = 0; i < IMAGE_SIZE; i++) {
        image_normalized[i] = image_raw[i] / 255.0f;
    }

    printk("Image received. First 5 normalized pixels: %f, %f, %f, %f, %f\n",
           image_normalized[0], image_normalized[1],
           image_normalized[2], image_normalized[3],
           image_normalized[4]);

    return true;
}

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    printk("UART Image Receiver Initialized\n");

    while (1) {
        bool success = receive_uart_image();
        if (!success) {
            printk("Failed to receive image.\n");
        }
        k_msleep(1000);
    }
}
