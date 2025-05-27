#include <version.h>
#if (KERNEL_VERSION_MAJOR > 3) || ((KERNEL_VERSION_MAJOR == 3) && (KERNEL_VERSION_MINOR >= 1))
#include <zephyr/kernel.h>
#else
#include <zephyr.h>
#endif
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/timing/timing.h> // For k_uptime_get for basic timeout
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include <hal/nrf_clock.h>

#ifdef EI_NORDIC
#include <nrfx_clock.h>
#endif

const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

// Buffer to hold the MNIST image data received via UART.
// Size is determined by EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE.
static float mnist_image[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

static int receive_image_data_from_uart(float* buffer, size_t float_count) {
    size_t bytes_to_receive = float_count * sizeof(float);
    unsigned char* rx_buf = (unsigned char*)buffer; // Cast buffer to unsigned char* for byte-wise reception
    size_t bytes_received = 0;

    printk("Waiting for %zu bytes of image data via UART (expected %zu floats)...\n", bytes_to_receive, float_count);

    // Basic timeout mechanism
    int60_t start_time = k_uptime_get();
    // Adjust timeout as needed. For 784 floats (3136 bytes), even at 9600 baud, 
    // this should be a few seconds. Add buffer.
    int64_t timeout_ms = 15000; // 15 seconds timeout 

    while (bytes_received < bytes_to_receive) {
        if (k_uptime_get() - start_time > timeout_ms) {
            printk("ERROR: Timeout waiting for UART data. Received %zu of %zu bytes.\n", bytes_received, bytes_to_receive);
            return -1; // Timeout error
        }

        unsigned char byte;
        // uart_poll_in returns 0 on success (byte received), -1 if no data, negative errno on error
        if (uart_poll_in(uart_dev, &byte) == 0) {
            rx_buf[bytes_received++] = byte;
            // Optional: print progress for debugging, can be very verbose
            // if (bytes_received % 100 == 0) {
            //    printk("Received %zu bytes...\n", bytes_received);
            // }
        } else {
            // No data currently available, or an error occurred.
            // A short sleep prevents busy-waiting and hogging CPU.
            // In a more complex app, k_poll() on the UART device descriptor would be better.
            k_msleep(1); 
        }
    }

    printk("Successfully received %zu bytes of image data.\n", bytes_received);
    return 0; // Success
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, mnist_image + offset, length * sizeof(float));
    return 0;
}

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);

#ifdef CONFIG_SOC_NRF5340_CPUAPP
    const struct device *clock_dev = DEVICE_DT_GET_ONE(nordic_nrf_clock);
    if (device_is_ready(clock_dev)) {
        clock_control_on(clock_dev, CLOCK_CONTROL_NRF_SUBSYS_HF);
    }
#endif

    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return 1; 
    }
    printk("UART device is ready.\n"); // Add a confirmation message

    printk("Edge Impulse MNIST digit classification (Zephyr)\n");

    if (sizeof(mnist_image) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printk("Invalid input size. Expected %d floats, got %u\n",
               EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(mnist_image) / sizeof(float));
        return 1;
    }

    ei_impulse_result_t result = { 0 };

    while (1) {
        // Receive image data from UART
        if (receive_image_data_from_uart(mnist_image, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) != 0) {
            printk("Failed to receive image data. Retrying...\n");
            k_msleep(1000); // Add a short delay before retrying
            continue;       // Skip to the next iteration
        }

        signal_t features_signal;
        features_signal.total_length = sizeof(mnist_image) / sizeof(mnist_image[0]);
        features_signal.get_data = &raw_feature_get_data;

        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
        // printk("run_classifier returned: %d\n", res); // Optional: keep for debugging

        if (res != 0) {
            printk("ERROR: run_classifier failed with error %d\n", res);
            k_msleep(1000); // Optionally, add a small delay here too before retrying
            continue; 
        }

        printk("Predictions (DSP: %d ms, Classification: %d ms, Anomaly: %d ms):\n",
               result.timing.dsp, result.timing.classification, result.timing.anomaly);

   // Output for classification models (like MNIST)
   printk("Classification results:\n");
   for (uint32_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
       ei_impulse_result_classification_t classification = result.classification[i];
       // Print label and value, ensuring label is not null if possible
       // (The Edge Impulse SDK usually provides valid labels for active classifications)
       if (classification.label) { // Check if label pointer is not null
            printk("  %s: %.5f\n", classification.label, classification.value);
       } else {
            // Fallback if label is null for some reason, print index
            printk("  Label %u: %.5f\n", i, classification.value);
       }
   }

   // Output for object detection models (retained for generality)
   if (result.bounding_boxes_count > 0) {
       printk("Bounding boxes:\n");
       for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
           ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
           if (bb.value == 0) continue; 
           printk("  Object: '%s' (%.2f) [x=%d, y=%d, w=%d, h=%d]\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
       }
   }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
   printk("Anomaly score: %.3f\n", result.anomaly);
#endif
   // Add a clear end-of-result marker and a newline for better readability
   printk("--- End of Classification ---\n\n");

        // k_msleep(2000); // Removed to make the system responsive
    }
}