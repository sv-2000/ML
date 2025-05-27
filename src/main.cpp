#include <version.h>
#if (KERNEL_VERSION_MAJOR > 3) || ((KERNEL_VERSION_MAJOR == 3) && (KERNEL_VERSION_MINOR >= 1))
#include <zephyr/kernel.h>
#else
#include <zephyr.h>
#endif
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include <hal/nrf_clock.h>

#ifdef EI_NORDIC
#include <nrfx_clock.h>
#endif

static const float mnist_image[] = {
    // Example 28x28 grayscale image, normalized between 0.0 - 1.0
    // Replace this with actual MNIST test image (flattened into 784 floats)
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, ... 784 floats 
};

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

    printk("Edge Impulse MNIST digit classification (Zephyr)\n");

    if (sizeof(mnist_image) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printk("Invalid input size. Expected %d floats, got %u\n",
               EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(mnist_image) / sizeof(float));
        return 1;
    }

    ei_impulse_result_t result = { 0 };

    while (1) {
        signal_t features_signal;
        features_signal.total_length = sizeof(mnist_image) / sizeof(mnist_image[0]);
        features_signal.get_data = &raw_feature_get_data;

        EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
        printk("run_classifier returned: %d\n", res);

        if (res != 0) return 1;

        printk("Predictions (DSP: %d ms, Classification: %d ms, Anomaly: %d ms):\n",
               result.timing.dsp, result.timing.classification, result.timing.anomaly);

               for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
                auto bb = result.bounding_boxes[i];
                if (bb.value == 0) continue; // skip low confidence
                printk("Found object: '%s' (%.2f%%) at [%d,%d,%d,%d]\n",
                       bb.label, bb.value * 100,
                       bb.x, bb.y, bb.width, bb.height);
            }
            

#if EI_CLASSIFIER_HAS_ANOMALY == 1
        printk("    anomaly score: %.3f\n", result.anomaly);
#endif

        k_msleep(2000);
    }
}