/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * The software is provided "AS IS", without warranty of any kind.
 */

/* Includes ---------------------------------------------------------------- */
#include <TechX3-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "camera.h"
#include "gc2145.h"
#include <ea_malloc.h>
#include <string.h>

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_RAW_FRAME_BYTE_SIZE             2

#define ALIGN_PTR(p,a)   ((p & (a-1)) ?(((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)

/* Edge Impulse ------------------------------------------------------------- */

typedef struct {
    size_t width;
    size_t height;
} ei_device_resize_resolutions_t;

int ei_get_serial_available(void) {
    return Serial.available();
}

char ei_get_serial_byte(void) {
    return Serial.read();
}

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;
static bool is_ll_initialised = false;

GC2145 galaxyCore;
Camera cam(galaxyCore);
FrameBuffer fb;

// Keep the original allocation pointer so it can be freed safely.
static uint8_t *ei_camera_capture_alloc = NULL;
static uint8_t *ei_camera_capture_out = NULL;

static void ei_camera_release_capture_buffer(void) {
    if (ei_camera_capture_alloc != NULL) {
        ea_free(ei_camera_capture_alloc);
        ei_camera_capture_alloc = NULL;
    }

    ei_camera_capture_out = NULL;
}
static uint8_t *ei_camera_frame_mem;
static uint8_t *ei_camera_frame_buffer;

/* UART victim-message settings ------------------------------------------- */
// Only detections at or above this confidence are sent to the Pico.
static const float UART_CONFIDENCE_THRESHOLD = 0.35f;

// Prevent repeated messages while the same victim remains in view.
// After 3 consecutive frames with no recognised victim, the system rearms.
static char lastSentMessage[16] = "";
static uint8_t clearFrameCount = 0;

/* Function definitions ---------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height,
                                uint32_t *resize_col_sz, uint32_t *resize_row_sz,
                                bool *do_resize);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len);

/* Compare labels without being sensitive to upper/lowercase ASCII. */
bool labelEqualsIgnoreCase(const char *a, const char *b) {
    while (*a && *b) {
        char ca = *a++;
        char cb = *b++;

        if (ca >= 'A' && ca <= 'Z') ca = ca - 'A' + 'a';
        if (cb >= 'A' && cb <= 'Z') cb = cb - 'A' + 'a';

        if (ca != cb) return false;
    }
    return (*a == '\0' && *b == '\0');
}

/* Convert the Edge Impulse model label into the message expected by Pico.
 * Expected model labels from this project: Psi, phi, Omega, Nothing.
 */
const char *getVictimMessage(const char *label) {
    if (labelEqualsIgnoreCase(label, "Psi") || strcmp(label, "Ψ") == 0) {
        return "STABLE";
    }

    if (labelEqualsIgnoreCase(label, "phi") || strcmp(label, "Φ") == 0) {
        return "HARMED";
    }

    if (labelEqualsIgnoreCase(label, "Omega") || strcmp(label, "Ω") == 0) {
        return "UNHARMED";
    }

    // "Nothing" and all unknown labels are ignored.
    return NULL;
}

/**
 * @brief Arduino setup function
 */
void setup() {
    Serial.begin(115200);

    // UART from Nicla Vision to Pico.
    // Nicla Serial1 TX is routed to the board's UART TX pin (PA9).
    Serial1.begin(115200);

    // Do not wait indefinitely for USB Serial to connect.
    // This allows the Nicla to start even when Serial Monitor is closed.
    unsigned long serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart < 3000)) {
        delay(10);
    }

    Serial.println("Edge Impulse Inferencing Demo");
    Serial.println("UART transmitter ready (115200 baud)");

    // Initialise M4 RAM.
    malloc_addblock((void*)0x30000000, 288 * 1024);

    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }
}

/**
 * @brief Get data and run inferencing
 */
void loop() {
    ei_printf("\nStarting inferencing in 2 seconds...\n");

    if (ei_sleep(2000) != EI_IMPULSE_OK) {
        return;
    }

    ei_printf("Taking photo...\n");

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH,
                          (size_t)EI_CLASSIFIER_INPUT_HEIGHT, NULL) == false) {
        ei_printf("Failed to capture image\r\n");
        return;
    }

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

    // The classifier has finished reading the image; release it now.
    ei_camera_release_capture_buffer();

    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");

    const char *detectedMessage = NULL;
    float bestConfidence = UART_CONFIDENCE_THRESHOLD;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];

        if (bb.value == 0) {
            continue;
        }

        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);

        const char *message = getVictimMessage(bb.label);

        if (message != NULL && bb.value >= bestConfidence) {
            bestConfidence = bb.value;
            detectedMessage = message;
        }
    }

    if (detectedMessage != NULL) {
        clearFrameCount = 0;

        // Send once for a continuous detection of the same class.
        // A change to a different recognised class sends a new message.
        if (strcmp(lastSentMessage, detectedMessage) != 0) {
            Serial1.println(detectedMessage);

            Serial.print("UART SENT: ");
            Serial.println(detectedMessage);

            strncpy(lastSentMessage, detectedMessage, sizeof(lastSentMessage) - 1);
            lastSentMessage[sizeof(lastSentMessage) - 1] = '\0';
        }
    }
    else {
        // Require 3 consecutive frames without a recognised victim
        // before the same class can trigger again.
        if (clearFrameCount < 3) {
            clearFrameCount++;
        }

        if (clearFrameCount >= 3) {
            lastSentMessage[0] = '\0';
        }
    }

#else
    ei_printf("This model is not configured for object detection.\r\n");
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    }
#endif
}

/**
 * @brief Setup image sensor
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    if (is_ll_initialised == false) {
        if (!cam.begin(CAMERA_R320x240, CAMERA_RGB565, -1)) {
            ei_printf("ERR: Failed to initialise camera\r\n");
            return false;
        }

        ei_camera_frame_mem = (uint8_t *)ei_malloc(
            EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
            EI_CAMERA_RAW_FRAME_BYTE_SIZE + 32);

        if (ei_camera_frame_mem == NULL) {
            ei_printf("failed to create ei_camera_frame_mem\r\n");
            return false;
        }

        ei_camera_frame_buffer = (uint8_t *)ALIGN_PTR((uintptr_t)ei_camera_frame_mem, 32);
        fb.setBuffer(ei_camera_frame_buffer);
        is_initialised = true;
    }

    return true;
}

/**
 * @brief Stop streaming of sensor data
 */
void ei_camera_deinit(void) {
    ei_camera_release_capture_buffer();
    ei_free(ei_camera_frame_mem);
    ei_camera_frame_mem = NULL;
    ei_camera_frame_buffer = NULL;
    is_initialised = false;
}

/**
 * @brief Capture, rescale and crop image
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    (void)out_buf; // This camera implementation uses ei_camera_capture_out.

    bool do_resize = false;

    // Clear any previous capture before allocating a new one.
    ei_camera_release_capture_buffer();

    // Allocate RGB888 storage. Keep the original pointer for ea_free().
    ei_camera_capture_alloc = (uint8_t *)ea_malloc(
        EI_CAMERA_RAW_FRAME_BUFFER_COLS *
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3 + 32
    );

    if (ei_camera_capture_alloc == NULL) {
        ei_printf("ERR: Failed to allocate image capture buffer\r\n");
        return false;
    }

    // Use an aligned working pointer, but free only ei_camera_capture_alloc.
    ei_camera_capture_out = (uint8_t *)ALIGN_PTR(
        (uintptr_t)ei_camera_capture_alloc, 32
    );

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        ei_camera_release_capture_buffer();
        return false;
    }

    int snapshot_response = cam.grabFrame(fb, 100);
    if (snapshot_response != 0) {
        ei_printf("ERR: Failed to get snapshot (%d)\r\n", snapshot_response);
        ei_camera_release_capture_buffer();
        return false;
    }

    bool converted = RBG565ToRGB888(
        ei_camera_frame_buffer,
        ei_camera_capture_out,
        cam.frameSize()
    );

    if (!converted) {
        ei_printf("ERR: Conversion failed\r\n");
        ei_camera_release_capture_buffer();
        return false;
    }

    uint32_t resize_col_sz;
    uint32_t resize_row_sz;

    int res = calculate_resize_dimensions(
        img_width,
        img_height,
        &resize_col_sz,
        &resize_row_sz,
        &do_resize
    );

    if (res) {
        ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
        ei_camera_release_capture_buffer();
        return false;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            ei_camera_capture_out,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            ei_camera_capture_out,
            resize_col_sz,
            resize_row_sz
        );
    }

    // IMPORTANT: Keep this buffer alive. run_classifier() reads it through
    // ei_camera_get_data(). It is released immediately after run_classifier().
    return true;
}

/**
 * @brief Convert RGB565 data to RGB888
 */
bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len) {
    uint8_t hb, lb;
    uint32_t pix_count = src_len / 2;

    for (uint32_t i = 0; i < pix_count; i++) {
        hb = *src_buf++;
        lb = *src_buf++;

        *dst_buf++ = hb & 0xF8;
        *dst_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
        *dst_buf++ = (lb & 0x1F) << 3;
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] =
            (ei_camera_capture_out[pixel_ix] << 16) +
            (ei_camera_capture_out[pixel_ix + 1] << 8) +
            ei_camera_capture_out[pixel_ix + 2];

        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}

/**
 * @brief Determine whether to resize and to which dimensions
 */
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height,
                                uint32_t *resize_col_sz, uint32_t *resize_row_sz,
                                bool *do_resize) {
    size_t list_size = 5;
    const ei_device_resize_resolutions_t list[list_size] = {
        {64, 64},
        {96, 96},
        {160, 120},
        {160, 160},
        {320, 240},
    };

    *resize_col_sz = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
    *resize_row_sz = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    *do_resize = false;

    for (size_t ix = 0; ix < list_size; ix++) {
        if ((out_width <= list[ix].width) && (out_height <= list[ix].height)) {
            *resize_col_sz = list[ix].width;
            *resize_row_sz = list[ix].height;
            *do_resize = true;
            break;
        }
    }

    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif