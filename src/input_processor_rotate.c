/*
 * Small-angle XY correction for Crosses trackballs.
 *
 * ZMK input processors see one axis event at a time. This processor never stops
 * the current axis event. Instead, it scales the current axis in place and
 * injects the small cross-axis correction needed for a -15 degree rotation.
 */

#define DT_DRV_COMPAT zmk_input_processor_rotate

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>

#include <zephyr/dt-bindings/input/input-event-codes.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define SCALE 1000

struct rotate_config {
    int16_t cos_milli;
    int16_t sin_milli;
};

struct rotate_data {
    int32_t x_remainder;
    int32_t y_remainder;
    bool injecting;
};

static int32_t scale_with_remainder(int32_t value, int32_t *remainder) {
    int32_t value_with_remainder = value + *remainder;
    int32_t scaled = value_with_remainder / SCALE;

    *remainder = value_with_remainder - (scaled * SCALE);

    return scaled;
}

static int rotate_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                               uint32_t param2, struct zmk_input_processor_state *state) {
    const struct rotate_config *cfg = dev->config;
    struct rotate_data *data = dev->data;

    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    if (data->injecting || event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /*
     * Negative-angle correction:
     *   x' =  x*cos + y*sin_magnitude
     *   y' = -x*sin_magnitude + y*cos
     *
     * X-only movement must keep its X event alive. The previous pair-buffered
     * version waited for Y and swallowed horizontal-only motion on this sensor.
     */
    if (event->code == INPUT_REL_X) {
        int32_t x = event->value;
        int32_t out_x = scale_with_remainder(x * cfg->cos_milli, &data->x_remainder);
        int32_t out_y = scale_with_remainder(-x * cfg->sin_milli, &data->y_remainder);

        data->injecting = true;
        input_report_rel(event->dev, INPUT_REL_Y, out_y, false, K_NO_WAIT);
        data->injecting = false;

        event->value = out_x;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_Y) {
        int32_t y = event->value;
        int32_t out_x = scale_with_remainder(y * cfg->sin_milli, &data->x_remainder);
        int32_t out_y = scale_with_remainder(y * cfg->cos_milli, &data->y_remainder);

        data->injecting = true;
        input_report_rel(event->dev, INPUT_REL_X, out_x, false, K_NO_WAIT);
        data->injecting = false;

        event->value = out_y;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api rotate_driver_api = {
    .handle_event = rotate_handle_event,
};

#define ROTATE_INST(n)                                                                            \
    static const struct rotate_config rotate_config_##n = {                                       \
        .cos_milli = DT_INST_PROP(n, cos_milli),                                                  \
        .sin_milli = DT_INST_PROP(n, sin_milli),                                                  \
    };                                                                                            \
    static struct rotate_data rotate_data_##n = {};                                               \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &rotate_data_##n, &rotate_config_##n, POST_KERNEL,       \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &rotate_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ROTATE_INST)
