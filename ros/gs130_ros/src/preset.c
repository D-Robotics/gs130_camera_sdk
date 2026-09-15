#include <string.h>

#include "gs130.h"
#include "gs130_define.h"

#include "gs130_ros/preset.h"

int gs130_ros_preset(const char *platform, const char *device, int mode,
                     int w, int h, int fps, int odr, gs130_config_t *out)
{
    const gs130_camera_mode_t camera_mode = (gs130_camera_mode_t)mode;

    if (out == NULL || platform == NULL || device == NULL) return -1;
    if (strcmp(platform, "RDKX5") != 0) return -1;

    if (strcmp(device, "GS130WI") == 0) {
        const gs130_config_t cfg = GS130_CONFIG_RDKX5_GS130WI(camera_mode, w, h, fps, odr);
        *out = cfg;
        return 0;
    }
    if (strcmp(device, "GS130W") == 0) {
        const gs130_config_t cfg = GS130_CONFIG_RDKX5_GS130W(camera_mode, w, h, fps, odr);
        *out = cfg;
        return 0;
    }
    return -1;
}
