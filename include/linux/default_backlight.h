#ifndef _DEFAULT_BACKLIGHT_H_
#define _DEFAULT_BACKLIGHT_H_

#include <linux/backlight.h>

extern void set_brightness_of_default_backlight(unsigned int brightness);
extern void register_default_backlight(struct backlight_device *bd);
extern void unregister_default_backlight(struct backlight_device *bd);

#endif /* _DEFAULT_BACKLIGHT_H_ */

