std_msgs/ColorRGBA color    # RGB values [0-1] for all LEDs
int32 start_led             # First LED in the stripe where LEDs will blink
int32 end_led               # Last LED in the stripe where LEDs will blink
int32 num_leds              # Number of LEDs relative to start_led (if != 0 it overrides end_led)
