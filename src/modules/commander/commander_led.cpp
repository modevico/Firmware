#include <stdint.h>
#include <px4_log.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

bool static _led_state[2] = { false , false };

__EXPORT void led_init()
{
	PX4_DEBUG("LED_INIT");
}

__EXPORT void led_on(int led)
{
	if (led == 1 || led == 0)
	{
		PX4_DEBUG("LED%d_ON", led);
		_led_state[led] = true;
	}
}

__EXPORT void led_off(int led)
{
	if (led == 1 || led == 0)
	{
		PX4_DEBUG("LED%d_OFF", led);
		_led_state[led] = false;
	}
}

__EXPORT void led_toggle(int led)
{
	if (led == 1 || led == 0)
	{
		_led_state[led] = !_led_state[led];
		PX4_DEBUG("LED%d_TOGGLE: %s", led, _led_state[led] ? "ON" : "OFF");

	}
}

