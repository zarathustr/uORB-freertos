#include "Subscription.hpp"
#include <uORB/uORB.h>
#include "log_printf.h"
#include <math_defines.h>

namespace uORB
{

SubscriptionBase::SubscriptionBase(const struct orb_metadata *meta,
				   unsigned interval, unsigned instance) :
	_meta(meta),
	_instance(instance),
	_handle()
{
	if (_instance > 0) {
		_handle =  orb_subscribe(
				   (char*)getMeta());
	} else {
		_handle =  orb_subscribe((char*)getMeta());
	}

	if (_handle < 0) { ERR("sub failed"); }

	if (interval > 0) {
		//orb_set_interval(getHandle(), interval);
	}
}

bool SubscriptionBase::updated()
{
	bool isUpdated = false;
	bool ret = orb_check(_handle, &isUpdated);

	if (ret != true) { ERR("orb check failed"); }

	return isUpdated;
}

void SubscriptionBase::update(void *data)
{
	if (updated()) {
		bool ret = orb_copy((char*)_meta, _handle, data);

		if (ret != true) { ERR("orb copy failed"); }
	}
}

SubscriptionBase::~SubscriptionBase()
{
	bool ret = orb_unsubscribe(_handle);

	if (ret != true) { ERR("orb unsubscribe failed"); }
}

template <class T>
Subscription<T>::Subscription(const struct orb_metadata *meta,
			      unsigned interval,
			      int instance,
			      List<SubscriptionNode *> *list) :
	SubscriptionNode(meta, interval, instance, list),
	_data() // initialize data structure to zero
{
}

template <class T>
Subscription<T>::Subscription(const Subscription &other) :
	SubscriptionNode(other._meta, other.getInterval(), other._instance, nullptr),
	_data() // initialize data structure to zero
{
}

template <class T>
Subscription<T>::~Subscription()
{
}

template <class T>
void Subscription<T>::update()
{
	SubscriptionBase::update((void *)(&_data));
}

template <class T>
bool Subscription<T>::check_updated()
{
	return SubscriptionBase::updated();
}

template <class T>
const T &Subscription<T>::get() { return _data; }

template class Subscription<parameter_update_s>;
template class Subscription<actuator_controls_s>;
template class Subscription<vehicle_gps_position_s>;
template class Subscription<satellite_info_s>;
template class Subscription<sensor_combined_s>;
template class Subscription<hil_sensor_s>;
template class Subscription<vehicle_attitude_s>;
template class Subscription<vehicle_global_position_s>;
template class Subscription<position_setpoint_triplet_s>;
template class Subscription<vehicle_status_s>;
template class Subscription<manual_control_setpoint_s>;
template class Subscription<mavlink_log_s>;
template class Subscription<log_message_s>;
template class Subscription<vehicle_local_position_setpoint_s>;
template class Subscription<vehicle_local_position_s>;
template class Subscription<vehicle_attitude_setpoint_s>;
template class Subscription<vehicle_rates_setpoint_s>;
template class Subscription<rc_channels_s>;
template class Subscription<vehicle_control_mode_s>;
template class Subscription<actuator_armed_s>;
template class Subscription<battery_status_s>;
template class Subscription<home_position_s>;
template class Subscription<optical_flow_s>;
template class Subscription<distance_sensor_s>;
template class Subscription<att_pos_mocap_s>;
template class Subscription<control_state_s>;
template class Subscription<vehicle_land_detected_s>;

} // namespace uORB
