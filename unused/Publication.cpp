#include "Publication.hpp"
#include <uORB/uORB.h>
#include <math_defines.h>
#include "log_printf.h"

namespace uORB
{

PublicationBase::PublicationBase(const struct orb_metadata *meta,
				 int priority) :
	_meta(meta),
	_priority(priority),
	_instance(),
	_handle(nullptr)
{
}

void PublicationBase::update(void *data)
{
	if (_handle != nullptr) {
		bool ret = orb_publish((char*)getMeta(), getHandle(), data);

		if (ret != true) { warnx("publish fail"); }

	} else {
		orb_advert_t handle;

                handle = orb_advertise((char*)getMeta(), data);

		if (handle != nullptr) {
			setHandle(handle);

		} else {
			warnx("advert fail");
		}
	}
}

PublicationBase::~PublicationBase()
{
	orb_unadvertise(getHandle());
}

PublicationNode::PublicationNode(const struct orb_metadata *meta,
				 int priority,
				 List<PublicationNode *> *list) :
	PublicationBase(meta, priority)
{
	if (list != nullptr) { list->add(this); }
}

// explicit template instantiation
template class Publication<vehicle_attitude_s>;
template class Publication<vehicle_local_position_s>;
template class Publication<vehicle_global_position_s>;
template class Publication<debug_key_value_s>;
template class Publication<actuator_controls_s>;
template class Publication<vehicle_global_velocity_setpoint_s>;
template class Publication<vehicle_attitude_setpoint_s>;
template class Publication<vehicle_rates_setpoint_s>;
template class Publication<actuator_outputs_s>;
template class Publication<actuator_direct_s>;
template class Publication<tecs_status_s>;
template class Publication<rc_channels_s>;
template class Publication<filtered_bottom_flow_s>;
template class Publication<ekf2_innovations_s>;

}
