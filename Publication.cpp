#include "Publication.hpp"
#include "topics/actuator_controls.h"
#include "topics/actuator_direct.h"
#include "topics/actuator_outputs.h"
#include "topics/debug_key_value.h"
#include "topics/ekf2_innovations.h"
#include "topics/filtered_bottom_flow.h"
#include "topics/rc_channels.h"
#include "topics/tecs_status.h"
#include "topics/vehicle_attitude.h"
#include "topics/vehicle_attitude_setpoint.h"
#include "topics/vehicle_global_position.h"
#include "topics/vehicle_global_velocity_setpoint.h"
#include "topics/vehicle_local_position.h"
#include "topics/vehicle_rates_setpoint.h"

#include <math_defines.h>
#include <log_printf.h>

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
		int ret = orb_publish(getMeta(), getHandle(), data);

		if (ret != OK) { warnx("publish fail"); }

	} else {
		orb_advert_t handle;

		if (_priority > 0) {
			handle = orb_advertise_multi(
					 getMeta(), data,
					 &_instance, _priority);

		} else {
			handle = orb_advertise(getMeta(), data);
		}

		if (int64_t(handle) != ERROR_CODE) {
			setHandle(handle);

		} else {
			warnx("advert fail");
		}
	}
}

PublicationBase::~PublicationBase()
{
    orb_advert_t handle=getHandle();
    orb_unadvertise(handle);
}

PublicationNode::PublicationNode(const struct orb_metadata *meta,
				 int priority,
				 List<PublicationNode *> *list) :
	PublicationBase(meta, priority)
{
	if (list != nullptr) { list->add(this); }
}

// explicit template instantiation
template class Publication<actuator_controls_s>;
template class Publication<actuator_direct_s>;
template class Publication<actuator_outputs_s>;
template class Publication<debug_key_value_s>;
template class Publication<ekf2_innovations_s>;
template class Publication<filtered_bottom_flow_s>;
template class Publication<rc_channels_s>;
template class Publication<tecs_status_s>;
template class Publication<vehicle_attitude_s>;
template class Publication<vehicle_attitude_setpoint_s>;
template class Publication<vehicle_global_position_s>;
template class Publication<vehicle_global_velocity_setpoint_s>;
template class Publication<vehicle_local_position_s>;
template class Publication<vehicle_rates_setpoint_s>;

}
