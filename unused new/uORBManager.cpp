#include <stdarg.h>
#include <errno.h>
#include "uORBUtils.hpp"
#include "uORBManager.hpp"
#include <math_defines.h>
#include <log_printf.h>
//#include "uORBDevices.hpp"


//=========================  Static initializations =================
uORB::Manager *uORB::Manager::_Instance = nullptr;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool uORB::Manager::initialize()
{
	if (_Instance == nullptr) {
		_Instance = new uORB::Manager();
	}

	return _Instance != nullptr;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
uORB::Manager::Manager()
	: _comm_channel(nullptr)
{
	for (int i = 0; i < Flavor_count; ++i) {
		_device_masters[i] = nullptr;
	}
}

uORB::Manager::~Manager()
{
	for (int i = 0; i < Flavor_count; ++i) {
		if (_device_masters[i]) {
			delete _device_masters[i];
		}
	}
}

uORB::DeviceMaster *uORB::Manager::get_device_master(Flavor flavor)
{
	if (!_device_masters[flavor]) {
		_device_masters[flavor] = new DeviceMaster(flavor);

		if (_device_masters[flavor]) {
			int ret = _device_masters[flavor]->init();

			if (ret != OK) {
				ERR("Initialization of DeviceMaster failed (%i)", ret);
				errno = -ret;
				delete _device_masters[flavor];
				_device_masters[flavor] = nullptr;
			}

		} else {
			ERR("Failed to allocate DeviceMaster");
			errno = 22;
		}
	}

	return _device_masters[flavor];
}

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance)
{
	/*
	 * Generate the path to the node and try to open it.
	 */
	char path[orb_maxpath];
	int inst = instance;
	int ret = uORB::Utils::node_mkpath(path, PUBSUB, meta, &inst);

	if (ret != OK) {
		errno = -ret;
		return ERROR;
	}
	ret = px4_access(path, F_OK);

	if (ret == -1 && meta != nullptr && !_remote_topics.empty()) {
		ret = (_remote_topics.find(meta->o_name) != _remote_topics.end()) ? OK : ERROR;
	}

	return ret;
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
		int priority, unsigned int queue_size)
{
	int result, fd;
	orb_advert_t advertiser;

	/* open the node as an advertiser */
	fd = node_open(PUBSUB, meta, data, true, instance, priority);

	if (fd == ERROR) {
		WARN("node_open as advertiser failed.");
		return nullptr;
	}

	/* Set the queue size. This must be done before the first publication; thus it fails if
	 * this is not the first advertiser.
	 */
	result = px4_ioctl(fd, ORBIOCSETQUEUESIZE, (unsigned long)queue_size);

	if (result < 0 && queue_size > 1) {
		WARN("orb_advertise_multi: failed to set queue size");
	}

	/* get the advertiser handle and close the node */
	result = px4_ioctl(fd, ORBIOCGADVERTISER, (unsigned long)&advertiser);
	px4_close(fd);

	if (result == ERROR) {
		WARN("px4_ioctl ORBIOCGADVERTISER  failed. fd = %d", fd);
		return nullptr;
	}

	//For remote systems call over and inform them
	uORB::DeviceNode::topic_advertised(meta, priority);

	/* the advertiser must perform an initial publish to initialise the object */
	result = orb_publish(meta, advertiser, data);

	if (result == ERROR) {
		WARN("orb_publish failed");
		return nullptr;
	}

	return advertiser;
}

int uORB::Manager::orb_unadvertise(orb_advert_t handle)
{
	return uORB::DeviceNode::unadvertise(handle);
}

int uORB::Manager::orb_subscribe(const struct orb_metadata *meta)
{
	return node_open(PUBSUB, meta, nullptr, false);
}

int uORB::Manager::orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	int inst = instance;
	return node_open(PUBSUB, meta, nullptr, false, &inst);
}

int uORB::Manager::orb_unsubscribe(int fd)
{
	return px4_close(fd);
}

int uORB::Manager::orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
	return uORB::DeviceNode::publish(meta, handle, data);
}

int uORB::Manager::orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
	int ret;

	ret = px4_read(handle, buffer, meta->o_size);

	if (ret < 0) {
		return ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	return OK;
}

int uORB::Manager::orb_check(int handle, bool *updated)
{
	/* Set to false here so that if `px4_ioctl` fails to false. */
	*updated = false;
	return px4_ioctl(handle, ORBIOCUPDATED, (unsigned long)(uintptr_t)updated);
}

int uORB::Manager::orb_stat(int handle, uint64_t *time)
{
	return px4_ioctl(handle, ORBIOCLASTUPDATE, (unsigned long)(uintptr_t)time);
}

int uORB::Manager::orb_priority(int handle, int32_t *priority)
{
	return px4_ioctl(handle, ORBIOCGPRIORITY, (unsigned long)(uintptr_t)priority);
}

int uORB::Manager::orb_set_interval(int handle, unsigned interval)
{
	return px4_ioctl(handle, ORBIOCSETINTERVAL, interval * 1000);
}

int uORB::Manager::orb_get_interval(int handle, unsigned *interval)
{
	ASSERT(interval);
	int ret = px4_ioctl(handle, ORBIOCGETINTERVAL, (unsigned long)interval);
	*interval /= 1000;
	return ret;
}


int uORB::Manager::node_advertise
(
	const struct orb_metadata *meta,
	int *instance,
	int priority
)
{
	int fd = -1;
	int ret = ERROR;

	/* fill advertiser data */
	const struct orb_advertdata adv = { meta, instance, priority };

	/* open the control device */
	fd = px4_open(TOPIC_MASTER_DEVICE_PATH, 0);

	if (fd < 0) {
		goto out;
	}

	/* advertise the object */
	ret = px4_ioctl(fd, ORBIOCADVERTISE, (unsigned long)(uintptr_t)&adv);

	/* it's OK if it already exists */
	if ((OK != ret) && (EEXIST == errno)) {
		ret = OK;
	}

out:

	if (fd >= 0) {
		px4_close(fd);
	}

	return ret;
}

int uORB::Manager::node_open
(
	Flavor f,
	const struct orb_metadata *meta,
	const void *data,
	bool advertiser,
	int *instance,
	int priority
)
{
	char path[orb_maxpath];
	int fd = -1, ret;

	/*
	 * If meta is null, the object was not defined, i.e. it is not
	 * known to the system.  We can't advertise/subscribe such a thing.
	 */
	if (nullptr == meta) {
		errno = ENOENT;
		return ERROR;
	}

	/*
	 * Advertiser must publish an initial value.
	 */
	if (advertiser && (data == nullptr)) {
		errno = EINVAL;
		return ERROR;
	}

	/* if we have an instance and are an advertiser, we will generate a new node and set the instance,
	 * so we do not need to open here */
	if (!instance || !advertiser) {
		/*
		 * Generate the path to the node and try to open it.
		 */
		ret = uORB::Utils::node_mkpath(path, f, meta, instance);

		if (ret != OK) {
			errno = -ret;
			return ERROR;
		}

		/* open the path as either the advertiser or the subscriber */
		fd = px4_open(path, advertiser ? PX4_F_WRONLY : PX4_F_RDONLY);

	} else {
		*instance = 0;
	}

	/* we may need to advertise the node... */
	if (fd < 0) {

		/* try to create the node */
		ret = node_advertise(meta, instance, priority);

		if (ret == OK) {
			/* update the path, as it might have been updated during the node_advertise call */
			ret = uORB::Utils::node_mkpath(path, f, meta, instance);

			if (ret != OK) {
				errno = -ret;
				return ERROR;
			}
		}

		/* on success, try the open again */
		if (ret == OK) {
			fd = px4_open(path, (advertiser) ? PX4_F_WRONLY : PX4_F_RDONLY);
		}
	}

	if (fd < 0) {
		errno = EIO;
		return ERROR;
	}

	/* everything has been OK, we can return the handle now */
	return fd;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::Manager::set_uorb_communicator(uORBCommunicator::IChannel *channel)
{
	_comm_channel = channel;

	if (_comm_channel != nullptr) {
		_comm_channel->register_handler(this);
	}
}

uORBCommunicator::IChannel *uORB::Manager::get_uorb_communicator()
{
	return _comm_channel;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_remote_topic(const char *topic_name, bool isAdvertisement)
{
	int16_t rc = 0;

	if (isAdvertisement) {
		_remote_topics.insert(topic_name);

	} else {
		_remote_topics.erase(topic_name);
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_add_subscription(const char *messageName,
		int32_t msgRateInHz)
{
	DEBUG("[posix-uORB::Manager::process_add_subscription(%d)] entering Manager_process_add_subscription: name: %s",
		  __LINE__, messageName);
	int16_t rc = 0;
	_remote_subscriber_topics.insert(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, PUBSUB, messageName);
	DeviceMaster *device_master = get_device_master(PUBSUB);

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		if (node == nullptr) {
			DEBUG("[posix-uORB::Manager::process_add_subscription(%d)]DeviceNode(%s) not created yet",
				  __LINE__, messageName);

		} else {
			// node is present.
			node->process_add_subscription(msgRateInHz);
		}

	} else {
		rc = -1;
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_remove_subscription(
	const char *messageName)
{
	int16_t rc = -1;
	_remote_subscriber_topics.erase(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, PUBSUB, messageName);
	DeviceMaster *device_master = get_device_master(PUBSUB);

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			DEBUG("[posix-uORB::Manager::process_remove_subscription(%d)]Error No existing subscriber found for message: [%s]",
				  __LINE__, messageName);

		} else {
			// node is present.
			node->process_remove_subscription();
			rc = 0;
		}
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_received_message(const char *messageName,
		int32_t length, uint8_t *data)
{
	int16_t rc = -1;
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, PUBSUB, messageName);
	DeviceMaster *device_master = get_device_master(PUBSUB);

	if (ret == OK && device_master) {
		uORB::DeviceNode *node = device_master->getDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			DEBUG("[uORB::Manager::process_received_message(%d)]Error No existing subscriber found for message: [%s] nodepath:[%s]",
				  __LINE__, messageName, nodepath);

		} else {
			// node is present.
			node->process_received_message(length, data);
			rc = 0;
		}
	}

	return rc;
}

bool uORB::Manager::is_remote_subscriber_present(const char *messageName)
{
	return _remote_subscriber_topics.find(messageName);
}