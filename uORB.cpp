#include "uORB.h"
#include <drv_hrt.h>
#include "topics/parameter_update.h"
#include "uORBRam.h"
#include "uORBHelper.h"
#include <cstdlib>
#include <cstring>
#include <poll.h>

#if defined(STM32F4)

#define USE_UORB_RAM

#endif

#pragma default_variable_attributes = @ ".ccmram"

  ORBData               *orb_data[total_uorb_num];

  pollfd_struct_t       poll_queue[UORB_MAX_POLL];
  
#pragma default_variable_attributes =
 
  

  
#ifdef          USE_UORB_RAM
  
#pragma location = ".ccmram"
  
uORBRam         ram_ccm;

#endif
  



//flag that indicates if we have entered the operating system
bool orb_in_os = false;

int orb_lock(void)
{
    if(orb_in_os)
    {
        return osEnterCritical();
    }
    
    return -1;
}

void orb_unlock(int state)
{
    if(orb_in_os && state != -1)
    {
        osExitCritical(state);
    }
}


void orb_sem_lock(ORBData *_orb)
{
    if(_orb && orb_in_os)
    {
        int32_t res = osSemaphoreWait(_orb->sem, 0xFFFFFFFF);
        switch (res) {
            case osOK:
                _orb->sem_taken = true;
                break;
            case osErrorResource:
                _orb->sem_taken = false;
                break;
            case osErrorParameter:
                _orb->sem_taken = false;
                break;
            default:
                _orb->sem_taken = false;
                break;
       }
    }
}

void orb_sem_unlock(ORBData *_orb)
{
    if(_orb && orb_in_os)
    {
        int32_t res = osSemaphoreRelease (_orb->sem);
        if (res == osOK) {
            _orb->sem_taken = false;
        }
    }
}




orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data)
{   
    orb_advert_t advert = nullptr;
    
    //get the serial number of the current orb
    int serial = get_orb_serial(meta->o_name);
    
    //we get an effective orb
    if(serial != -1)
    {
        //obtain the instance according to whether the orb is a multi-prio orb
        int instance;
        if(is_orb_multi(serial))
            instance = ORB_MULTI_MAX_INSTANCES - 1;
        else
            instance = 0;
        
        orb_sem_lock(&orb_data[serial][instance]);
        
        //reset the published flag to false to prevent subscribing and checking
        orb_data[serial][instance].published = false;
        
        int atomic_state = orb_lock();
        
        //copy the data
        std::memcpy(orb_data[serial][instance].data, data, meta->o_size);
        
        orb_unlock(atomic_state);
        
        //copy the current serial number into the ORBData struct
        orb_data[serial][instance].serial = serial;
        
        //name the advert as the pointer of the orb's internal data
        advert = (void*)(&orb_data[serial][instance]);
        
        //get the current system time in us
        orb_data[serial][instance].last_updated_time = hrt_absolute_time();
        
        //update the published flag to true
        orb_data[serial][instance].published = true;
        
        int task_id = (int)osThreadGetId();
        for(int i = 0; i < UORB_MAX_SUB; ++i)
        {
            orb_data[serial][instance].authority_list[i] = -1;
        }
        
        orb_sem_unlock(&orb_data[serial][instance]);
    }
    
    return advert;
}




orb_advert_t orb_advertise_queue(const struct orb_metadata *meta, const void *data, unsigned int queue_size)
{
	return orb_advertise_multi_queue(meta, data, nullptr, ORB_PRIO_MIN, queue_size);
}


//Unqueued version
orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
				 int priority)
{
    
    orb_advert_t advert = nullptr;
    
    //get the serial number of the current orb
    int serial = get_orb_serial(meta->o_name);
    
    //we get an effective and multi-prio orb
    if(serial != -1)
    {   
        //search for existing orb
        int inst = get_orb_instance_according_to_priority(priority);
        
        if(!is_orb_multi(serial))
            inst = 0;
        
        if(instance != nullptr)
            *instance = inst;
        
        //If we find it, copy the data
        if(inst != -1)
        {
            orb_sem_lock(&orb_data[serial][inst]);
            
            //reset the published flag to false to prevent subscribing and checking
            orb_data[serial][inst].published = false;
            
            int atomic_state = orb_lock();
            
            //copy the data
            std::memcpy(orb_data[serial][inst].data, data, meta->o_size);
            
            orb_unlock(atomic_state);
            
            //copy the serial number
            orb_data[serial][inst].serial = serial;
        
            //name the advert as the pointer of the orb's internal data
            advert = (void*)(&orb_data[serial][inst]);
            
            //get the current system time in us
            orb_data[serial][inst].last_updated_time = hrt_absolute_time();
            
            //update the published flag to true
            orb_data[serial][inst].published = true;
            
            int task_id = (int)osThreadGetId();
            for(int i = 0; i < UORB_MAX_SUB; ++i)
            {
                orb_data[serial][inst].authority_list[i] = -1;
            }
            
            orb_sem_unlock(&orb_data[serial][inst]);
        }
    }
    
    return advert;
}

//Queued version
orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
				 int priority,size_t queue_size)
{   
    orb_advert_t advert = nullptr;
    
    //get the serial number
    int serial = get_orb_serial(meta->o_name);
    
    if(serial != -1)
    {   
        //search for existing orb
        int inst = get_orb_instance_according_to_priority(priority);
        if(instance != nullptr)
            *instance = inst;
        
        //If we find it, allocate data region for copy
        if(inst != -1)
        {
            orb_sem_lock(&orb_data[serial][inst]);
            
            orb_data[serial][inst].published = false;
            
            //If the queue does not exist, allocate one
            if(orb_data[serial][inst].queue == nullptr)
            {
                orb_data[serial][inst].queue = new ringbuffer::RingBuffer(queue_size, meta->o_size);
                orb_data[serial][inst].queue->flush();
            }
            
            //Enqueue the data into the current queue
            //Already full, get one and remove it from the queue
            if(orb_data[serial][inst].queue->full())
            {
                
                uint8_t buff[255];
                
                orb_data[serial][inst].queue->get(buff, meta->o_size);
        
                int atomic_state = orb_lock();
                
                std::memcpy(orb_data[serial][inst].data, buff, meta->o_size);
                
                orb_unlock(atomic_state);
                
                orb_data[serial][inst].serial = serial;
            }
            
            //Now we have some space, send in no time
            orb_data[serial][inst].queue->put(data, meta->o_size);     
        
            advert = (void*)(&orb_data[serial][inst]);
            
            orb_data[serial][inst].last_updated_time = hrt_absolute_time();
            
            orb_data[serial][inst].published = true;
            
            int task_id = (int)osThreadGetId();
            for(int i = 0; i < UORB_MAX_SUB; ++i)
            {
                orb_data[serial][inst].authority_list[i] = -1;
            }
            
            orb_sem_unlock(&orb_data[serial][inst]);
        }
    }
    
    return advert;
}

orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta, const void *data, int *instance,
				       int priority, unsigned int queue_size)
{
	return orb_advertise_multi(meta, data, instance, priority, queue_size);
}

int orb_unadvertise(orb_advert_t &handle)
{
    handle = nullptr;
    return 0;
}

int orb_publish_auto(const struct orb_metadata *meta, orb_advert_t *handle, const void *data, int *instance,
		     int priority)
{
	if (*handle == nullptr) {
                int serial = get_orb_serial(meta->o_name);
                if(is_orb_multi(serial))
                    *handle = orb_advertise_multi(meta, data, instance, priority);
                else{
                    *handle = orb_advertise(meta, data);
                    if(instance)
                        *instance = 0;
                }

		if (*handle != nullptr) {
			return 0;
		}

	} else {
		return orb_publish(meta, *handle, data);
	}

	return -1;
}


int  orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
    
    int ret = -1;
    int serial = get_orb_serial(meta->o_name);
    
    if(serial != -1)
    {   
        int instance = 0;

        if(is_orb_multi(serial))
        {
            for(int i = 0; i < ORB_MULTI_MAX_INSTANCES; ++i)
            {
                if(handle == &orb_data[serial][i])
                {
                    instance = i;
                    break;
                }
            }
        }
        
        
        //Not a queued publish
        if(orb_data[serial][instance].queue == nullptr)
        {
            orb_sem_lock(&orb_data[serial][instance]);
            
            orb_data[serial][instance].published = false;
            
            int atomic_state = orb_lock();
            
            std::memcpy(orb_data[serial][instance].data, data, meta->o_size);
            
            orb_unlock(atomic_state);
                
            orb_data[serial][instance].serial = serial;
                
            ret = 0;
                
            orb_data[serial][instance].last_updated_time = hrt_absolute_time();
            
            orb_data[serial][instance].published = true;
            
            for(int i = 0; i < UORB_MAX_SUB; ++i)
            {
                orb_data[serial][instance].authority_list[i] = -1;
            }
            
            for(int i = 0; i < UORB_MAX_POLL; ++i)
            {
                if(poll_queue[i].fd == (int)((serial << 4) | instance))
                {
                    orb_poll_notify(poll_queue[i].fd, POLLIN);
                    break;
                }
            }
            
            orb_sem_unlock(&orb_data[serial][instance]);
        }
        //We have a queued multi publish
        else
        {    
            orb_sem_lock(&orb_data[serial][instance]);
            
            if(orb_data[serial][instance].queue->get(orb_data[serial][instance].data, meta->o_size))
            {   
                orb_data[serial][instance].published = false;
                
                orb_data[serial][instance].serial = serial;
                
                ret = 0;
                
                orb_data[serial][instance].last_updated_time = hrt_absolute_time();
                
                orb_data[serial][instance].published = true;
                
                for(int i = 0; i < UORB_MAX_SUB; ++i)
                {
                    orb_data[serial][instance].authority_list[i] = -1;
                }
                
                for(int i = 0; i < UORB_MAX_POLL; ++i)
                {
                    if(poll_queue[i].fd == (int)((serial << 4) | instance))
                    {
                        orb_poll_notify(poll_queue[i].fd, POLLIN);
                        break;
                    }
                }
                        
                orb_data[serial][instance].queue->put(data, meta->o_size);
            }
            else
                ret = -1;
            orb_sem_unlock(&orb_data[serial][instance]);
        }
    }
    
    return ret;
}

int  orb_subscribe(const struct orb_metadata *meta)
{
    if(meta == nullptr || meta->o_name == nullptr)
        return -1;
    
    int serial = get_orb_serial(meta->o_name);
    
    int ret = -1;
    
    if(serial >= 0 && serial < total_uorb_num)
    {
        int instance = 0;
        if(is_orb_multi(serial))
        {
            instance = ORB_MULTI_MAX_INSTANCES - 1;
        }
        
        int task_id = (int)osThreadGetId();
        
        int atomic_state = orb_lock();
        
        for(int i = 0; i < UORB_MAX_SUB; ++i)
        {
            if(orb_data[serial][instance].registered_list[i] == task_id)
            {
                ret = (int)((serial << 4) | (instance));
                break;
            }
            
            if(orb_data[serial][instance].registered_list[i] == -1)
            {
                orb_data[serial][instance].registered_list[i] = task_id;
                ret = (int)((serial << 4) | (instance));
                break;
            }
        }
        
        orb_unlock(atomic_state);
    }

    return ret;
}

int  orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
    if(meta == nullptr || meta->o_name == nullptr)
        return -1;
    
    int serial = get_orb_serial(meta->o_name);
    
    int ret = -1;
    if(serial >= 0 && serial < total_uorb_num && is_orb_multi(serial))
    {
        int task_id = (int)osThreadGetId();
        
        int atomic_state = orb_lock();
        
        for(int i = 0; i < UORB_MAX_SUB; ++i)
        {
            if(orb_data[serial][instance].registered_list[i] == task_id)
            {
                ret = (int)((serial << 4) | (instance));
                break;
            }
            
            if(orb_data[serial][instance].registered_list[i] == -1)
            {
                orb_data[serial][instance].registered_list[i] = task_id;
                ret = (int)((serial << 4) | (instance));
                break;
            }
        }
        
        orb_unlock(atomic_state);
    }
    
    return ret;
}

int  orb_unsubscribe(int handle)
{
    if(handle < 0)
    {
        return -1;
    }
    
    int serial = (handle >> 4);
    
    int instance = 0;
    if(is_orb_multi(serial))
    {
        instance = ORB_MULTI_MAX_INSTANCES - 1;
    }
    
    if(serial >= 0 && serial < total_uorb_num)
    {   
        int task_id = (int)osThreadGetId();
        
        int atomic_state = orb_lock();
        for(int i = 0; i < UORB_MAX_SUB; ++i)
        {
            if(orb_data[serial][instance].registered_list[i] == task_id)
            {
                orb_data[serial][instance].registered_list[i] = -1;
                orb_unlock(atomic_state);
                return 0;
            }
        }
        orb_unlock(atomic_state);
    }
    else
    {
        return -1;
    }
    
    return 0;
}

int  orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{   
    int ret = -1;
    int serial = get_orb_serial(meta->o_name);
    if(serial != -1 && serial == (handle >> 4))
    {   
        int instance = handle - ((handle >> 4) << 4);
        
        orb_sem_lock(&orb_data[serial][instance]);
        
        if(instance > 0 && !is_orb_multi(serial))
        {
            ret = -1;
            
            orb_sem_unlock(&orb_data[serial][instance]);
            return ret;
        }
        
        else if(instance >= 0 && instance < ORB_MULTI_MAX_INSTANCES && 
           orb_data[serial][instance].data != nullptr)
        {
            bool authorised = false;
            int task_id = (int)osThreadGetId();
                
            for(int i = 0; i < UORB_MAX_SUB; ++i)
            {
                if(orb_data[serial][instance].authority_list[i] == -1)
                {
                    orb_data[serial][instance].authority_list[i] = task_id;
                    authorised = true;
                    break;
                }
            }
            
            if(authorised)
            {
                int atomic_state = orb_lock();
            
                std::memcpy(buffer, orb_data[serial][instance].data, meta->o_size);
            
                orb_unlock(atomic_state);
            
                ret = 0;
            }
            
            int registered_len = 0,authority_len = 0;
            for(int i = 0;i < UORB_MAX_SUB; ++i)
            {
                if(orb_data[serial][instance].registered_list[i] != -1)
                    ++registered_len;
                if(orb_data[serial][instance].authority_list[i] != -1)
                    ++authority_len;
            }
            
            if(authority_len >= registered_len)
                orb_data[serial][instance].published = false;
        }
        
        orb_sem_unlock(&orb_data[serial][instance]);
    }
    
    return ret;
}


int  orb_check(int handle, bool *updated)
{
    if(handle < 0)
    {
        *updated = false;
        return -1;
    }
    
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial < 0 || serial >= total_uorb_num ||
       instance < 0 || instance >= ORB_MULTI_MAX_INSTANCES)
    {
        *updated = false;
        return -1;
    }
    
    orb_sem_lock(&orb_data[serial][instance]);
    
    bool authorised = false;
    int task_id = (int)osThreadGetId();
    
    for(int i = 0; i < UORB_MAX_SUB; ++i)
    {
        if(orb_data[serial][instance].authority_list[i] == task_id)
        {
            authorised = true;
            break;
        }
    }
    
    if(orb_data[serial][instance].published && !authorised)
    {
        *updated = true;
        
        orb_sem_unlock(&orb_data[serial][instance]);
        
        return 0;
    }
    else
        *updated = false;
    
    orb_sem_unlock(&orb_data[serial][instance]);
    
    return -1;
}

int  orb_stat(int handle, uint64_t *time)
{   
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >= 0 && serial < total_uorb_num &&
       instance>= 0 && instance <ORB_MULTI_MAX_INSTANCES)
    {
        *time = orb_data[serial][instance].last_updated_time;
        
        return 0;
    }
    
    return -1;
}

int  orb_exists(const struct orb_metadata *meta, int instance)
{
    int serial = get_orb_serial(meta->o_name);
    if(serial < 0 || serial >= total_uorb_num)
        return -1;
    if(!is_orb_multi(serial) && instance > 0)
        return -1;
    
    if((orb_data[serial][instance].data) != nullptr && 
       (orb_data[serial][instance].serial) != -1)
    {
        return 0;
    }
    
    return -1;
}

int  orb_group_count(const struct orb_metadata *meta)
{
	unsigned instance = 0;

        for(int i = 0; i < ORB_MULTI_MAX_INSTANCES; ++i)
        {
            if(orb_exists(meta, i) == 0)
                ++instance;
        }

	return instance;
}

int  orb_priority(int handle, int32_t *priority)
{
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >=0 && serial < total_uorb_num &&
       instance >=0 && instance < ORB_MULTI_MAX_INSTANCES)
    {
        *priority = orb_data[serial][instance].priority;
        return 0;
    }
    return -1;
}

int orb_set_interval(int handle, unsigned interval)
{
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >= 0 && serial < total_uorb_num &&
       instance >= 0 && instance < ORB_MULTI_MAX_INSTANCES)
    {
        orb_data[serial][instance].interval = interval;
        return 0;
    }
    return -1;
}

int orb_get_interval(int handle, unsigned *interval)
{
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >=0 && serial < total_uorb_num &&
       instance >=0 && instance < ORB_MULTI_MAX_INSTANCES)
    {
        *interval = orb_data[serial][instance].interval;
        return 0;
    }
    return -1;
}

//just POLLIN
int orb_poll(struct pollfd *fds, nfds_t nfds, int timeout)      //timeout: ms
{
    int ret[5];
    
    std::memset(ret, -1, nfds * sizeof(int));
    
    for(int i = 0; i < nfds; ++i)
    {
        if(fds[i].fd < 0)
            continue;
        
        bool registered = false;
        pollevent_t *revents_ptr = nullptr;
        int *fd_ptr = nullptr;
        
        fds[i].id = osThreadGetId();
        fds[i].sem = nullptr;
        fds[i].revents = 0;
        
        for(int j = 0; j < UORB_MAX_POLL; ++j)
        {
            if(fds[i].fd == poll_queue[j].fd && 
               fds[i].events == poll_queue[j].events &&
               fds[i].id == poll_queue[j].id)
            {
                fds[i].sem = poll_queue[j].sem;
                revents_ptr = &poll_queue[j].revents;
                fd_ptr = &poll_queue[j].fd;
                registered = true;
                break;
            }
        }
        
        if(!registered)
        {
            for(int j = 0; j < UORB_MAX_POLL; ++j)
            {
                if(poll_queue[j].fd == -1)
                {
                    poll_queue[j].fd = fds[i].fd;
                    poll_queue[j].id = fds[i].id;
                    poll_queue[j].events = fds[i].events;
                    fds[i].sem = poll_queue[j].sem;
                    revents_ptr = &poll_queue[j].revents;
                    fd_ptr = &poll_queue[j].fd;
                    registered = true;
                    break;
                }
            }
        }
        
        if(registered && fds[i].sem)
        {
            int res = osSemaphoreWait(fds[i].sem, (uint32_t)(((float)(configTICK_RATE_HZ / 1000 * timeout)) / ((float)nfds)));

            if(res == osOK){
                ret[i] = 1;
                
                if(fd_ptr)
                {
                    *fd_ptr = -1;
                }
                
                if(revents_ptr)
                {
                    fds[i].revents = *revents_ptr;
                    *revents_ptr = 0;
                }
            }
            else{
                ret[i] = 0;
                fds[i].revents = 0;
                if(revents_ptr)
                {
                    *revents_ptr = 0;
                }
            }
        }
    }
    
    int final = nfds;
    
    for(int i = 0; i < nfds; ++i)
    {
        if(ret[i] == -1 || ret[i] == 0)
        {
            --final;
        }
    }
    
    return final;
}

void orb_poll_notify(int fd, pollevent_t events)
{
    int atomic_state = orb_lock();
    
    for(int i = 0; i < UORB_MAX_POLL; ++i)
    {
        if(poll_queue[i].fd == fd && poll_queue[i].sem)
        {
            osSemaphoreRelease(poll_queue[i].sem);
            poll_queue[i].revents = events;
        }
    }
    
    orb_unlock(atomic_state);
}


void orb_init(void)
{
    orb_in_os = false;

    orb_helper_init();
    
    for(int i = 0; i < UORB_MAX_POLL; ++i)
    {
        poll_queue[i].fd = -1;
        poll_queue[i].id = 0;
        
        osSemaphoreDef(ORB_POLL_SEM);
        poll_queue[i].sem = osSemaphoreCreate(osSemaphore(ORB_POLL_SEM), 1);
        
        poll_queue[i].revents = 0;
        osSemaphoreWait(poll_queue[i].sem, 0xFFFFFFFF);
    }
    
    for(int i = 0; i < total_uorb_num; ++i)
    {
        if(is_orb_multi(i))
        {
#ifdef USE_UORB_RAM
            orb_data[i] = (ORBData*)ram_ccm.uORBDataCalloc(ORB_MULTI_MAX_INSTANCES, sizeof(ORBData));
#else
            orb_data[i] = new ORBData[ORB_MULTI_MAX_INSTANCES];            
#endif

            for(int j = 0; j < ORB_MULTI_MAX_INSTANCES; ++j)
            {
                orb_data[i][j].data = get_orb_public_according_to_serial_and_instance(i, j);
                orb_data[i][j].priority = (ORB_PRIO)get_priority(j);
                orb_data[i][j].serial = -1;
                orb_data[i][j].interval = 0;
                orb_data[i][j].queue = nullptr;
                orb_data[i][j].published = false;
                
                osSemaphoreDef(ORB_SEM);
                orb_data[i][j].sem = osSemaphoreCreate(osSemaphore(ORB_SEM), 1);
                orb_data[i][j].sem_taken = false;
                
                orb_data[i][j].last_updated_time = 0;
                
                orb_data[i][j].registered_list = new int32_t[UORB_MAX_SUB];
                orb_data[i][j].authority_list = new int32_t[UORB_MAX_SUB];
                
                for(int k = 0; k < UORB_MAX_SUB; ++k)
                {
                    orb_data[i][j].registered_list[k] = -1;
                    orb_data[i][j].authority_list[k] = -1;
                }
            }
        }
        else
        {
#ifdef USE_UORB_RAM
            orb_data[i] = (ORBData*)ram_ccm.uORBDataCalloc(1, sizeof(ORBData));
#else
            orb_data[i] = new ORBData;
#endif
            orb_data[i]->data = get_orb_public_according_to_serial_and_instance(i, 0);
            orb_data[i]->priority = ORB_PRIO_DEFAULT;
            orb_data[i]->serial = -1;
            orb_data[i]->interval = 0;
            orb_data[i]->queue = nullptr;
            orb_data[i]->published = false;
            
            osSemaphoreDef(ORB_SEM);
            orb_data[i]->sem = osSemaphoreCreate(osSemaphore(ORB_SEM), 1);
            orb_data[i]->sem_taken = false;
            
            orb_data[i]->last_updated_time = 0;
            
            orb_data[i]->registered_list = new int32_t[UORB_MAX_SUB];
            orb_data[i]->authority_list = new int32_t[UORB_MAX_SUB];

            for(int k = 0; k < UORB_MAX_SUB; ++k)
            {
                orb_data[i]->registered_list[k] = -1;
                orb_data[i]->authority_list[k] = -1;
            }
        }
    }
}