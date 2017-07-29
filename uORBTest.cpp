#include <uORB/uORB.h>
#include <cstring>
#include "cmsis_os.h"
#include "uORBTest.h"
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/uORBHelper.h>
#include <drv_hrt.h>

orb_advert_t       att_adv;
orb_advert_t       att_sp_adv;

void uORBPub_Task(void const * args)
{
    vehicle_attitude_setpoint_s     att_sp={};
    vehicle_attitude_s              att={};
  
    att_sp.pitch_body=1.0f;
    att.pitchspeed=14.5f;
    
    orb_set_in_os();
  
    att_sp_adv=orb_advertise_queue(ORB_ID(vehicle_attitude_setpoint),&att_sp,5);
    att_adv=orb_advertise_multi_queue(ORB_ID(vehicle_attitude),&att,nullptr,ORB_PRIO_DEFAULT,10);
  
    uint32_t counter=0;
    while(1)
    {
        osDelay(1);
        
        att_sp.q_d[0]=(float)++counter;
        att.q[0]=(float)++counter;
        
        att_sp.timestamp=hrt_absolute_time();
        att.timestamp=hrt_absolute_time();
        
    
        orb_publish(ORB_ID(vehicle_attitude_setpoint),att_sp_adv,&att_sp);
        orb_publish(ORB_ID(vehicle_attitude),att_adv,&att);
    }
}


vehicle_attitude_s              Att;
vehicle_attitude_setpoint_s     Att_sp;

void uORBSub_Task(void const * args)
{
    int       att_sub,att_sp_sub;
    
    for(int i=0;i<10000;++i)
    {
        osDelay(1);
        att_sub=orb_subscribe_multi(ORB_ID(vehicle_attitude),3);
        att_sp_sub=orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    }
  
    while(1)
    { 
        osDelay(1);
        bool updated;
        
        orb_check(att_sub,&updated);
        if(updated)
            orb_copy(ORB_ID(vehicle_attitude),att_sub,&Att);
        
        orb_check(att_sp_sub,&updated);
        if(updated)
            orb_copy(ORB_ID(vehicle_attitude_setpoint),att_sp_sub,&Att_sp);
    }
}