#!/usr/bin/env python3


import rospy
import rosservice

# this scrips calls the necessary ros services to set the settings of yumi and start EGM.

def myhook():
    print("shutdown begin")
    rospy.wait_for_service('/yumi/rws/sm_addin/stop_egm')
    rosservice.call_service('/yumi/rws/sm_addin/stop_egm', {})

    rospy.sleep(0.7)

    rospy.wait_for_service('/yumi/rws/stop_rapid')
    rosservice.call_service('/yumi/rws/stop_rapid', {})
    rospy.sleep(0.7)

    print("shutdown commplete")


if __name__ == "__main__":
    # first makes sure that anything running is stopped
    rospy.wait_for_service('/yumi/rws/stop_rapid')
    rosservice.call_service('/yumi/rws/stop_rapid', {})
    rospy.sleep(1.5)
    # resets and starts rapid scripts
    rospy.wait_for_service('/yumi/rws/pp_to_main')
    rosservice.call_service('/yumi/rws/pp_to_main', {})
    rospy.sleep(1.5)
    rospy.wait_for_service('/yumi/rws/start_rapid')
    rosservice.call_service('/yumi/rws/start_rapid', {})
    rospy.sleep(1.5)

    # settings, see wiki for more information
    setup_uc = dict(use_filtering=True, comm_timeout=1.0)
    xyz = dict(x=0.0, y=0.0, z=0.0)
    quat = dict(q1=1.0, q2=0.0, q3=0.0, q4=0.0)
    tframe = dict(trans=xyz, rot=quat)
    #tload = dict(mass=0.001, cog=dict(x=0.0, y=0.0, z=0.001), aom=quat, ix=0.0, iy=0.0, iz=0.0)
    #mass [kg], cog [mm], inertia [kgm^2]
    # page 21-22 in https://abb.sluzba.cz/Pages/Public/IRC5UserDocumentationRW6/en/3HC054949%20PM%20IRB%2014000%20Gripper-en.pdf
    tload = dict(mass=0.230, cog=dict(x=8.2, y=11.7, z=52.0), aom=quat, ix=0.00021, iy=0.00024, iz=0.00009)
    tload_F = dict(mass=0.230, cog=dict(x=8.2, y=11.7, z=52.0), aom=quat, ix=0.00021, iy=0.00024, iz=0.00009)
    tool = dict(robhold=True, tframe=tframe, tload=tload)
    tool_F = dict(robhold=True, tframe=tframe, tload=tload)
    wobj = dict(robhold=False, ufprog=True,  ufmec='', uframe=dict(trans=xyz, rot=quat), oframe=dict(trans=xyz,
                                                                                                     rot=quat))
    correction_frame=dict(trans=xyz, rot=quat)
    sensor_frame=dict(trans=xyz, rot=quat)
    activate = dict(tool=tool, wobj=wobj, correction_frame=correction_frame, sensor_frame=sensor_frame,
                    cond_min_max=0.0, lp_filter=20.0, sample_rate=4, max_speed_deviation=90.0)
    activate_F = dict(tool=tool_F, wobj=wobj, correction_frame=correction_frame, sensor_frame=sensor_frame,
                    cond_min_max=0.0, lp_filter=20.0, sample_rate=4, max_speed_deviation=90.0)
    run = dict(cond_time=60.0, ramp_in_time=1.0, offset=dict(trans=xyz, rot=quat), pos_corr_gain=0.0)
    stop = dict(ramp_out_time=1.0)
    settings = dict(allow_egm_motions=True, use_presync=False, setup_uc=setup_uc, activate=activate, run=run, stop=stop)
    settings_F = dict(allow_egm_motions=True, use_presync=False, setup_uc=setup_uc, activate=activate_F, run=run, stop=stop)

    # set settings for both arms.
    rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')
    rosservice.call_service('/yumi/rws/sm_addin/set_egm_settings', dict(task="T_ROB_L", settings=settings_F))
    rospy.sleep(1.5)
    rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')
    rosservice.call_service('/yumi/rws/sm_addin/set_egm_settings', dict(task="T_ROB_R", settings=settings))
    rospy.sleep(1.5)

    # starts the EGM session
    rospy.wait_for_service('/yumi/rws/sm_addin/start_egm_joint')
    rosservice.call_service('/yumi/rws/sm_addin/start_egm_joint', {})
    rospy.sleep(1.5)

    rospy.wait_for_service('/yumi/egm/controller_manager/switch_controller')
    print(rosservice.call_service('/yumi/egm/controller_manager/switch_controller',
                                  dict(start_controllers=['joint_group_velocity_controller'],
                                       stop_controllers=[''],
                                       strictness=1,
                                       start_asap=False,
                                       timeout=0.0)))

    rospy.on_shutdown(myhook)

    rospy.spin()




