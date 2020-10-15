"""
    Direct

This module exports all the direct interface functions. This is a low-level interface to the
Dobot that directly exposes all Dobot functionality.
"""
module Direct

using ..DobotMagician:
    # Commands - Device information
    set_device_sn,
    get_device_sn,
    set_device_name,
    get_device_name,
    get_device_version,
    set_device_with_l,
    get_device_with_l,
    get_device_time,
    get_device_id,
    # Commands - Real-time pose
    get_pose,
    reset_pose,
    get_pose_l,
    # Commands - Alarm
    get_alarms_state,
    clear_all_alarms_state,
    # Commands - Homing
    set_home_params,
    get_home_params,
    set_home_cmd,
    set_auto_leveling,
    get_auto_leveling,
    # Commands - Handhold teaching
    set_hht_trig_mode,
    get_hht_trig_mode,
    set_hht_trig_output_enabled,
    get_hht_trig_output_enabled,
    get_hht_trig_output,
    # Commands - End effector
    set_end_effector_params,
    get_end_effector_params,
    set_end_effector_laser,
    get_end_effector_laser,
    set_end_effector_suction_cup,
    get_end_effector_suction_cup,
    set_end_effector_gripper,
    get_end_effector_gripper,
    # Commands - Jog
    set_jog_joint_params,
    get_jog_joint_params,
    set_jog_coordinate_params,
    get_jog_coordinate_params,
    set_jog_common_params,
    get_jog_common_params,
    set_jog_cmd,
    set_jog_l_params,
    # Commands - PTP
    set_ptp_joint_params,
    get_ptp_joint_params,
    set_ptp_coordinate_params,
    get_ptp_coordinate_params,
    set_ptp_jump_params,
    get_ptp_jump_params,
    set_ptp_common_params,
    get_ptp_common_params,
    set_ptp_cmd,
    set_ptp_l_params,
    set_ptp_with_l_cmd,
    set_ptp_jump2_params,
    get_ptp_jump2_params

export
    # Commands - Device information
    set_device_sn,
    get_device_sn,
    set_device_name,
    get_device_name,
    get_device_version,
    set_device_with_l,
    get_device_with_l,
    get_device_time,
    get_device_id,
    # Commands - Real-time pose
    get_pose,
    reset_pose,
    get_pose_l,
    # Commands - Alarm
    get_alarms_state,
    clear_all_alarms_state,
    # Commands - Homing
    set_home_params,
    get_home_params,
    set_home_cmd,
    set_auto_leveling,
    get_auto_leveling,
    # Commands - Handhold teaching
    set_hht_trig_mode,
    get_hht_trig_mode,
    set_hht_trig_output_enabled,
    get_hht_trig_output_enabled,
    get_hht_trig_output,
    # Commands - End effector
    set_end_effector_params,
    get_end_effector_params,
    set_end_effector_laser,
    get_end_effector_laser,
    set_end_effector_suction_cup,
    get_end_effector_suction_cup,
    set_end_effector_gripper,
    get_end_effector_gripper,
    # Commands - Jog
    set_jog_joint_params,
    get_jog_joint_params,
    set_jog_coordinate_params,
    get_jog_coordinate_params,
    set_jog_common_params,
    get_jog_common_params,
    set_jog_cmd,
    set_jog_l_params,
    # Commands - PTP
    set_ptp_joint_params,
    get_ptp_joint_params,
    set_ptp_coordinate_params,
    get_ptp_coordinate_params,
    set_ptp_jump_params,
    get_ptp_jump_params,
    set_ptp_common_params,
    get_ptp_common_params,
    set_ptp_cmd,
    set_ptp_l_params,
    set_ptp_with_l_cmd,
    set_ptp_jump2_params,
    get_ptp_jump2_params

end
