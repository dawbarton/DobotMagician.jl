module Simple

using StaticArrays: FieldVector
using DocStringExtensions: SIGNATURES

using ..DobotMagician: Magician, alarm_description
using ..Direct

export XYZR, XYZRL, Joint, JointL, MoveMode, MOVJ, MOVL, JUMP
export move_to,
    rmove_to,
    pose,
    pose_l,
    end_effector,
    laser,
    gripper,
    suction_cup,
    wait_for,
    alarm_description

"""
    XYZR

A structure to represent Cartesian coordinates `(x, y, z)` plus the end effector rotation
`r`.
"""
struct XYZR <: FieldVector{4,Float32}
    x::Float32
    y::Float32
    z::Float32
    r::Float32
end

"""
    XYZRL

A structure to represent Cartesian coordinates `(x, y, z)` plus the end effector rotation
`r` and the sliding rail displacement `l`.
"""
struct XYZRL <: FieldVector{5,Float32}
    x::Float32
    y::Float32
    z::Float32
    r::Float32
    l::Float32
end

"""
    Joint

A structure to represent joint coordinates `(J1, J2, J3, J4)`.
"""
struct Joint <: FieldVector{4,Float32}
    j1::Float32
    j2::Float32
    j3::Float32
    j4::Float32
end

"""
    JointL

A structure to represent joint coordinates `(J1, J2, J3, J4)` plus the sliding rail
displacement `l`.
"""
struct JointL <: FieldVector{5,Float32}
    j1::Float32
    j2::Float32
    j3::Float32
    j4::Float32
    l::Float32
end

abstract type MoveMode end

"""
    MOVJ

Linear movement in the joint coordinate system.
"""
struct MOVJ <: MoveMode end

"""
    MOVL

Linear movement in the Cartesian coordinate system.
"""
struct MOVL <: MoveMode end

"""
    JUMP

Move to `lifting_height` above the current point, execute a MOVL to `lifting_height` above
the destination point, and then descend to the destination point.

Also see `get_ptp_jump_params`, `set_ptp_jump_params`, `get_ptp_jump2_params`, and
`set_ptp_jump2_params`.
"""
struct JUMP <: MoveMode end

"""
    move_to(dobot, xyzr::Union{XYZR, XYZRL}, [::MoveMode=MOVL]; [queue=false])
    move_to(dobot, joint::Union{Joint, JointL}, [::MoveMode=MOVJ]; [queue=false])

Move to the specified position in Cartesian coordinates (`xyzr`) or in joint coordinates
(`joint`) using the specified `MoveMode`, where `MoveMode` is one of `MOVJ`, `MOVL`, or
`JUMP`.

Move commands can be queued or executed immediately. Note: it appears that commands executed
immediately are only successful if no other command is being carried out.
"""
function move_to end

"""
    rmove_to(dobot, xyzr::Union{XYZR, XYZRL}, [::MoveMode=MOVL]; [queue=false])
    rmove_to(dobot, joint::Union{Joint, JointL}, [::MoveMode=MOVJ]; [queue=false])

Move to the specified relative position in Cartesian coordinates (`xyzr`) or in joint
coordinates (`joint`) using the specified `MoveMode`, where `MoveMode` is one of `MOVJ`,
`MOVL`, or `JUMP`.

Move commands can be queued or executed immediately. Note: it appears that commands executed
immediately are only successful if no other command is being carried out.
"""
function rmove_to end

#! format: off

# Defaults
move_to(dobot::Magician, xyzr::Union{XYZR, XYZRL}; queue=false) = move_to(dobot, xyzr, MOVL; queue=queue)
move_to(dobot::Magician, joint::Union{Joint, JointL}; queue=false) = move_to(dobot, joint, MOVJ; queue=queue)

rmove_to(dobot::Magician, xyzr::XYZR; queue=false) = rmove_to(dobot, xyzr, MOVL; queue=queue)
rmove_to(dobot::Magician, joint::Joint; queue=false) = rmove_to(dobot, joint, MOVJ; queue=queue)

# ptp_mode: 0=JUMP_XYZ, 1=MOVJ_XYZ, 2=MOVL_XYZ, 3=JUMP_ANGLE, 4=MOVJ_ANGLE, 5=MOVL_ANGLE,
# 6=MOVJ_INC, 7=MOVL_INC, 8=MOVJ_XYZ_INC, 9=JUMP_MOVL_XYZ

move_to(dobot::Magician, xyzr::XYZR, ::Type{JUMP}; queue=false) = set_ptp_cmd(dobot, (0, xyzr); queue=queue)
move_to(dobot::Magician, xyzr::XYZR, ::Type{MOVJ}; queue=false) = set_ptp_cmd(dobot, (1, xyzr); queue=queue)
move_to(dobot::Magician, xyzr::XYZR, ::Type{MOVL}; queue=false) = set_ptp_cmd(dobot, (2, xyzr); queue=queue)
move_to(dobot::Magician, joint::Joint, ::Type{JUMP}; queue=false) = set_ptp_cmd(dobot, (3, joint); queue=queue)
move_to(dobot::Magician, joint::Joint, ::Type{MOVJ}; queue=false) = set_ptp_cmd(dobot, (4, joint); queue=queue)
move_to(dobot::Magician, joint::Joint, ::Type{MOVL}; queue=false) = set_ptp_cmd(dobot, (5, joint); queue=queue)

move_to(dobot::Magician, xyzrl::XYZRL, ::Type{JUMP}; queue=false) = set_ptp_with_l_cmd(dobot, (0, xyzrl); queue=queue)
move_to(dobot::Magician, xyzrl::XYZRL, ::Type{MOVJ}; queue=false) = set_ptp_with_l_cmd(dobot, (1, xyzrl); queue=queue)
move_to(dobot::Magician, xyzrl::XYZRL, ::Type{MOVL}; queue=false) = set_ptp_with_l_cmd(dobot, (2, xyzrl); queue=queue)
move_to(dobot::Magician, jointl::JointL, ::Type{JUMP}; queue=false) = set_ptp_with_l_cmd(dobot, (3, jointl); queue=queue)
move_to(dobot::Magician, jointl::JointL, ::Type{MOVJ}; queue=false) = set_ptp_with_l_cmd(dobot, (4, jointl); queue=queue)
move_to(dobot::Magician, jointl::JointL, ::Type{MOVL}; queue=false) = set_ptp_with_l_cmd(dobot, (5, jointl); queue=queue)

# Note: Dobot Magician API is rather confusing about the relative moves - these need checking...
rmove_to(dobot::Magician, joint::Joint, ::Type{MOVJ}; queue=false) = set_ptp_cmd(dobot, (6, joint); queue=queue)
rmove_to(dobot::Magician, xyzr::XYZR, ::Type{MOVL}; queue=false) = set_ptp_cmd(dobot, (7, xyzr); queue=queue)
rmove_to(dobot::Magician, xyzr::XYZR, ::Type{MOVJ}; queue=false) = set_ptp_cmd(dobot, (8, xyzr); queue=queue)
rmove_to(dobot::Magician, xyzr::XYZR, ::Type{JUMP}; queue=false) = set_ptp_cmd(dobot, (9, xyzr); queue=queue) # doesn't seem to work?

rmove_to(dobot::Magician, jointl::JointL, ::Type{MOVJ}; queue=false) = set_ptp_with_l_cmd(dobot, (6, jointl); queue=queue)
rmove_to(dobot::Magician, jointl::JointL, ::Type{MOVL}; queue=false) = set_ptp_with_l_cmd(dobot, (7, jointl); queue=queue)
rmove_to(dobot::Magician, xyzrl::XYZRL, ::Type{MOVJ}; queue=false) = set_ptp_with_l_cmd(dobot, (8, xyzrl); queue=queue)
rmove_to(dobot::Magician, xyzrl::XYZRL, ::Type{JUMP}; queue=false) = set_ptp_with_l_cmd(dobot, (9, xyzrl); queue=queue)

#! format: on

"""
    $(SIGNATURES)

Return the pose of the Dobot in the specified coordinate frame. The coordinate frame can be
`XYZR` or `Joint`; if the sliding rail attachment is used, the corresponding coordinate
frames are `XYZRL` and `JointL`.
"""
pose(dobot::Magician, coordinates::Type{XYZR}) = XYZR(get_pose(dobot)[1])
pose(dobot::Magician, ::Type{Joint}) = Joint(get_pose(dobot)[2])
pose(dobot::Magician, ::Type{XYZRL}) = XYZRL(get_pose(dobot)[1]..., get_pose_l(dobot)[1])
pose(dobot::Magician, ::Type{JointL}) = JointL(get_pose(dobot)[2]..., get_pose_l(dobot)[1])

"""
    $(SIGNATURES)

Enable control of a particular end effector. Valid values for `effector` are

- `:laser`
- `:gripper`
- `:suction_cup`
"""
function end_effector(dobot::Magician, effector::Symbol, ctrl::Bool=true; queue=false)
    if effector === :laser
        set_end_effector_laser(dobot, (UInt8(ctrl), 0); queue=queue)
    elseif effector === :gripper
        set_end_effector_gripper(dobot, (UInt8(ctrl), 0); queue=queue)
    elseif effector === :suction_cup
        set_end_effector_suction_cup(dobot, (UInt8(ctrl), 0); queue=queue)
    else
        throw(ArgumentError("Unknown end effector"))
    end
    return nothing
end

"""
    $(SIGNATURES)

Return true if the laser is on, otherwise return false.
"""
function laser(dobot::Magician)
    (ctrl, on) = get_end_effector_laser(dobot)
    return (ctrl == 0x1) && (on == 0x1)
end

"""
    $(SIGNATURES)

Turn the laser on or off.
"""
function laser(dobot::Magician, status::Bool; queue=false)
    return set_end_effector_laser(dobot, (1, UInt8(status)); queue=queue)
end

"""
    $(SIGNATURES)

Return true if the gripper is on, otherwise return false.
"""
function gripper(dobot::Magician)
    (ctrl, on) = get_end_effector_gripper(dobot)
    return (ctrl == 0x1) && (on == 0x1)
end

"""
    $(SIGNATURES)

Turn the gripper on or off.
"""
function gripper(dobot::Magician, status::Bool; queue=false)
    return set_end_effector_gripper(dobot, (1, UInt8(status)); queue=queue)
end

"""
    $(SIGNATURES)

Return true if the suction_cup is on, otherwise return false.
"""
function suction_cup(dobot::Magician)
    (ctrl, on) = get_end_effector_suction_cup(dobot)
    return (ctrl == 0x1) && (on == 0x1)
end

"""
    $(SIGNATURES)

Turn the suction_cup on or off.
"""
function suction_cup(dobot::Magician, status::Bool; queue=false)
    return set_end_effector_suction_cup(dobot, (1, UInt8(status)); queue=queue)
end

"""
    $(SIGNATURES)

Wait for `timeout` milliseconds.
"""
function wait_for(dobot::Magician, timeout::Integer; queue=false)
    return set_wait_cmd(dobot, (timeout,); queue=queue)
end

end  # module
