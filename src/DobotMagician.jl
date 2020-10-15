module DobotMagician

using LibSerialPort
using DocStringExtensions

const Port = LibSerialPort.Port

struct Magician
    port::Port
    timeout::Base.RefValue{Int}
    function Magician(port::Port, timeout::Int)
        return new(port, Ref(timeout))
    end
end
Magician(port::Port) = Magician(port, 1000)  # default timeout 1000ms

"""
    $(SIGNATURES)

Return the serial port used by the attached Dobot Magician. Returns nothing if no
appropriate interface is found.

# Keywords
- `nports_guess=64`: maximum number of serial ports to inspect to find the Dobot interface.

# Returns
- `Union{Port, Nothing}`
"""
function find_port(; nports_guess=64)
    dobot_port = nothing
    ports = sp_list_ports()
    for port in unsafe_wrap(Array, ports, nports_guess; own=false)
        if port == C_NULL
            break
        elseif sp_get_port_transport(port) == SP_TRANSPORT_USB
            if sp_get_port_usb_vid_pid(port) == (0x10c4, 0xea60)
                dobot_port = sp_copy_port(port)
                break
            end
        end
    end
    sp_free_port_list(ports)
    return dobot_port
end

"""
    $(SIGNATURES)

Connect to the specified serial port interface of a Dobot Magician.

# Returns
- `Magician`: a connection to the Dobot Magician.
"""
function connect(port::Port)
    sp_open(port, SP_MODE_READ_WRITE)
    # Values taken from the Dobot Magician Communication Protocol document
    sp_set_baudrate(port, 115200)
    sp_set_bits(port, 8)
    sp_set_stopbits(port, 1)
    sp_set_parity(port, SP_PARITY_NONE)
    sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE)
    return Magician(port)
end

"""
    $(SIGNATURES)

Connect to the serial port interface of a Dobot Magician. Available serial ports are
inspected to find the correct one.

# Returns
- `Port`: the serial port that was opened.
"""
connect(; kwargs...) = connect(find_port(; kwargs...))
connect(::Nothing) = throw(ErrorException("Dobot interface not found"))

"""
    $(SIGNATURES)

Disconnect the specified Dobot Magician.
"""
function disconnect(magician::Magician)
    sp_close(magician.port)
    sp_free_port(magician.port)
    return nothing
end

"""
    Command{S,R}
"""
struct Command{S,R,P}
    id::UInt8
    rw::Bool
    allow_queue::Bool
    description::String
end

_extract_pair(a) = ((), a)
_extract_pair(a::Pair) = (a[1], a[2])

function _extract_pair(a::NTuple{N,<:Pair}) where {N}
    return (Tuple(b[1] for b in a), Tuple{(b[2] for b in a)...})
end

function Command(id, rw, allow_queue, send, receive, description::String)
    # Use pairs because then everything doesn't need to be in Tuples (problems for Vector{UInt8})
    # Will either be a pair or a tuple of pairs
    send_names, send_types = _extract_pair(send)
    receive_names, receive_types = _extract_pair(receive)
    if send_names isa Tuple{}
        payload_names = receive_names
    else
        payload_names = send_names
    end
    return Command{send_types,receive_types,payload_names}(id, rw, allow_queue, description)
end

function (cmd::Command)(magician::Magician, args...; kwargs...)
    return execute_command(magician, cmd, args...; kwargs...)
end

function Base.show(io::IO, ::MIME"text/plain", cmd::Command{S,R,P}) where {S,R,P}
    _S = S <: Tuple ? S.parameters : (S,)
    _R = R <: Tuple ? R.parameters : (R,)
    names = P isa Symbol ? (P,) : P
    types = isempty(_S) ? _R : _S
    return print(
        io,
        """
            Dobot Magician command
            ID:          $(cmd.id)
            R/W:         $(cmd.rw)
            Allow queue: $(cmd.allow_queue)
            Description: $(cmd.description)
            Payload:
                $(join([string(name)*"::"*string(type) for (name, type) in zip(names, types)], "\n    "))""",
    )
end

function payload_size(payload::Union{Vector{UInt8},String})
    return UInt8(2 + length(payload))
end

function payload_size(payload::Tuple)
    return UInt8(mapreduce(sizeof, +, payload; init=2))
end

payload_size(payload) = UInt8(2 + sizeof(payload))

const TIMEOUT_MSG = "Serial port errored or timed out waiting for response"

function execute_command(
    magician::Magician, cmd::Command{S}, payload::S=(); queue=false
) where {S}
    # Construct and send the command package
    package = construct_command(cmd, payload, queue)
    sp_blocking_write(magician.port, package, magician.timeout[])
    sp_drain(magician.port)  # ensure that the command package has been sent
    # Read the header + ID + Ctrl
    nbytes, header = sp_blocking_read(magician.port, 5, magician.timeout[])
    if nbytes < 5
        throw(ErrorException(TIMEOUT_MSG))
    end
    if (header[1] != 0xaa) || (header[2] != 0xaa)
        throw(ErrorException("Invalid header"))
    end
    sz = header[3]
    id = header[4]
    ctrl = header[5]
    # Check the ID
    if id != cmd.id
        throw(ErrorException("Incorrect ID returned"))
    end
    # Check the Ctrl
    if ((ctrl & 0x1) > 0) != cmd.rw
        throw(ErrorException("Incorrect rw returned"))
    end
    if ((ctrl & 0x2) > 0) != queue
        throw(ErrorException("Incorrect isQueued returned"))
    end
    # Read the rest of the payload
    nbytes, payload = sp_blocking_read(magician.port, sz - 2, magician.timeout[])
    if nbytes < (sz - 2)
        throw(ErrorException(TIMEOUT_MSG))
    end
    # Read the checksum
    nbytes, checksum = sp_blocking_read(magician.port, 1, magician.timeout[])
    if nbytes < 1
        throw(ErrorException(TIMEOUT_MSG))
    end
    # Check the checksum
    checksum_actual = -id - ctrl
    for x in payload
        checksum_actual -= x
    end
    if checksum_actual != checksum[1]
        throw(ErrorException("Invalid checksum returned"))
    end
    # Unpack the result
    if queue
        return read(IOBuffer(payload), UInt64)
    else
        return unpack_payload(cmd, payload)
    end
end

function construct_command(cmd::Command{S}, payload::S, queue::Bool) where {S}
    # Uses dispatch to enforce correct payload types
    (!cmd.allow_queue && queue) && throw(ArgumentError("Command cannot be queued"))
    # String checks
    if payload isa String
        if !isascii(payload)
            throw(ArgumentError("Strings must only contain ASCII characters"))
        end
        if payload[end] != '\0'
            payload *= '\0'
        end
    end
    # Construct the message
    sz = payload_size(payload)
    io = IOBuffer(; sizehint=sz + 4)
    write(io, 0xaaaa)
    write(io, sz)
    write(io, cmd.id)
    write(io, UInt8(cmd.rw) | (UInt8(queue) << 1))
    if S <: Tuple
        mapfoldl(x -> write(io, x), +, payload; init=0)
    else
        write(io, payload)
    end
    seek(io, 3)
    checksum = 0x0
    for i in Base.OneTo(sz)
        checksum -= read(io, UInt8)
    end
    write(io, checksum)
    return take!(io)
end

unpack_payload(cmd::Command{<:Any,Vector{UInt8}}, payload) = payload

function unpack_payload(cmd::Command{<:Any,String}, payload)
    # Strip the trailing null terminator
    return read(IOBuffer(payload), String)[1:(end - 1)]
end

function unpack_payload(cmd::Command{<:Any,R}, payload) where {R<:Tuple}
    io = IOBuffer(payload)
    return Tuple(read(io, r) for r in R.parameters)
end

unpack_payload(cmd::Command{<:Any,R}, payload) where {R} = read(IOBuffer(payload), R)

#! format: off

# Commands - Device information
const set_device_sn = Command(0, true, false, :device_sn=>String, (), "Set device serial number")
const get_device_sn = Command(0, false, false, (), :device_sn=>String, "Get device serial number")
const set_device_name = Command(1, true, false, :device_name=>String, (), "Set device name")
const get_device_name = Command(1, false, false, (), :device_name=>String, "Get device name")
const get_device_version = Command(2, false, false, (), (:major_version=>UInt8, :minor_version=>UInt8, :revision=>UInt8), "Get device version")
const set_device_with_l = Command(3, true, false, (:is_with_l=>UInt8, :version=>UInt8), (), "Set sliding rail enable status")
const get_device_with_l = Command(3, false, false, (), :is_with_l=>UInt8, "Get sliding rail enable status")
const get_device_time = Command(4, false, false, (), :g_systick=>UInt32, "Get device time")
const get_device_id = Command(5, false, false, (), (:device_id1=>UInt32, :device_id2=>UInt32, :device_id3=>UInt32), "Get device ID")

# Commands - Real-time pose
const get_pose = Command(10, false, false, (), (:x=>Float32, :y=>Float32, :z=>Float32, :r=>Float32, :J1=>Float32, :J2=>Float32, :J3=>Float32, :J4=>Float32), "Get the real-time pose")
const reset_pose = Command(11, true, false, (:manual=>UInt8, :rear_arm_angle=>Float32, :front_arm_angle=>Float32), (), "Reset the real-time pose")
const get_pose_l = Command(13, false, false, (), :pose_l=>Float32, "Get the real-time pose of the sliding rail")

# Commands - Alarm
const get_alarms_state = Command(20, false, false, (), :alarms_state=>Vector{UInt8}, "Get the alarms state")
const clear_all_alarms_state = Command(21, true, false, (), (), "Clear the alarms state")

# Commands - Homing
const set_home_params = Command(30, true, true, (:x=>Float32, :y=>Float32, :z=>Float32, :r=>Float32), (), "Set the homing position")
const get_home_params = Command(30, false, false, (), (:x=>Float32, :y=>Float32, :z=>Float32, :r=>Float32), "Get the homing position")
const set_home_cmd = Command(31, true, true, :reserved=>UInt32, (), "Execute the homing function")
const set_auto_leveling = Command(32, true, true, (:is_autoleveling=>UInt8, :accuracy=>Float32), (), "Set automatic leveling")
const get_auto_leveling = Command(32, false, false, (), :result=>Float32, "Get automatic leveling result")

# Commands - Handhold teaching
const set_hht_trig_mode = Command(40, true, false, :hht_trig_mode=>UInt64, (), "Set hand-hold teaching mode")
const get_hht_trig_mode = Command(40, false, false, (), :hht_trig_mode=>UInt64, "Get hand-hold teaching mode")
const set_hht_trig_output_enabled = Command(41, true, false, :is_enabled=>UInt8, (), "Set status of hand-hold teaching mode")
const get_hht_trig_output_enabled = Command(41, false, false, (), :is_enabled=>UInt8, "Get status of hand-hold teaching mode")
const get_hht_trig_output = Command(42, false, false, (), :is_triggered=>UInt8, "Get the status of the hand-hold trigger")

# Commands - End effector
const set_end_effector_params = Command(60, true, true, (:x_bias=>Float32, :y_bias=>Float32, :z_bias=>Float32), (), "Set the offset of the end effector")
const get_end_effector_params = Command(60, false, false, (), (:x_bias=>Float32, :y_bias=>Float32, :z_bias=>Float32), "Get the offset of the end effector")
const set_end_effector_laser = Command(61, true, true, (:enable_ctrl=>UInt8, :on=>UInt8), (), "Set the status of the laser")
const get_end_effector_laser = Command(61, false, false, (), (:is_ctrl_enabled=>UInt8, :is_on=>UInt8), "Get the status of the laser")
const set_end_effector_suction_cup = Command(62, true, true, (:is_ctrl_enabled=>UInt8, :is_sucked=>UInt8), (), "Set the status of the suction cup")
const get_end_effector_suction_cup = Command(62, false, false, (), (:is_ctrl_enabled=>UInt8, :is_sucked=>UInt8), "Get the status of the suction cup")
const set_end_effector_gripper = Command(63, true, true, (:is_ctrl_enabled=>UInt8, :is_gripped=>UInt8), (), "Set the status of the gripper")
const get_end_effector_gripper = Command(63, false, false, (), (:is_ctrl_enabled=>UInt8, :is_gripped=>UInt8), "Set the status of the gripper")

# Commands - Jog
const set_jog_joint_params = Command(70, true, true, (:velocity_J1=>Float32,:velocity_J2=>Float32,:velocity_J3=>Float32,:velocity_J4=>Float32,:acceleration_J1=>Float32,:acceleration_J2=>Float32,:acceleration_J3=>Float32,:acceleration_J4=>Float32), (), "Set the velocity and acceleration of joints in jogging mode")
const get_jog_joint_params = Command(70, false, false, (), (:velocity_J1=>Float32,:velocity_J2=>Float32,:velocity_J3=>Float32,:velocity_J4=>Float32,:acceleration_J1=>Float32,:acceleration_J2=>Float32,:acceleration_J3=>Float32,:acceleration_J4=>Float32), "Get the velocity and acceleration of joints in jogging mode")
const set_jog_coordinate_params = Command(71, true, true, (:velocity_x=>Float32,:velocity_y=>Float32,:velocity_z=>Float32,:velocity_r=>Float32,:acceleration_x=>Float32,:acceleration_y=>Float32,:acceleration_z=>Float32,:acceleration_r=>Float32), (), "Set the velocity and acceleration in Cartesian coordinates in jogging mode")
const get_jog_coordinate_params = Command(71, false, false, (), (:velocity_x=>Float32,:velocity_y=>Float32,:velocity_z=>Float32,:velocity_r=>Float32,:acceleration_x=>Float32,:acceleration_y=>Float32,:acceleration_z=>Float32,:acceleration_r=>Float32), "Get the velocity and acceleration in Cartesian coordinates in jogging mode")
const set_jog_common_params = Command(72, true, true, (:velocity_ratio=>Float32, :acceleration_ratio=>Float32), (), "Set the velocity and acceleration ratios of the sliding rail")
const get_jog_common_params = Command(72, false, false, (), (:velocity_ratio=>Float32, :acceleration_ratio=>Float32), "Get the velocity and acceleration ratios of the sliding rail")
const set_jog_cmd = Command(73, true, true, (:is_joint=>UInt8, :cmd=>UInt8), (), "Execute the jog command in Cartesian coordinates or joints (cmd=0 stop, cmd=1 or 2 for positive or negative jogging in x/J1, cmd=3 or 4 for y/J2, etc)")
const set_jog_l_params = Command(74, true, true, (:velocity=>Float32, :acceleration=>Float32), (), "Set the velocity and acceleration of the sliding rail in jog mode") # NOTE: there is a discrepancy in the manual regarding the data types and overall length of the message

# Commands - PTP
const set_ptp_joint_params = Command(80, true, true, (:velocity_J1=>Float32,:velocity_J2=>Float32,:velocity_J3=>Float32,:velocity_J4=>Float32,:acceleration_J1=>Float32,:acceleration_J2=>Float32,:acceleration_J3=>Float32,:acceleration_J4=>Float32), (), "Set the velocity and acceleration of the joint coordinate axes in PTP mode")
const get_ptp_joint_params = Command(80, false, false, (), (:velocity_J1=>Float32,:velocity_J2=>Float32,:velocity_J3=>Float32,:velocity_J4=>Float32,:acceleration_J1=>Float32,:acceleration_J2=>Float32,:acceleration_J3=>Float32,:acceleration_J4=>Float32), "Get the velocity and acceleration of the joint coordinate axes in PTP mode")
const set_ptp_coordinate_params = Command(81, true, true, (:xyz_velocity=>Float32, :r_velocity=>Float32, :xyz_acceleration=>Float32, :r_acceleration=>Float32), (), "Set the velocity and acceleration of the Cartesian coordinate axes in PTP mode")
const get_ptp_coordinate_params = Command(81, false, false, (), (:xyz_velocity=>Float32, :r_velocity=>Float32, :xyz_acceleration=>Float32, :r_acceleration=>Float32), "Get the velocity and acceleration of the Cartesian coordinate axes in PTP mode")
const set_ptp_jump_params = Command(82, true, true, (:jump_height=>Float32, :z_limit=>Float32), (), "Set the lifting height and maximum lifting height in JUMP mode")
const get_ptp_jump_params = Command(82, false, false, (), (:jump_height=>Float32, :z_limit=>Float32), "Get the lifting height and maximum lifting height in JUMP mode")
const set_ptp_common_params = Command(83, true, true, (:velocity_ratio=>Float32, :acceleration_ratio=>Float32), (), "Set the velocity and acceleration ratio in PTP mode")
const get_ptp_common_params = Command(83, false, false, (:velocity_ratio=>Float32, :acceleration_ratio=>Float32), (), "Set the velocity and acceleration ratio in PTP mode")
const set_ptp_cmd = Command(84, true, true, (:ptp_mode=>UInt8, :x=>Float32, :y=>Float32, :z=>Float32, :r=>Float32), (), "Execute a PTP command (ptp_mode: 0=JUMP_XYZ, 1=MOVJ_XYZ, 2=MOVL_XYZ, 3=JUMP_ANGLE, 4=MOVJ_ANGLE, 5=MOVL_ANGLE, 6=MOVJ_INC, 7=MOVL_INC, 8=MOVJ_XYZ_INC, 9=JUMP_MOVL_XYZ)")
const set_ptp_l_params = Command(85, true, true, (:velocity=>Float32, :acceleration=>Float32), (), "Set the velocity and acceleration of the sliding rail in PTP mode")
const set_ptp_with_l_cmd = Command(86, true, true, (:ptp_mode=>UInt8, :x=>Float32, :y=>Float32, :z=>Float32, :r=>Float32, :l=>Float32), (), "Execute a PTP command with the sliding rail (ptp_mode: 0=JUMP_XYZ, 1=MOVJ_XYZ, 2=MOVL_XYZ, 3=JUMP_ANGLE, 4=MOVJ_ANGLE, 5=MOVL_ANGLE, 6=MOVJ_INC, 7=MOVL_INC, 8=MOVJ_XYZ_INC, 9=JUMP_MOVL_XYZ)")
const set_ptp_jump2_params = Command(87, true, true, (:start_jump_height=>Float32, :end_jump_height=>Float32, :z_limit=>Float32), (), "Set the extended parameters in JUMP mode")
const get_ptp_jump2_params = Command(87, false, false, (), (:start_jump_height=>Float32, :end_jump_height=>Float32, :z_limit=>Float32), "Get the extended parameters in JUMP mode")

#! format: on

include("Direct.jl")

end
