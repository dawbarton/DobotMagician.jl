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
    Command{P}
"""
struct Command{P}
    id::UInt8
    rw::Bool
    allow_queue::Bool
    description::String
    payload::Vector{Union{Tuple{Symbol,DataType},Tuple{Symbol,UnionAll}}}
end

function Command(id, rw, allow_queue, payload, description)
    return Command{Tuple{values(payload)...}}(
        id, rw, allow_queue, description, [(name, type) for (name, type) in pairs(payload)]
    )
end

function (cmd::Command)(magician::Magician, args...; kwargs...)
    return execute_command(magician, cmd, args...; kwargs...)
end

function Base.show(io::IO, ::MIME"text/plain", cmd::Command)
    return print(
        io,
        """
            Dobot Magician command
            ID:          $(cmd.id)
            R/W:         $(cmd.rw)
            Allow queue: $(cmd.allow_queue)
            Description: $(cmd.description)
            Payload:
                $(join([string(name)*"::"*string(type) for (name, type) in cmd.payload], "\n    "))""",
    )
end

const TIMEOUT_MSG = "Serial port errored or timed out waiting for response"

function execute_command(magician::Magician, cmd::Command, payload=(); queue=false)
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
    elseif !cmd.rw
        return unpack_payload(cmd, payload)
    else
        return nothing
    end
end

write_var_as_type(io::IO, T::Type, var) = (write(io, convert(T, var)); nothing)

function write_var_as_type(io::IO, T::Type{String}, var)
    str = string(var)
    if !isascii(str)
        throw(ArgumentError("Strings must only use ASCII characters"))
    end
    if !isempty(str) && str[end] != '\0'
        str *= '\0'
    end
    write(io, str)
    return nothing
end

_write_var_as_type(T::Type, ::Any, varsym) = :(write_var_as_type(io, $T, $varsym))

function _write_var_as_type(T::Type{<:Tuple}, var, varsym)
    if T isa UnionAll
        # This occurs where there can be an unspecified number of repeated units
        # Note the body of the UnionAll of an NTuple is Tuple{Vararg{innerT, N}}
        body = T.body
        if !(body isa UnionAll) && (body.parameters[1] <: Vararg)
            innerT = body.parameters[1].parameters[1]
            N = var <: Tuple ? length(var.parameters) : length(var)  # allows for StaticArrays
            return _write_var_as_type(NTuple{N,innerT}, var, varsym)  # call again with bound N
        else
            throw(ArgumentError("Got a UnionAll that is not an NTuple with unbound N (only)"))
        end
    else
        body = quote end
        for (i, TT) in enumerate(T.parameters)
            push!(
                body.args,
                _write_var_as_type(
                    TT, var <: Tuple ? var.parameters[i] : eltype(var), :($(varsym)[$i])
                ),
            )
        end
        push!(body.args, :(nothing))
        return body
    end
end

@generated function write_var_as_type(io::IO, T::Type{<:Tuple}, var)
    # Use a generated function to unroll the loop over all the components of the Tuple
    return _write_var_as_type(T.parameters[1], var, :var)
end

function construct_command(cmd::Command{P}, payload, queue::Bool) where {P}
    # Uses dispatch to enforce correct payload types
    (!cmd.allow_queue && queue) && throw(ArgumentError("Command cannot be queued"))
    # Construct the message
    io = IOBuffer()
    write(io, 0xaaaa)
    write(io, 0x0) # write len later
    write(io, cmd.id)
    write(io, UInt8(cmd.rw) | (UInt8(queue) << 1))
    if cmd.rw
        write_var_as_type(io, P, payload)
    end
    sz = UInt8(position(io) - 3)
    seek(io, 2)
    write(io, sz)
    checksum = 0x0
    for i in Base.OneTo(sz)
        checksum -= read(io, UInt8)
    end
    write(io, checksum)
    return take!(io)
end

read_var_as_type(io::IO, T::Type) = read(io, T)
read_var_as_type(io::IO, T::Type{Vector{UInt8}}) = read(io)
read_var_as_type(io::IO, T::Type{String}) = rstrip(read(io, String), ('\0',))

_read_var_as_type(T::Type) = :(read_var_as_type(io, $T))

function _read_var_as_type(T::Type{<:Tuple})
    if T isa UnionAll
        # This occurs where there can be an unspecified number of repeated units
        # Note the body of the UnionAll of an NTuple is Tuple{Vararg{innerT, N}}
        body = T.body
        if !(body isa UnionAll) && (body.parameters[1] <: Vararg)
            innerT = body.parameters[1].parameters[1]
            return quote
                args = []
                while !eof(io)
                    push!(args, _read_var_as_type(innerT))
                end
                args
            end
        else
            throw(ArgumentError("Got a UnionAll that is not an NTuple with unbound N (only)"))
        end
    else
        body = :(())
        for TT in T.parameters
            push!(body.args, _read_var_as_type(TT))
        end
        return body
    end
end

@generated function read_var_as_type(io::IO, T::Type{<:Tuple})
    return _read_var_as_type(T.parameters[1])
end

function unpack_payload(cmd::Command{P}, payload) where {P}
    io = IOBuffer(payload)
    return read_var_as_type(io, P)
end

#! format: off

# Commands - Device information
const set_device_sn = Command(0, true, false, (device_sn=String,), "Set device serial number")
const get_device_sn = Command(0, false, false, (device_sn=String,), "Get device serial number")
const set_device_name = Command(1, true, false, (device_name=String,), "Set device name")
const get_device_name = Command(1, false, false, (device_name=String,), "Get device name")
const get_device_version = Command(2, false, false, (major_version=UInt8, minor_version=UInt8, revision=UInt8), "Get device version")
const set_device_with_l = Command(3, true, false, (is_with_l=UInt8, version=UInt8), "Set sliding rail enable status")
const get_device_with_l = Command(3, false, false, (is_with_l=UInt8,), "Get sliding rail enable status")
const get_device_time = Command(4, false, false, (g_systick=UInt32,), "Get device time")
const get_device_id = Command(5, false, false, (device_id=NTuple{3,UInt32},), "Get device ID")

# Commands - Real-time pose
const get_pose = Command(10, false, false, (xyzr=NTuple{4,Float32}, J=NTuple{4,Float32}), "Get the real-time pose")
const reset_pose = Command(11, true, false, (manual=UInt8, rear_arm_angle=Float32, front_arm_angle=Float32), "Reset the real-time pose")
const get_pose_l = Command(13, false, false, (pose_l=Float32,), "Get the real-time pose of the sliding rail")

# Commands - Alarm
const get_alarms_state = Command(20, false, false, (alarms_state=Vector{UInt8},), "Get the alarms state")
const clear_all_alarms_state = Command(21, true, false, (), "Clear the alarms state")

# Commands - Homing
const set_home_params = Command(30, true, true, (x=Float32, y=Float32, z=Float32, r=Float32), "Set the homing position")
const get_home_params = Command(30, false, false, (x=Float32, y=Float32, z=Float32, r=Float32), "Get the homing position")
const set_home_cmd = Command(31, true, true, (reserved=UInt32,), "Execute the homing function")
const set_auto_leveling = Command(32, true, true, (is_autoleveling=UInt8, accuracy=Float32), "Set automatic leveling")
const get_auto_leveling = Command(32, false, false, (result=Float32,), "Get automatic leveling result")

# Commands - Handhold teaching
const set_hht_trig_mode = Command(40, true, false, (hht_trig_mode=UInt64,), "Set hand-hold teaching mode")
const get_hht_trig_mode = Command(40, false, false, (hht_trig_mode=UInt64,), "Get hand-hold teaching mode")
const set_hht_trig_output_enabled = Command(41, true, false, (is_enabled=UInt8,), "Set status of hand-hold teaching mode")
const get_hht_trig_output_enabled = Command(41, false, false, (is_enabled=UInt8,), "Get status of hand-hold teaching mode")
const get_hht_trig_output = Command(42, false, false, (is_triggered=UInt8,), "Get the status of the hand-hold trigger")

# Commands - End effector
const set_end_effector_params = Command(60, true, true, (x_bias=Float32, y_bias=Float32, z_bias=Float32), "Set the offset of the end effector")
const get_end_effector_params = Command(60, false, false, (x_bias=Float32, y_bias=Float32, z_bias=Float32), "Get the offset of the end effector")
const set_end_effector_laser = Command(61, true, true, (enable_ctrl=UInt8, on=UInt8), "Set the status of the laser")
const get_end_effector_laser = Command(61, false, false, (is_ctrl_enabled=UInt8, is_on=UInt8), "Get the status of the laser")
const set_end_effector_suction_cup = Command(62, true, true, (is_ctrl_enabled=UInt8, is_sucked=UInt8), "Set the status of the suction cup")
const get_end_effector_suction_cup = Command(62, false, false, (is_ctrl_enabled=UInt8, is_sucked=UInt8), "Get the status of the suction cup")
const set_end_effector_gripper = Command(63, true, true, (is_ctrl_enabled=UInt8, is_gripped=UInt8), "Set the status of the gripper")
const get_end_effector_gripper = Command(63, false, false, (is_ctrl_enabled=UInt8, is_gripped=UInt8), "Set the status of the gripper")

# Commands - Jog
const set_jog_joint_params = Command(70, true, true, (velocity=NTuple{4,Float32}, acceleration=NTuple{4,Float32}), "Set the velocity and acceleration of joints in jogging mode")
const get_jog_joint_params = Command(70, false, false, (velocity=NTuple{4,Float32}, acceleration=NTuple{4,Float32}), "Get the velocity and acceleration of joints in jogging mode")
const set_jog_coordinate_params = Command(71, true, true, (velocity=NTuple{4,Float32}, acceleration=NTuple{4,Float32}), "Set the velocity and acceleration in Cartesian coordinates in jogging mode")
const get_jog_coordinate_params = Command(71, false, false, (velocity=NTuple{4,Float32}, acceleration=NTuple{4,Float32}), "Get the velocity and acceleration in Cartesian coordinates in jogging mode")
const set_jog_common_params = Command(72, true, true, (velocity_ratio=Float32, acceleration_ratio=Float32), "Set the velocity and acceleration ratios of the sliding rail")
const get_jog_common_params = Command(72, false, false, (velocity_ratio=Float32, acceleration_ratio=Float32), "Get the velocity and acceleration ratios of the sliding rail")
const set_jog_cmd = Command(73, true, true, (is_joint=UInt8, cmd=UInt8), "Execute the jog command in Cartesian coordinates or joints (cmd=0 stop, cmd=1 or 2 for positive or negative jogging in x/J1, cmd=3 or 4 for y/J2, etc)")
const set_jog_l_params = Command(74, true, true, (velocity=Float32, acceleration=Float32), "Set the velocity and acceleration of the sliding rail in jog mode") # NOTE: there is a discrepancy in the manual regarding the data types and overall length of the message

# Commands - PTP
const set_ptp_joint_params = Command(80, true, true, (velocity=NTuple{4,Float32}, acceleration=NTuple{4,Float32}), "Set the velocity and acceleration of the joint coordinate axes in PTP mode")
const get_ptp_joint_params = Command(80, false, false, (velocity=NTuple{4,Float32}, acceleration=NTuple{4,Float32}), "Get the velocity and acceleration of the joint coordinate axes in PTP mode")
const set_ptp_coordinate_params = Command(81, true, true, (xyz_velocity=Float32, r_velocity=Float32, xyz_acceleration=Float32, r_acceleration=Float32), "Set the velocity and acceleration of the Cartesian coordinate axes in PTP mode")
const get_ptp_coordinate_params = Command(81, false, false, (xyz_velocity=Float32, r_velocity=Float32, xyz_acceleration=Float32, r_acceleration=Float32), "Get the velocity and acceleration of the Cartesian coordinate axes in PTP mode")
const set_ptp_jump_params = Command(82, true, true, (jump_height=Float32, z_limit=Float32), "Set the lifting height and maximum lifting height in JUMP mode")
const get_ptp_jump_params = Command(82, false, false, (jump_height=Float32, z_limit=Float32), "Get the lifting height and maximum lifting height in JUMP mode")
const set_ptp_common_params = Command(83, true, true, (velocity_ratio=Float32, acceleration_ratio=Float32), "Set the velocity and acceleration ratio in PTP mode")
const get_ptp_common_params = Command(83, false, false, (velocity_ratio=Float32, acceleration_ratio=Float32), "Set the velocity and acceleration ratio in PTP mode")
const set_ptp_cmd = Command(84, true, true, (ptp_mode=UInt8, position=NTuple{4,Float32}), "Execute a PTP command (ptp_mode: 0=JUMP_XYZ, 1=MOVJ_XYZ, 2=MOVL_XYZ, 3=JUMP_ANGLE, 4=MOVJ_ANGLE, 5=MOVL_ANGLE, 6=MOVJ_INC, 7=MOVL_INC, 8=MOVJ_XYZ_INC, 9=JUMP_MOVL_XYZ)")
const set_ptp_l_params = Command(85, true, true, (velocity=Float32, acceleration=Float32), "Set the velocity and acceleration of the sliding rail in PTP mode")
const set_ptp_with_l_cmd = Command(86, true, true, (ptp_mode=UInt8, position=NTuple{5,Float32}), "Execute a PTP command with the sliding rail (ptp_mode: 0=JUMP_XYZ, 1=MOVJ_XYZ, 2=MOVL_XYZ, 3=JUMP_ANGLE, 4=MOVJ_ANGLE, 5=MOVL_ANGLE, 6=MOVJ_INC, 7=MOVL_INC, 8=MOVJ_XYZ_INC, 9=JUMP_MOVL_XYZ)")
const set_ptp_jump2_params = Command(87, true, true, (start_jump_height=Float32, end_jump_height=Float32, z_limit=Float32), "Set the extended parameters in JUMP mode")
const get_ptp_jump2_params = Command(87, false, false, (start_jump_height=Float32, end_jump_height=Float32, z_limit=Float32), "Get the extended parameters in JUMP mode")
const set_ptp_po_cmd = Command(88, true, true, (ptp_mode=UInt8, position=NTuple{4,Float32}, po=NTuple{N,Tuple{UInt8,UInt16,UInt8}} where N), "Execute a PTP command while manipulating the I/O at a certain point (po=(ratio, address, level))")
const set_ptp_po_with_l_cmd = Command(89, true, true, (ptp_mode=UInt8, position=NTuple{5,Float32}, po=NTuple{N,Tuple{UInt8,UInt16,UInt8}} where N), "Execute a PTP command with the sliding rail while manipulating the I/O at a certain point (po=(ratio, address, level))")

# Commands - CP
const set_cp_params = Command(90, true, true, (plan_acc=Float32, junction_vel=Float32, acc_period=Float32, real_time_track=UInt8), "Set the parameters of continuous trajectory; acc_period is the maximum actual acceleration in non-real time mode and the interpolation period in real-time mode")
const get_cp_params = Command(90, false, false, (plan_acc=Float32, junction_vel=Float32, acc_period=Float32, real_time_track=UInt8), "Get the parameters of continuous trajectory; acc_period is the maximum actual acceleration in non-real time mode and the interpolation period in real-time mode")
const set_cp_cmd = Command(91, true, true, (cp_mode=UInt8, xyz=NTuple{3,Float32}, velocity=Float32), "Execute the CP command (cp_mode=0 for relative motion or cp_mode=1 for absolute motion; velocity is reserved/ignored?)")
const set_cp_le_cmd = Command(92, true, true, (cp_mode=UInt8, xyz=NTuple{3,Float32}, power=Float32), "Execute the CP command with laser engraving (cp_mode=0 for relative motion or cp_mode=1 for absolute motion)")

# Commands - Queued execution control commands
const set_queued_cmd_start_exec = Command(240, true, false, (), "Start the command queue")
const set_queued_cmd_stop_exec = Command(241, true, false, (), "Stop the command queue")
const set_queued_cmd_force_stop_exec = Command(242, true, false, (), "Forcibly stop the command queue")
const set_queued_cmd_start_download = Command(243, true, false, (total_loop=UInt32, line_per_loop=UInt32), "Start downloading")
const set_queued_cmd_stop_download = Command(244, true, false, (), "Stop downloading")
const set_queued_cmd_clear = Command(245, true, false, (), "Clear the command queue")
const get_queued_cmd_current_index = Command(246, false, false, (queued_cmd_current_index=UInt64,), "Get the command index")

#! format: on

include("Direct.jl")

end
