module DobotMagician

using LibSerialPort
using DocStringExtensions

export connect, disconnect, send, Magician

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
                dobot_port = port
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
    return sp_close(magician.port)
end

"""
    Command{S,R}
"""
struct Command{S,R}
    id::UInt8
    rw::Bool
    allow_queue::Bool
end
Command(id, rw, allow_queue, send, receive) = Command{send,receive}(id, rw, allow_queue)

function payload_size(cmd::Command{Vector{UInt8}}, payload::Vector{UInt8})
    return UInt8(2 + length(payload))
end

function payload_size(cmd::Command{String}, payload::String)
    if !isascii(payload)
        throw(ArgumentError("Strings must only contain ASCII characters"))
    end
    return UInt8(2 + length(payload))
end

function payload_size(cmd::Command{S}, payload::S) where {S<:Tuple}
    return mapreduce(sizeof, +, S; init=0x2)
end

payload_size(cmd::Command{S}, payload::S) where {S} = UInt8(2 + sizeof(S))

const TIMEOUT_MSG = "Serial port errored or timed out waiting for response"

function execute_command(
    magician::Magician, cmd::Command{S}, payload::S; queue=false
) where {S}
    # Construct and send the command package
    package = construct_command(cmd, payload, queue)
    sp_blocking_write(magician.port, package, magician.timeout[])
    sp_drain(magician.port)  # ensure that the command package has been sent
    # Read the header
    header = Ref(zero(UInt16))
    nbytes = Int(sp_blocking_read(magician.port, header, 1, magician.timeout[]))
    if nbytes < 2
        throw(ErrorException(TIMEOUT_MSG))
    end
    if header[] != 0xaaaa
        throw(ErrorException("Invalid header"))
    end
    # Read the payload length
    sz = Ref(zero(UInt8))
    nbytes = Int(sp_blocking_read(magician.port, sz, 1, magician.timeout[]))
    if nbytes < 1
        throw(ErrorException(TIMEOUT_MSG))
    end
    # Read the ID
    id = Ref(zero(UInt8))
    nbytes = Int(sp_blocking_read(magician.port, id, 1, magician.timeout[]))
    if nbytes < 1
        throw(ErrorException(TIMEOUT_MSG))
    end
    # Check the ID
    if id[] != cmd.id
        throw(ErrorException("Incorrect ID returned"))
    end
    # Read the Ctrl
    ctrl = Ref(zero(UInt8))
    nbytes = Int(sp_blocking_read(magician.port, ctrl, 1, magician.timeout[]))
    if nbytes < 1
        throw(ErrorException(TIMEOUT_MSG))
    end
    # Check the Ctrl
    if ((ctrl & 0x1) > 0) != cmd.rw
        throw(ErrorException("Incorrect rw returned"))
    end
    if ((ctrl & 0x2) > 0) != queue
        throw(ErrorException("Incorrect isQueued returned"))
    end
    # Read the rest of the payload
    nbytes, payload = Int(sp_blocking_read(magician.port, sz[] - 0x2, magician.timeout[]))
    if nbytes < (sz[] - 0x2)
        throw(ErrorException(TIMEOUT_MSG))
    end
    # Read the checksum
    checksum = Ref(zero(UInt8))
    nbytes = Int(sp_blocking_read(magician.port, checksum, 1, magician.timeout[]))
    if nbytes < 1
        throw(ErrorException(TIMEOUT_MSG))
    end
    # Check the checksum
    checksum_actual = -id[] - ctrl[]
    for x in payload
        checksum_actual -= x
    end
    if checksum_actual != checksum[]
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
    (!cmd.allow_queue && queue) && throw(ArgumentError("Command cannot be queued"))
    sz = payload_size(cmd, payload)
    io = IOBuffer(; sizehint=sz + 4)
    write(io, 0xaaaa)
    write(io, sz)
    write(io, id)
    write(io, UInt8(cmd.rw) | (UInt8(queue) << 1))
    if S <: Tuple
        mapfoldl(x -> write(io, x), +, payload; init=0)
    else
        write(io, payload)
    end
    seek(io, 4)
    checksum = 0x0
    for i in Base.OneTo(sz)
        checksum -= read(io, UInt8)
    end
    write(io, checksum)
    return take!(io)
end

unpack_payload(cmd::Command{<:Any,Vector{UInt8}}, payload) = payload

function unpack_payload(cmd::Command{<:Any,R}, payload) where {R<:Tuple}
    io = IOBuffer(payload)
    return Tuple(read(io, r) for r in R)
end

unpack_payload(cmd::Command{<:Any,R}, payload) where {R} = read(IOBuffer(payload), R)

# Commands - Device information
const set_device_sn = Command(UInt8(0), true, false, String, ())
const get_device_sn = Command(UInt8(0), false, false, (), String)
const set_device_name = Command(UInt8(1), true, false, String, ())
const get_device_name = Command(UInt8(1), false, false, (), String)
const get_device_version = Command(UInt8(2), false, false, (), (UInt8, UInt8, UInt8))
const set_device_with_l = Command(UInt8(3), true, false, (UInt8, UInt32), ())
const get_device_with_l = Command(UInt8(3), false, false, (), (UInt8, UInt32))
const get_device_time = Command(UInt8(4), false, false, (), UInt32)
const get_device_id = Command(UInt8(5), false, false, (), (UInt32, UInt32, UInt32))

# Commands - Real-time pose
const get_pose = Command(UInt8(10), false, false, (), (Float32, Float32, Float32, Float32, Float32, Float32, Float32, Float32))
const reset_pose = Command(UInt8(11), true, false, (UInt8, Float32, Float32), ())
const get_pose_l = Command(UInt8(13), false, false, (), Float32)

# Commands - Alarm
const get_alarms_state = Command(20, false, false, (), Vector{UInt8})
const clear_all_alarms_state = Command(21, true, false, (), ())

# Commands - Homing
const set_home_params = Command(30, true, true, (Float32, Float32, Float32, Float32), ())
const get_home_params = Command(30, false, false, (), (Float32, Float32, Float32, Float32))
const set_home_cmd = Command(31, true, true, UInt32, ())
const set_auto_leveling = Command(32, true, true, (UInt8, Float32), ())
const get_auto_leveling = Command(32, false, false, (), (UInt8, Float32))

# Commands - Handhold teaching
const set_hht_trig_mode = Command(40, true, false, )

end
