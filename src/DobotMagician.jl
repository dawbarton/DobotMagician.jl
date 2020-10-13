module DobotMagician

using LibSerialPort
using DocStringExtensions

export Magician
export connect, disconnect

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
    return
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
_extract_pair(a::NTuple{N, <:Pair}) where N = (Tuple(b[1] for b in a), Tuple{(b[2] for b in a)...})

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

function Base.show(io::IO, mime::MIME"text/plain", cmd::Command)

end

function payload_size(payload::Vector{UInt8})
    return UInt8(2 + length(payload))
end

function payload_size(payload::String)
    if !isascii(payload)
        throw(ArgumentError("Strings must only contain ASCII characters"))
    end
    return UInt8(2 + length(payload))
end

function payload_size(payload::Tuple)
    return UInt8(mapreduce(sizeof, +, payload; init=2))
end

payload_size(payload) = UInt8(2 + sizeof(payload))

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
    # Uses dispatch to enforce correct payload types
    (!cmd.allow_queue && queue) && throw(ArgumentError("Command cannot be queued"))
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
# const get_pose = Command(10, false, false, (), (Float32, Float32, Float32, Float32, Float32, Float32, Float32, Float32))
# const reset_pose = Command(11, true, false, (UInt8, Float32, Float32), ())
# const get_pose_l = Command(13, false, false, (), Float32)

# # Commands - Alarm
# const get_alarms_state = Command(20, false, false, (), Vector{UInt8})
# const clear_all_alarms_state = Command(21, true, false, (), ())

# # Commands - Homing
# const set_home_params = Command(30, true, true, (Float32, Float32, Float32, Float32), ())
# const get_home_params = Command(30, false, false, (), (Float32, Float32, Float32, Float32))
# const set_home_cmd = Command(31, true, true, UInt32, ())
# const set_auto_leveling = Command(32, true, true, (UInt8, Float32), ())
# const get_auto_leveling = Command(32, false, false, (), (UInt8, Float32))

# # Commands - Handhold teaching
# const set_hht_trig_mode = Command(40, true, false)

end
