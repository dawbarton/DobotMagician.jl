module DobotMagician

using LibSerialPort
using DocStringExtensions

export connect, disconnect, send, Magician

struct Magician
    port::Port
    timeout::Base.RefValue{Int}
    buffer::Vector{UInt8}
    iobuffer::Base.GenericIOBuffer{Vector{UInt8}}
    function Magician(port::Port, timeout::Int)
        buffer = Vector{UInt8}(undef, 256)
        return new(port, Ref(timeout), buffer, IOBuffer(buffer))
    end
end
Magician(port::Port) = Magician(port, 1000)  # default timeout 1000ms

struct Command{S,R}
    id::UInt8
    rw::Bool
    allow_queue::Bool
end
Command(id, rw, allow_queue, send, receive) = Command{send,receive}(id, rw, allow_queue)

function command_send_size(cmd::Command{Vector{UInt8}}, payload::Vector{UInt8})
    return UInt8(length(payload))
end
command_send_size(cmd::Command{S}, payload::S) where {S} = mapreduce(sizeof, +, S; init=0x0)

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

function execute_command(magician::Magician, cmd::Command; queue=false)
    nbytes = Int(sp_blocking_read(magician.port, magician.buffer, 3, magician.timeout[]))
    if nbytes < 3
        throw(ErrorException("Serial port errored or timed out waiting for response"))
    end
    if (magician.buffer[1] != 0xaa) || (magician.buffer[2] != 0xaa)
        throw(ErrorException("Invalid header"))
    end
    len = magician.buffer[3]
    nbytes = Int(sp_blocking_read(
        magician.port, magician.buffer, len + 1, magician.timeout[]
    ))
    if nbytes < len + 1
        throw(ErrorException("Serial port errored or timed out waiting for response"))
    end
end

function construct_command(cmd::Command{S}, params::S)
    io = IOBuffer()
    write(io, 0xaaaa)
    write(io, 0x2 + mapreduce(sizeof, +, params; init=0x0))
    write(io, id)
    write(io, cmd.rw | (UInt8(!wait) << 1))
    mapfoldl(x -> write(io, x), +, params; init=0)
    seek(io, 6)
    checksum = 0
    while !eof(io)
        checksum -= read(io, UInt8)
    end
    write(io, checksum)
    return take!(io)
end

_unpack_params(io::IO, len::Integer, ::Type{Vector{UInt8}}) = read(io, len)
function _unpack_params(io::IO, ::Integer, param_types::Tuple)
    return Tuple(read(io, param_type) for param_type in param_types)
end
_unpack_params(::IO, ::Integer, ::Tuple{}) = ()
_unpack_params(io::IO, ::Integer, param_types::Tuple{<:Any}) = (read(io, param_types[1]),)
function _unpack_params(io::IO, ::Integer, param_types::Tuple{<:Any,<:Any})
    return (read(io, param_types[1]), read(io, param_types[2]))
end
function _unpack_params(io::IO, ::Integer, param_types::Tuple{<:Any,<:Any,<:Any})
    return (read(io, param_types[1]), read(io, param_types[2]), read(io, param_types[3]))
end
function _unpack_params(io::IO, ::Integer, param_types::Tuple{<:Any,<:Any,<:Any,<:Any})
    return (
        read(io, param_types[1]),
        read(io, param_types[2]),
        read(io, param_types[3]),
        read(io, param_types[4]),
    )
end
function _unpack_params(
    io::IO, ::Integer, param_types::Tuple{<:Any,<:Any,<:Any,<:Any,<:Any}
)
    return (
        read(io, param_types[1]),
        read(io, param_types[2]),
        read(io, param_types[3]),
        read(io, param_types[4]),
        read(io, param_types[5]),
    )
end

function unpack_result(
    result::AbstractVector{UInt8}, id::UInt8, rw::Bool, is_queued::Bool, param_types
)
    avail = length(result)
    io = IOBuffer(result)
    avail < 2 && throw(ErrorException("Result truncated before header"))
    read(io, UInt16) != 0xaaaa && throw(ErrorException("Incorrect header"))
    avail -= 2
    avail < 1 && throw(ErrorException("Result truncated before len"))
    len = read(io, UInt8) - 0x2
    avail -= 1
    avail < 1 && throw(ErrorException("Result truncated before ID"))
    read(io, UInt8) != id && throw(ErrorException("Incorrect ID"))
    avail -= 1
    avail < 1 && throw(ErrorException("Result truncated before ctrl"))
    ctrl = read(io, UInt8)
    avail -= 1
    (ctrl & 0x01) != rw && throw(ErrorException("Incorrect rw"))
    ((ctrl >> 1) & 0x01) != is_queued && throw(ErrorException("Incorrect is_queued"))
    checksum = 0x0
    for i in 1:(len + 1)
        eof(io) && throw(ErrorException("Result truncated before end of params"))
        checksum += read(io, UInt8)
    end
    checksum != 0x0 && throw(ErrorException("Incorrect checksum"))
    seek(io, 6)
    return _unpack_params(io, len, param_types)
end

end
