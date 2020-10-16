"""
    Direct

This module exports all the direct interface functions. This is a low-level interface to the
Dobot that directly exposes all Dobot functionality.
"""
module Direct

using ..DobotMagician: DobotMagician

# Automatically export all Commands
for name in names(DobotMagician; all=true)
    if getproperty(DobotMagician, name) isa DobotMagician.Command
        eval(:(const $name = DobotMagician.$name))
        eval(:(export $name))
    end
end

end
