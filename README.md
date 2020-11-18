# DobotMagician

[![Build Status](https://github.com/dawbarton/DobotMagician.jl/workflows/CI/badge.svg)](https://github.com/dawbarton/DobotMagician.jl/actions)
[![codecov](https://codecov.io/gh/dawbarton/DobotMagician.jl/branch/main/graph/badge.svg?token=9kDsXHRQMq)](https://codecov.io/gh/dawbarton/DobotMagician.jl)

An interface to the [Dobot Magician](https://dobot.cc/) robotic arm. This package uses [LibSerialPort.jl](https://github.com/JuliaIO/LibSerialPort.jl) to interface directly with the Dobot and so only the [USB serial port driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) is required. (If Dobot Studio is installed, the driver is automatically installed.)

The interface follows the Dobot communication protocol v1.1.5.

## Usage

There are two interfaces implemented for the Dobot: a direct, low-level interface and a simpler, high-level interface. The direct interface simply calls Dobot functions directly whereas the high-level interface simplifies some of the function calls but only exposes a small amount of the functionality.

### Direct interface

To use the direct interface, import the package and the `Direct` submodule. All the functions specified in the Dobot communication protocol will be available. A list of all the available functions can be obtained using `names(DobotMagician.Direct)`.

Each of the available functions provides a short description and details of the payload. To see the details simply call `show` on the function (or just type the function name into the REPL). Due to the way these functions are implemented, the docstrings are not used.

*Note: not all functions have been tested.*

```julia
using DobotMagician
using DobotMagician.Direct

# Connect to the Dobot (automatically detect the port)
dobot = DobotMagician.Magician()

# Get the Dobot version
ver = get_device_version(dobot)

# Move the arm (mode 2=MOVL_XYZ: linear motion, cartesian end point)
set_ptp_cmd(dobot, (2, (200, 0, 0, 0)))
sleep(2)

# Turn on the suction cup
set_end_effector_suction_cup(dobot, (1, 1))

# Move the arm (mode 7=MOVL_INC: relative linear motion, cartesian end point)
# Queue the commands so they happen sequentially
set_ptp_cmd(dobot, (7, (20, 0, 0, 0)); queue=true)
set_ptp_cmd(dobot, (7, (0, 20, 0, 0)); queue=true)
set_ptp_cmd(dobot, (7, (0, 0, 20, 0)); queue=true)

# Turn off the suction cup
set_end_effector_suction_cup(dobot, (1, 0); queue=true)
```

### Simple interface

To use the simple interface, import the package and the `Simple` submodule. Currently provided functions are

- `move_to`
- `rmove_to`
- `pose`
- `pose_l`
- `end_effector`
- `laser`
- `gripper`
- `suction_cup`
- `wait_for`

Each has a docstring.

*Note: not all functions have been tested.*

```julia
using DobotMagician
using DobotMagician.Simple

# Connect to the Dobot (automatically detect the port)
dobot = DobotMagician.Magician()

# Move the arm (linear motion, cartesian end point)
move_to(dobot, XYZR(200, 0, 0, 0))
sleep(2)

# Turn on the suction cup
suction_cup(dobot, true)

# Move the arm
# Queue the commands so they happen sequentially
rmove_to(dobot, XYZR(20, 0, 0, 0); queue=true)
rmove_to(dobot, XYZR(0, 20, 0, 0); queue=true)
rmove_to(dobot, XYZR(0, 0, 20, 0); queue=true)

# Turn off the suction cup
suction_cup(dobot, false; queue=true)
```
