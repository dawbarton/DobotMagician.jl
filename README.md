# DobotMagician

[![Build Status](https://github.com/dawbarton/DobotMagician.jl/workflows/CI/badge.svg)](https://github.com/dawbarton/DobotMagician.jl/actions)
[![Coverage](https://codecov.io/gh/dawbarton/DobotMagician.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/dawbarton/DobotMagician.jl)

An interface to the Dobot Magician robotic arm. This package uses [LibSerialPort.jl](https://github.com/JuliaIO/LibSerialPort.jl) to interface directly with the Dobot and so only the [USB serial port driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) is required. (If Dobot Studio is installed, the driver is automatically installed.)
