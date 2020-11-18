using DobotMagician: DobotMagician, construct_command, unpack_payload
using DobotMagician.Direct
using DobotMagician.Simple

using Test

try
    dobot = DobotMagician.Magician()
    global HAS_DOBOT = true
catch
    global HAS_DOBOT = false
end

@testset "Offline tests" begin
    # Test a range of input types
    @test construct_command(get_device_name, (), false) == UInt8[0xaa, 0xaa, 0x02, 0x01, 0x00, 0xff]
    @test_throws ArgumentError construct_command(get_device_name, (), true)
    @test construct_command(set_device_name, ("Hello",), false) == UInt8[0xaa, 0xaa, 0x08, 0x01, 0x01, 0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x00, 0x0a]
    @test construct_command(set_ptp_po_cmd, (0x0, (0.2, 0.4, 0.6, 0.8), ((1, 2, 3), (4, 5, 6))), true) == UInt8[0xaa, 0xaa, 0x1b, 0x58, 0x03, 0x00, 0xcd, 0xcc, 0x4c, 0x3e, 0xcd, 0xcc, 0xcc, 0x3e, 0x9a, 0x99, 0x19, 0x3f, 0xcd, 0xcc, 0x4c, 0x3f, 0x01, 0x02, 0x00, 0x03, 0x04, 0x05, 0x00, 0x06, 0x1b]
    # Test a range of output types
    @test unpack_payload(get_device_name, UInt8[0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x00])[1] == "Hello"
    @test unpack_payload(get_jog_coordinate_params, UInt8[0x00, 0x00, 0x70, 0x42, 0x00, 0x00, 0x70, 0x42, 0x00, 0x00, 0x70, 0x42, 0x00, 0x00, 0x70, 0x42, 0x00, 0x00, 0x70, 0x42, 0x00, 0x00, 0x70, 0x42, 0x00, 0x00, 0x70, 0x42, 0x00, 0x00, 0x70, 0x42, 0x29]) == ((60.0f0, 60.0f0, 60.0f0, 60.0f0), (60.0f0, 60.0f0, 60.0f0, 60.0f0))
end

if HAS_DOBOT
    # No set functions are tested since it could be physically dangerous to manipulate an
    # unknown robotic arm
    @testset "Direct interface (get only)" begin
        # Commands - Device information
        @test !isempty(get_device_sn(dobot))
        @test !isempty(get_device_name(dobot))
        @test !isempty(get_device_version(dobot))
        @test !isempty(get_device_with_l(dobot))
        @test !isempty(get_device_time(dobot))
        @test !isempty(get_device_info(dobot))

        # Commands - Real-time pose
        @test !isempty(get_pose(dobot))
        @test !isempty(get_kinematics(dobot))
        @test !isempty(get_pose_l(dobot))

        # Commands - Alarm
        @test !isempty(get_alarms_state(dobot))

        # Commands - Homing
        @test !isempty(get_home_params(dobot))
        @test !isempty(get_auto_leveling(dobot))

        # Commands - Handhold teaching
        @test !isempty(get_hht_trig_mode(dobot))
        @test !isempty(get_hht_trig_output_enabled(dobot))
        @test !isempty(get_hht_trig_output(dobot))

        # Commands - Arm orientation
        @test !isempty(get_arm_orientation(dobot))

        # Commands - End effector
        @test !isempty(get_end_effector_params(dobot))
        @test !isempty(get_end_effector_laser(dobot))
        @test !isempty(get_end_effector_suction_cup(dobot))
        @test !isempty(get_end_effector_gripper(dobot))

        # Commands - Jog
        @test !isempty(get_jog_joint_params(dobot))
        @test !isempty(get_jog_coordinate_params(dobot))
        @test !isempty(get_jog_common_params(dobot))

        # Commands - PTP
        @test !isempty(get_ptp_joint_params(dobot))
        @test !isempty(get_ptp_coordinate_params(dobot))
        @test !isempty(get_ptp_jump_params(dobot))
        @test !isempty(get_ptp_common_params(dobot))
        @test !isempty(get_ptp_jump2_params(dobot))

        # Commands - CP
        @test !isempty(get_cp_params(dobot))

        # Commands - Queued execution control commands
        @test !isempty(get_queued_cmd_current_index(dobot))
        @test !isempty(get_queued_cmd_motion_finish(dobot))
    end

    @testset "Simple interface (get only)" begin
        @test pose(dobot) isa NamedTuple{(:xyzr, :joint),Tuple{XYZR,Joint}}
        @test pose_l(dobot) isa NamedTuple{(:xyzrl, :jointl),Tuple{XYZRL,JointL}}
        @test laser(dobot) isa Bool
        @test gripper(dobot) isa Bool
        @test suction_cup(dobot) isa Bool
        @test typeof(alarm_description(dobot)) in (Nothing, String)
    end
end
