// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // Swerve Motor Assemblies - SparkMax
    // Drive motor CAN == module #
    // Steer motor CAN == module # + 10
    // CAN coder CAN == module #
    public static final int kFrontLeftModule = 4;
    public static final int kBackLeftModule = 2;
    public static final int kBackRigvhtModule = 5;
    public static final int kFrontRightModule = 1;
    // unused module - 3

    // CAN motor addresses
        // SparkMax
    public static final int kLeftClimbCANSparkMaxMotor = 21;
    public static final int kRightClimbCANSparkMaxMotor = 22;
    public static final int kLeftShooterCANSparkMaxWheel = 23;
    public static final int kRightShooterCANSparkMaxWheel = 24;
        // Victor SPX
    public static final int kIntakeVictorSpMotor = 1;
        // Talon SRX
    public static final int kWallSpacerTalonSrxMotor = 2;

    // PWM motor addresses
        // Servos
    public static final int kLeftAmpServo = 9;
    public static final int kRightAmpServo = 8;

    // Analog Input addresses
    public static final int kAmpStringPotAnalog = 1;

    // REV Pneumatic Hub solenoid addresses

    // Digital IO addresses
    public static final int kNoteDetectorDigitalInput = 1;
    public static final int kWallSpacerExtendedDigitalInput = 2;
    public static final int kWallSpacerRetractedDigitalInput = 3;

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;

    // Driver Buttons
    public static final int kRightTriggerFire = 1;
    public static final int kRightToggleIntakeButton = 2;
    public static final int kRightLockWheels = 3;
    public static final int kRightResetNavXButton = 10;
    public static final int kLeftExtendClimbButton = 7;
    public static final int kLeftRetractClimbButton = 11;

    // Co-Piolet Button board
    public static final int kExtendAmpAssistPleaseButton = 1;
    public static final int kRetractAmpAssistPleaseButton = 10;
    public static final int kAmpShooterVelocityControl = 3;
    public static final int kTrapShooterVelocityControl = 5;
    public static final int kReverseIntake = 6;
    public static final int kCenterSpeakerShooterVelocityControl = 8;
    public static final int kLeftSpeakerShooterVelocityControl = 12;
    public static final int kRightSpeakerShooterVelocityControl = 9;
    public static final int kLoadFromSourceButton = 7;
    public static final int kWallSpacerExtendButton = 2;
    public static final int kWallSpacerRetractButton = 11;

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kZeroedFrontLeft = 5.171;    // for FL module 4
    public static final double kZeroedFrontRight = 2.668;   // for FR module 1
    public static final double kZeroedBackLeft = 5.429;     // for BL module 2
    public static final double kZeroedBackRight = 3.731;    // for BR module 5
};