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
    public static final int kBackRigvhtModule = 3;
    public static final int kFrontRightModule = 1;
    // unused module - 5

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
        // Amp
    public static final int kAmpNEO = 25;

    // PWM motor addresses
        // Servos

    // Analog Input addresses
    public static final int kAmpStringPotAnalog = 1;

    // REV Pneumatic Hub solenoid addresses

    // Digital IO addresses
    public static final int kNoteDetectorDigitalInput = 1;
    public static final int kWallSpacerExtendedDigitalInput = 2;
    public static final int kWallSpacerRetractedDigitalInput = 3;
    public static final int kbeastEyesDigitalOutput = 4;

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;

    // Driver Buttons
    public static final int kRightTriggerFire = 1;
    public static final int kRightToggleIntakeButton = 2;
    public static final int kRightLockWheels = 3;
    public static final int kRightResetNavXButton = 10;
    public static final int kLeftExtendClimbButton = 8; //7;
    public static final int kLeftRetractClimbButton = 9; //9;
    public static final int kLeftLatchClimbButton = 5; //11;
    public static final int kTrapAlignButton = 3;//8;
    public static final int kRightAmpAlign = 4;

    // Co-Piolt Button board
    public static final int kExtendAmpAssistPleaseButton = 1;
    public static final int kRetractAmpAssistPleaseButton = 3;
    public static final int kAmpShooterVelocityControl = 3;
    public static final int kTrapShooterVelocityControl = 11;
    public static final int kReverseIntake = 6;
    public static final int kCenterSpeakerShooterVelocityControl = 8;
    public static final int kLeftSpeakerShooterVelocityControl = 12;
    public static final int kRightSpeakerShooterVelocityControl = 9;
    public static final int kFireNote = 5;
    public static final int kAmpShootVelocity = 10;


    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kDegreesToRadians = Math.PI * 2.0 / 360.0;
    public static final double kZeroedFrontLeft = 295.0 * kDegreesToRadians;    // for FL module 4
    public static final double kZeroedFrontRight = 136.9 * kDegreesToRadians;   // for FR module 1
    public static final double kZeroedBackLeft = 302.2 * kDegreesToRadians;     // for BL module 2
    public static final double kZeroedBackRight = 69.4 * kDegreesToRadians;    // for BR module 3
};