// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotMap {

    // CAN motor addresses
    public static final int kFrontLeftDriveCANSparkMaxMotor = 4;
    public static final int kBackLeftDriveCANSparkMaxMotor = 2;
    public static final int kBackRightDriveCANSparkMaxMotor = 3;
    public static final int kFrontRightDriveCANSparkMaxMotor = 1;
    public static final int kFrontLeftTurnCANSparkMaxMotor = 14;
    public static final int kBackLeftTurnCANSparkMaxMotor = 12;
    public static final int kBackRightTurnCANSparkMaxMotor = 13;
    public static final int kFrontRightTurnCANSparkMaxMotor = 11;
    public static final int kLeftClimbCANSparkMaxMotor = 21;
    public static final int kRightClimbCANSparkMaxMotor = 22;
    public static final int kLeftShooterCANSparkMaxWheel = 23;
    public static final int kRightShooterCANSparkMaxWheel = 24;

    public static final int kIntakeVictorSpMotor = 1;
    public static final int kWallSpacerVictorSpMotor = 2;

    // PWM motor addresses

    // Servos
    public static final int kLeftAmpServo = 1;
    public static final int kRightAmpServo = 0;
    public static final int kLeftWallIntake = 2;
    public static final int kRightWallIntake = 3;

    // REV Pneumatic Hub solenoid addresses

    // Digital IO addresses

    // Operator input USB ports
    public static final int kLeftJoystick = 0;
    public static final int kRightJoystick = 1;
    public static final int kButtonBox = 2;

    // Driver Buttons
    public static final int kRightTriggerFire = 1;
    public static final int kRightToggleIntakeButton = 2;
    public static final int kRightLockWheels = 3;

    // Right Joystick Test
    public static final int kResetNavXButton = 10;

    // Co-Piolet Button board

    // Zeroed values, should be in radians
    // source is google document in Electrical for team - module data
    public static final double kZeroedFrontLeft = 5.171;    // for FL module 4
    public static final double kZeroedFrontRight = 2.668;   // for FR module 1
    public static final double kZeroedBackLeft = 5.429;     // for BL module 2
    public static final double kZeroedBackRight = 3.731;    // for BR module 5

    public static final int kFrontLeftCANCoder = 4;     // for FL module 4
    public static final int kBackLeftCANCoder = 2;      //for BL module 2
    public static final int kFrontRightCANCoder = 1;    // for FR module 1
    public static final int kBackRightCANCoder = 5;     //for BR module 5
};