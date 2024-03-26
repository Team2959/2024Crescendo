// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cwtech.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class AprilTagPID
{
    private DriveSubsystem m_driveSubsystem;
    private double kSpeedKp = 0.38;
    private double kRotationKp = 0.035;
    private double kSpeedKi = 0;
    private double kRotationKi = 0;
    private double kSpeedKd = 0;
    private double kRotationKd = 0;

    PIDController m_rotationController = new PIDController(kRotationKp, kRotationKi, kRotationKd);
    PIDController m_xSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);
    PIDController m_zSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);

    public AprilTagPID(DriveSubsystem driveSubsystem)
    {
        m_driveSubsystem = driveSubsystem;

        SmartDashboard.putNumber("AprilTag/kP Speed", kSpeedKp);
        SmartDashboard.putNumber("AprilTag/kP Rotation", kRotationKp);
        SmartDashboard.putNumber("AprilTag/kI Speed", kSpeedKi);
        SmartDashboard.putNumber("AprilTag/kI Rotation", kRotationKi);
        SmartDashboard.putNumber("AprilTag/kD Speed", kSpeedKd);
        SmartDashboard.putNumber("AprilTag/kD Rotation", kRotationKd);
    }

    public void updatePID()
    {
        kSpeedKp = SmartDashboard.getNumber("AprilTag/kP Speed", kSpeedKp);
        kRotationKp = SmartDashboard.getNumber("AprilTag/kP Rotation", kRotationKp);
        kSpeedKi = SmartDashboard.getNumber("AprilTag/kI Speed", kSpeedKi);
        kRotationKi = SmartDashboard.getNumber("AprilTag/kI Rotation", kRotationKi);
        kSpeedKd = SmartDashboard.getNumber("AprilTag/kD Speed", kSpeedKd);
        kRotationKd = SmartDashboard.getNumber("AprilTag/kD Rotation", kRotationKd);
    }

    public void setTargetPosition(double xPosition, double zPosition, double rotation)
    {
        m_xSpeedController.setSetpoint(xPosition);
        m_zSpeedController.setSetpoint(zPosition);
        m_rotationController.setSetpoint(rotation);
    }

    private double zSpeed(double tz)
    {
        double targetingForwardSpeed = m_zSpeedController.calculate(tz);
        targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
        return targetingForwardSpeed;
    }

    private double ySpeed(double tx)
    {
        double targetingForwardSpeed = m_xSpeedController.calculate(tx);
        targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    private double rotationTarget()
    {
        double botAngle = m_driveSubsystem.getAngle().getDegrees();
        double remappedAngle = remapAngle(botAngle);
        double targetingAngularVelocity = m_rotationController.calculate(remappedAngle);
        targetingAngularVelocity *= DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;

        return targetingAngularVelocity;
    }

    public void driveToTarget(double tx, double tz)
    {
        m_driveSubsystem.drive(-zSpeed(tz), ySpeed(tx), rotationTarget(), false);
    }

    private double remapAngle(double fromNavX)
    {
        double remapAngle = fromNavX % 360;
        if (remapAngle <= -45) 
        {
            remapAngle = remapAngle + 360;
        }
        return remapAngle;
    }
}
