// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package cwtech.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class AprilTagPID
{
    private double kSpeedKp = 0.38;
    private double kRotationKp = 0.035;
    private double kSpeedKi = 0;
    private double kRotationKi = 0;
    private double kSpeedKd = 0;
    private double kRotationKd = 0;
 

    PIDController m_rotationController = new PIDController(kRotationKp, kRotationKi, kRotationKd);
    PIDController m_xSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);
    PIDController m_ySpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);

    public AprilTagPID()
    {
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
}
  // from 2023 arm rotation subsystem
  // private ProfiledPIDController m_armRotatorMotorProfiledPidController = new ProfiledPIDController(kArmRotatorP,
  //     kArmRotatorI, kArmRotatorD,
  //     new Constraints(kArmRotatorMaxVelocity, kArmRotatorMaxAccel));

  // double rawProfiled = m_armRotatorMotorProfiledPidController.calculate(getArmAngle());
  //     m_armRotatorMotor.set(rawProfiled);

  // // profiled PID control
  //   m_armRotatorMotorProfiledPidController.setGoal(degrees);

