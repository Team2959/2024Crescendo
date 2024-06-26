// Copyright (c) FIRST and other WPILib contributors.
// What? Who would hide an EasterEgg in here? 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import cwtech.util.AprilTagPID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Vision;

public class AlignWithTrapCommand extends Command {
  /** Creates a new AlignWithTrapCommand. */
  private DriveSubsystem m_driveSubsystem;
  private final AprilTagPID m_AprilTagPID;
  // private double m_lastRotation;
  // private double m_lastDeltaX;
  // private double m_lastDeltaZ;
  private double m_targetZ = 1.348;
  private double m_targetRotaion = 0;
  // private double kSpeedKp = 0.38;
  // private double kRotationKp = 0.035;

  public AlignWithTrapCommand(DriveSubsystem driveSubsystem, AprilTagPID aprilTagPID) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_AprilTagPID = aprilTagPID;

    addRequirements(driveSubsystem);

    SmartDashboard.putNumber("AprilTag/target Z", m_targetZ);
    // SmartDashboard.putNumber("AprilTag/kP Speed", kSpeedKp);
    // SmartDashboard.putNumber("AprilTag/kP Rotation", kRotationKp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var tid = (int)LimelightHelpers.getFiducialID(Vision.kLimeLightName);
    if (tid == 15 || tid == 11)
      m_targetRotaion = 120;
    else if (tid == 16 || tid == 12)
      m_targetRotaion = 240;
    else
      m_targetRotaion = 0;

   m_AprilTagPID.setTargetPosition(0, m_targetZ, m_targetRotaion);
    // m_lastRotation = 5;
    // m_lastDeltaX = 5;
    // m_lastDeltaZ = 5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      var tid = (int)LimelightHelpers.getFiducialID(Vision.kLimeLightName);

      if (tid < 10)
      {
        System.out.println("No TiD found");
        end(true);
        return;
      }

      // m_lastRotation = rotationTarget();

      // double xSpeed = xSpeed();
      // double ySpeed = ySpeed();
      // EasterEgg

      m_AprilTagPID.driveToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
       m_driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
    // return m_AprilTagPID.atTargetPosition();
      // return m_lastRotation < 0.5 && m_lastDeltaX < 0.05 && m_lastDeltaZ < 0.05;
  }

  // private double ySpeed()
  // {
  //   double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace(Vision.kLimeLightName);
  //   var tx = robotSpace[0];
  //   double targetingForwardSpeed = m_lastDeltaX = tx * kSpeedKp;
  //   targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
  //   return targetingForwardSpeed;
  // }

  // private double xSpeed()
  // {
  //   double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace(Vision.kLimeLightName);
  //   var tz = robotSpace[2];
  //   m_lastDeltaZ = m_targetZ - tz;
  //   double targetingForwardSpeed = m_lastDeltaZ * kSpeedKp;
  //   targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
  //   targetingForwardSpeed *= -1.0;
  //   return targetingForwardSpeed;
  // }

  // private double rotationTarget()
  // {
  //   double botAngle = m_driveSubsystem.getAngle().getDegrees();
  //   double remappedAngle = remapAngle(botAngle);
  //   double targetingAngularVelocity = (m_targetRotaion - remappedAngle)  * kRotationKp;
  //   targetingAngularVelocity *= DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;

  //   return targetingAngularVelocity;
  // }

  // private double remapAngle(double fromNavX)
  // {
  //   double remapAngle = fromNavX % 360;
  //   if (remapAngle <= -45) 
  //   {
  //     remapAngle = remapAngle + 360;
  //   }
  //   return remapAngle;
  // }
}
