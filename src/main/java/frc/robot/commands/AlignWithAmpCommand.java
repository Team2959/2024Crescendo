// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import cwtech.util.AprilTagPID;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignWithAmpCommand extends Command {
  /** Creates a new AlignWithAmpCommand. */
  private DriveSubsystem m_driveSubsystem;
  private final AprilTagPID m_AprilTagPID;

  private double m_targetRotaion = 0;

  public AlignWithAmpCommand(DriveSubsystem driveSubsystem, AprilTagPID aprilTagPID) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_AprilTagPID = aprilTagPID;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var tid = (int)LimelightHelpers.getFiducialID(Vision.kLimeLightName);
    if (tid == 6)
      m_targetRotaion = 270;
    else if (tid == 5)
      m_targetRotaion = 90;
    else
      m_targetRotaion = 0;

   m_AprilTagPID.setTargetPosition(0, 0, m_targetRotaion);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var tid = (int)LimelightHelpers.getFiducialID(Vision.kLimeLightName);

      if (tid != 5 && tid != 6)
      {
        System.out.println("No TiD found");
        end(true);
        return;
      }

      m_AprilTagPID.driveToTargetNoZ();
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
  }
}
