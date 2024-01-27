// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSubsystem extends SubsystemBase {
  
  private CANSparkMax m_leftClimbMotor;
  private CANSparkMax m_rightClimbMotor;
  private SparkPIDController m_climbLeftPidController;
  private SparkPIDController m_climbRightPidController;

  private static final double kClimbP = 0.01;
  private static final double kClimbI = 0.0;
  private static final double kClimbD = 0.0;
  private static final double kClimbFF = 0;
  private static final double kClimbIZone = 0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_leftClimbMotor = new CANSparkMax(RobotMap.kLeftClimbCANSparkMaxMotor, CANSparkMax.MotorType.kBrushless);
    m_rightClimbMotor = new CANSparkMax(RobotMap.kRightClimbCANSparkMaxMotor, CANSparkMax.MotorType.kBrushless);

    m_leftClimbMotor.restoreFactoryDefaults();
    m_rightClimbMotor.restoreFactoryDefaults();

    m_leftClimbMotor.getEncoder();
    m_rightClimbMotor.getEncoder();

    m_leftClimbMotor.setIdleMode(IdleMode.kBrake);
    m_rightClimbMotor.setIdleMode(IdleMode.kBrake);

    m_climbLeftPidController = m_leftClimbMotor.getPIDController();
    m_climbLeftPidController.setP(kClimbP);
    m_climbLeftPidController.setI(kClimbI);
    m_climbLeftPidController.setD(kClimbD);
    m_climbLeftPidController.setFF(kClimbFF);
    m_climbLeftPidController.setIZone(kClimbIZone);

    m_climbRightPidController = m_rightClimbMotor.getPIDController();
    m_climbRightPidController.setP(kClimbP);
    m_climbRightPidController.setI(kClimbI);
    m_climbRightPidController.setD(kClimbD);
    m_climbRightPidController.setFF(kClimbFF);
    m_climbRightPidController.setIZone(kClimbIZone);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
