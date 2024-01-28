// Copyright (c) FIRST and o  private SparkPIDController m_shooterPidController;
//ther WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_leftShooterWheel;
  private CANSparkMax m_rightShooterWheel;
  private SparkRelativeEncoder m_shooterEncoder;

  private SparkPIDController m_shooterPidController;


  private static final double kShooterP = 0.01;
  private static final double kShooterI = 0.0;
  private static final double kShooterD = 0.0;
  private static final double kShooterFF = 0;
  private static final double kShooterIZone = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_leftShooterWheel = new CANSparkMax(RobotMap.kLeftShooterCANSparkMaxWheel, CANSparkMax.MotorType.kBrushless);
    m_rightShooterWheel = new CANSparkMax(RobotMap.kRightShooterCANSparkMaxWheel, CANSparkMax.MotorType.kBrushless);

    m_leftShooterWheel.restoreFactoryDefaults();

    m_leftShooterWheel.setIdleMode(IdleMode.kCoast);
    m_rightShooterWheel.setIdleMode(IdleMode.kCoast);

    m_rightShooterWheel.follow(m_leftShooterWheel);
    m_rightShooterWheel.setInverted(true);

    m_shooterPidController = m_leftShooterWheel.getPIDController();
    m_shooterPidController.setP(kShooterP);
    m_shooterPidController.setI(kShooterI);
    m_shooterPidController.setD(kShooterD);
    m_shooterPidController.setFF(kShooterFF);
    m_shooterPidController.setIZone(kShooterIZone);

    m_shooterEncoder = (SparkRelativeEncoder)m_leftShooterWheel.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // how to access the current shooter velocity
    // m_shooterEncoder.getVelocity();
  }

  public void ShooterVelocity(double velocity)
  {
    m_shooterPidController.setReference(velocity, ControlType.kVelocity);
  }
}
