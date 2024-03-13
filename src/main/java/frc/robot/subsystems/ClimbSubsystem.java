// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClimbSubsystem extends SubsystemBase {
  
  private CANSparkMax m_leftClimbMotor;
  private CANSparkMax m_rightClimbMotor;
  private SparkPIDController m_climbLeftPidController;
  private SparkPIDController m_climbRightPidController;
  private SparkRelativeEncoder m_leftClimbEncoder;
  private SparkRelativeEncoder m_rightClimbEncoder;

  private static final double kClimbP = 0.15;
  private static final double kClimbI = 0.0;
  private static final double kClimbD = 0.0;
  private static final double kClimbFF = 0;
  private static final double kClimbIZone = 0;

  private double m_lastLeftTarget = 0;
  private double m_lastRightTarget = 0;
  private double m_extendDistance = 142;
  private double m_retractDistance = 12;
  private double m_latchDistance = 2;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_leftClimbMotor = new CANSparkMax(RobotMap.kLeftClimbCANSparkMaxMotor, CANSparkMax.MotorType.kBrushless);
   m_rightClimbMotor = new CANSparkMax(RobotMap.kRightClimbCANSparkMaxMotor, CANSparkMax.MotorType.kBrushless);

    m_leftClimbMotor.restoreFactoryDefaults();
    m_rightClimbMotor.restoreFactoryDefaults();

    m_leftClimbEncoder = (SparkRelativeEncoder)m_leftClimbMotor.getEncoder();
    m_rightClimbEncoder = (SparkRelativeEncoder)m_rightClimbMotor.getEncoder();

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

  public void smartDashboardInit()
  {
    SmartDashboard.putNumber(getName() + "/left current position", m_leftClimbEncoder.getPosition());
    // SmartDashboard.putNumber(getName() + "/left target position", 0);
    SmartDashboard.putNumber(getName() + "/right current position", m_rightClimbEncoder.getPosition());
    // SmartDashboard.putNumber(getName() + "/right target position", 0);
    // SmartDashboard.putBoolean(getName() + "/Test Go To Targets", false);

    // SmartDashboard.putBoolean(getName() + "/Update PIDs", false);
    // SmartDashboard.putNumber(getName() + "/climb P", kClimbP);
    // SmartDashboard.putNumber(getName() + "/climb I", kClimbI);
    // SmartDashboard.putNumber(getName() + "/climb D", kClimbD);
    // SmartDashboard.putNumber(getName() + "/climb FF", kClimbFF);

    // SmartDashboard.putNumber(getName() + "/Extend Distance", m_extendDistance);
    // SmartDashboard.putNumber(getName() + "/Retract Distance", m_retractDistance);

    // SmartDashboard.putNumber(getName() + "/left velocity", 0);
    // SmartDashboard.putNumber(getName() + "/left motor output", 0);
  }

  public void smartDashboardUpdate()
  {
    SmartDashboard.putNumber(getName() + "/left current position", m_leftClimbEncoder.getPosition());
    SmartDashboard.putNumber(getName() + "/right current position", m_rightClimbEncoder.getPosition());
    // SmartDashboard.putNumber(getName() + "/left velocity", m_leftClimbEncoder.getVelocity());
    // SmartDashboard.putNumber(getName() + "/left motor output", m_leftClimbMotor.get());

    // if (SmartDashboard.getBoolean(getName() + "/Update PIDs", false))
    // {
    //   m_extendDistance = SmartDashboard.getNumber(getName() + "/Extend Distance", m_extendDistance);
    //   m_retractDistance = SmartDashboard.getNumber(getName() + "/Retract Distance", m_retractDistance);

    //   double pGain = SmartDashboard.getNumber(getName() + "/climb P", kClimbP);
    //   double iGain = SmartDashboard.getNumber(getName() + "/climb I", kClimbI);
    //   double dGain = SmartDashboard.getNumber(getName() + "/climb D", kClimbD);
    //   double ffGain = SmartDashboard.getNumber(getName() + "/climb FF", kClimbFF);

    //   m_climbLeftPidController.setP(pGain); 
    //   m_climbLeftPidController.setI(iGain); 
    //   m_climbLeftPidController.setD(dGain); 
    //   m_climbLeftPidController.setFF(ffGain); 

    //   m_climbRightPidController.setP(pGain); 
    //   m_climbRightPidController.setI(iGain); 
    //   m_climbRightPidController.setD(dGain); 
    //   m_climbRightPidController.setFF(ffGain); 

    //   SmartDashboard.putBoolean(getName() + "/Update PIDs", false);
    // }

    // if (SmartDashboard.getBoolean(getName() + "/Test Go To Targets", false))
    // {
    //   double leftTarget = SmartDashboard.getNumber(getName() + "/left target position", m_lastLeftTarget);
    //   double rightTarget = SmartDashboard.getNumber(getName() + "/right target position", m_lastRightTarget);

    //   setLeftTargetPosition(leftTarget);
    //   setRightTargetPosition(rightTarget);

    //   SmartDashboard.putBoolean(getName() + "/Test Go To Targets", false);
    // }
  }

  private void setLeftTargetPosition(double position)
  {
    m_climbLeftPidController.setReference(position, ControlType.kPosition);
    m_lastLeftTarget = position;
  }

  private void setRightTargetPosition(double position)
  {
    m_climbRightPidController.setReference(position, ControlType.kPosition);
    m_lastRightTarget = position;
  }

  public void extendClimbArms()
  {
    setRightTargetPosition(-m_extendDistance);
    setLeftTargetPosition(m_extendDistance);
  }

  public void retractClimbArms()
  {
    setRightTargetPosition(-m_retractDistance);
    setLeftTargetPosition(m_retractDistance);
  }

  public void latchClimbArms()
  {
    setRightTargetPosition(-m_latchDistance);
    setLeftTargetPosition(m_latchDistance);
  }

  public boolean isAtTargetPosition() {
    return Math.abs(m_lastLeftTarget - m_leftClimbEncoder.getPosition()) < 2 &&
           Math.abs(m_lastRightTarget - m_rightClimbEncoder.getPosition()) < 2;
  }

  public void stopAtCurrentPosition() {
    setRightTargetPosition(m_rightClimbEncoder.getPosition());
    setLeftTargetPosition(m_leftClimbEncoder.getPosition());
  }
}
