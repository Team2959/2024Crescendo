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

public class AmpAssistSubsystem extends SubsystemBase {
  private CANSparkMax m_AmpRampNEO; 
  private SparkRelativeEncoder m_AmpRampEncoder;
  private SparkPIDController m_ampPidController;
  
  private double m_lastTarget = 0;
  private static final double kMaxMeasuredExtension = -29.5;
  private static final double kMinMeasuredRetraction = 1;
  public double m_extendDistance = -28.5; 
  private double m_retractDistance = -0.5;

  private static final double kAmpP = 0.4;
  private static final double kAmpI = 0.0;
  private static final double kAmpD = 0.0;
  private static final double kAmpFF = 0; 
  private static final double kAmpIZone = 0;
  private static final double kMaxExtendPower = -0.5;
  private static final double kMaxRetractPower = 0.5;

  // private static final int kSmartMotionSlot = 0;
  // private static final double kAmpMinVelocity = 50;
  // private static final double kAmpMaxVelocity = 145;
  // private static final double kAmpMaxAcceleration = 70;
  // private static final double kAmpClosedLoopError = 0.0;

  public static final int kDefaultSlot = 0;
  public static final int kExtendSlot = 1;

  /** Creates a new AmpShooterSubsystem. */
  public AmpAssistSubsystem() 
  {
    m_AmpRampNEO = new CANSparkMax(RobotMap.kAmpNEO, CANSparkMax.MotorType.kBrushless);

    m_AmpRampNEO.restoreFactoryDefaults();

    m_AmpRampNEO.setIdleMode(IdleMode.kBrake);
 
    m_AmpRampEncoder = (SparkRelativeEncoder)m_AmpRampNEO.getEncoder();

    stopMotor();

    m_ampPidController = m_AmpRampNEO.getPIDController();
    m_ampPidController.setOutputRange(-1.0, kMaxRetractPower, kDefaultSlot);
    m_ampPidController.setP(kAmpP, kDefaultSlot);
    m_ampPidController.setI(kAmpI, kDefaultSlot);
    m_ampPidController.setD(kAmpD, kDefaultSlot);
    m_ampPidController.setFF(kAmpFF, kDefaultSlot);
    m_ampPidController.setIZone(kAmpIZone, kDefaultSlot);

    m_ampPidController.setOutputRange(kMaxExtendPower, kMaxRetractPower, kExtendSlot);
    m_ampPidController.setP(kAmpP, kExtendSlot);
    m_ampPidController.setI(kAmpI, kExtendSlot);
    m_ampPidController.setD(kAmpD, kExtendSlot);
    m_ampPidController.setFF(kAmpFF, kExtendSlot);
    m_ampPidController.setIZone(kAmpIZone, kExtendSlot);

    // m_ampPidController.setSmartMotionMinOutputVelocity(kAmpMinVelocity, kSmartMotionSlot);
    // m_ampPidController.setSmartMotionMaxVelocity(kAmpMaxVelocity, kSmartMotionSlot);
    // m_ampPidController.setSmartMotionMaxAccel(kAmpMaxAcceleration, kSmartMotionSlot);
    // m_ampPidController.setSmartMotionAllowedClosedLoopError(kAmpClosedLoopError, kSmartMotionSlot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotor()
  {
    m_AmpRampNEO.set(0);
  }

  public void directDrivePower()
  {
    var power = SmartDashboard.getNumber(getName() + "/direct power", kMaxExtendPower);
    power = Math.max(-1, Math.min(1, power));
    m_AmpRampNEO.set(power);
    // m_AmpRampNEO.set(kMaxExtendPower);
  }

  public double getAmpPosition()
  {
    return m_AmpRampEncoder.getPosition();
  }

  public boolean isAtPosition()
  {
    return Math.abs(m_lastTarget - getAmpPosition()) < 0.5;
  }

  public void smartDashboardInit()
  {
    SmartDashboard.putNumber(getName() + "/Position", getAmpPosition());
    SmartDashboard.putNumber(getName() + "/velocity", 0);
    SmartDashboard.putNumber(getName() + "/motor output", 0);

    SmartDashboard.putBoolean(getName() + "/Update PIDs", false);
    SmartDashboard.putNumber(getName() + "/amp P", kAmpP);
    SmartDashboard.putNumber(getName() + "/amp I", kAmpI);
    SmartDashboard.putNumber(getName() + "/amp D", kAmpD);
    SmartDashboard.putNumber(getName() + "/amp FF", kAmpFF);

    SmartDashboard.putNumber(getName() + "/Min Output", kMaxExtendPower);
    SmartDashboard.putNumber(getName() + "/Max Output", kMaxRetractPower);

    // SmartDashboard.putNumber(getName() + "/Min Vel", kAmpMinVelocity);
    // SmartDashboard.putNumber(getName() + "/Max Vel", kAmpMaxVelocity);
    // SmartDashboard.putNumber(getName() + "/Max Accel", kAmpMaxAcceleration);
    // SmartDashboard.putNumber(getName() + "/Closed Loop Error", kAmpClosedLoopError);

    SmartDashboard.putNumber(getName() + "/Extend Distance", m_extendDistance);
    SmartDashboard.putNumber(getName() + "/Retract Distance", m_retractDistance);

    SmartDashboard.putBoolean(getName() + "/Test Go To Targets", false);
    SmartDashboard.putNumber(getName() + "/target position", 0);

    SmartDashboard.putNumber(getName() + "/direct power", kMaxExtendPower);
  }

  public void smartDashboardUpdate()
  {
     SmartDashboard.putNumber(getName() + "/Position", getAmpPosition());
    
    SmartDashboard.putNumber(getName() + "/Position", m_AmpRampEncoder.getPosition());
    SmartDashboard.putNumber(getName() + "/velocity", m_AmpRampEncoder.getVelocity());
    SmartDashboard.putNumber(getName() + "/motor output", m_AmpRampNEO.getAppliedOutput());

    if (SmartDashboard.getBoolean(getName() + "/Update PIDs", false))
    {
      m_extendDistance = SmartDashboard.getNumber(getName() + "/Extend Distance", m_extendDistance);
      m_retractDistance = SmartDashboard.getNumber(getName() + "/Retract Distance", m_retractDistance);

      double pGain = SmartDashboard.getNumber(getName() + "/amp P", kAmpP);
      double iGain = SmartDashboard.getNumber(getName() + "/amp I", kAmpI);
      double dGain = SmartDashboard.getNumber(getName() + "/amp D", kAmpD);
      double ffGain = SmartDashboard.getNumber(getName() + "/amp FF", kAmpFF);

      m_ampPidController.setP(pGain, kDefaultSlot); 
      m_ampPidController.setI(iGain, kDefaultSlot); 
      m_ampPidController.setD(dGain, kDefaultSlot); 
      m_ampPidController.setFF(ffGain, kDefaultSlot);

      m_ampPidController.setP(pGain, kExtendSlot); 
      m_ampPidController.setI(iGain, kExtendSlot); 
      m_ampPidController.setD(dGain, kExtendSlot); 
      m_ampPidController.setFF(ffGain, kExtendSlot); 

      // var minOutput = SmartDashboard.getNumber(getName() + "/Min Output", kMaxExtendPower);
      // var maxOutput = SmartDashboard.getNumber(getName() + "/Max Output", kMaxRetractPower);
      // m_ampPidController.setOutputRange(minOutput, maxOutput);

      // var minVel = SmartDashboard.getNumber(getName() + "/Min Vel", kAmpMinVelocity);
      // var maxVel = SmartDashboard.getNumber(getName() + "/Max Vel", kAmpMaxVelocity);
      // var maxAccel = SmartDashboard.getNumber(getName() + "/Max Accel", kAmpMaxAcceleration);
      // var loopError = SmartDashboard.getNumber(getName() + "/Closed Loop Error", kAmpClosedLoopError);

      // m_ampPidController.setSmartMotionMinOutputVelocity(minVel, kSmartMotionSlot);
      // m_ampPidController.setSmartMotionMaxVelocity(maxVel, kSmartMotionSlot);
      // m_ampPidController.setSmartMotionMaxAccel(maxAccel, kSmartMotionSlot);
      // m_ampPidController.setSmartMotionAllowedClosedLoopError(loopError, kSmartMotionSlot);

      SmartDashboard.putBoolean(getName() + "/Update PIDs", false);
    }

    if (SmartDashboard.getBoolean(getName() + "/Test Go To Targets", false))
    {
      double target = SmartDashboard.getNumber(getName() + "/target position", m_lastTarget);

      // limit operator input
      target = Math.min(kMinMeasuredRetraction, Math.max(kMaxMeasuredExtension, target));
      setTargetPosition(target, kExtendSlot);

      SmartDashboard.putBoolean(getName() + "/Test Go To Targets", false);
    }
  }

  private void setTargetPosition(double position, int slotId)
  {
    m_ampPidController.setReference(position, ControlType.kPosition, slotId);
    // m_ampPidController.setReference(position, ControlType.kSmartMotion);
    m_lastTarget = position;
  }

  public void extendAmpRamp(int slotId)
  {
    setTargetPosition(m_extendDistance, slotId); 
  }

  public void retractAmpRamp()
  {
    setTargetPosition(m_retractDistance, kDefaultSlot);
  }
}
