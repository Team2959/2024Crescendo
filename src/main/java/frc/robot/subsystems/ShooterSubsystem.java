// Copyright (c) FIRST and o  private SparkPIDController m_shooterPidController;
//ther WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSubsystem extends SubsystemBase
{
  public enum ShooterLocation
  {
    Generic,
    Trap,
    CenterSpeaker,
    LeftSpeaker,
    RightSpeaker,
    Amp,
    SourceLoad,
  };

  private CANSparkMax m_leftShooterWheel;
  private CANSparkMax m_rightShooterWheel;
  private SparkRelativeEncoder m_leftShooterEncoder;
  private SparkRelativeEncoder m_rightShooterEncoder;
  private SparkPIDController m_leftShooterPidController;
  private SparkPIDController m_rightShooterPidController;

  private static final double kShooterP = 0.01;
  private static final double kShooterI = 0.0;
  private static final double kShooterD = 0.0;
  private static final double kShooterFF = 0;
  private static final double kShooterIZone = 0;
  private double m_leftTargetVelocity = 1.0;//5000.0; We have been using -0.75 at distance of 20 in from trap
  private double m_rightTargetVelocity = 1.0;//5000.0; We have been using -0.75 at distance of 20 in from trap
  private double m_trapVelocity = 0.75; //guess
  private double m_centerSpeakerVelocity = 0.75; //guess
  private double m_sideSlowMotorSpeakerVelocity = 0.50; //guess
  private double m_sideFastMotorSpeakerVelocity = 0.75; //guess
  private double m_ampVelocity = 0.75; //guess
 private double m_sourceVelocity = 0.75; //guess


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() 
  {
    m_leftShooterWheel = new CANSparkMax(RobotMap.kLeftShooterCANSparkMaxWheel, CANSparkMax.MotorType.kBrushless);
    m_rightShooterWheel = new CANSparkMax(RobotMap.kRightShooterCANSparkMaxWheel, CANSparkMax.MotorType.kBrushless);

    m_leftShooterWheel.restoreFactoryDefaults();

    m_leftShooterWheel.setIdleMode(IdleMode.kCoast);
    m_rightShooterWheel.setIdleMode(IdleMode.kCoast);

    m_leftShooterWheel.setInverted(true);

    m_leftShooterPidController = m_leftShooterWheel.getPIDController();
    m_leftShooterPidController.setP(kShooterP);
    m_leftShooterPidController.setI(kShooterI);
    m_leftShooterPidController.setD(kShooterD);
    m_leftShooterPidController.setFF(kShooterFF);
    m_leftShooterPidController.setIZone(kShooterIZone);

    m_rightShooterPidController = m_rightShooterWheel.getPIDController();
    m_rightShooterPidController.setP(kShooterP);
    m_rightShooterPidController.setI(kShooterI);
    m_rightShooterPidController.setD(kShooterD);
    m_rightShooterPidController.setFF(kShooterFF);
    m_rightShooterPidController.setIZone(kShooterIZone);

    m_leftShooterEncoder = (SparkRelativeEncoder)m_leftShooterWheel.getEncoder();
    m_rightShooterEncoder = (SparkRelativeEncoder)m_rightShooterWheel.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void controlShooterToVelocity(ShooterLocation location)
  {
    double leftVelocity = 0;
    double rightVelocity = 0;

    switch (location) {
      case Generic:
        leftVelocity = m_leftTargetVelocity;
        rightVelocity = m_rightTargetVelocity;
        break;
      case Trap:
        leftVelocity = m_trapVelocity;
        rightVelocity = m_trapVelocity;
        break;
     case CenterSpeaker:
        leftVelocity = m_centerSpeakerVelocity;
        rightVelocity = m_centerSpeakerVelocity;
        break;
     case LeftSpeaker:
        leftVelocity = m_sideFastMotorSpeakerVelocity;
        rightVelocity = m_sideSlowMotorSpeakerVelocity;
        break;
     case RightSpeaker:
        leftVelocity = m_sideSlowMotorSpeakerVelocity;
        rightVelocity = m_sideFastMotorSpeakerVelocity;
        break;
    case Amp:
        leftVelocity = m_ampVelocity;
        rightVelocity = m_ampVelocity;
        break;
    case SourceLoad:
        leftVelocity = m_sourceVelocity;
        rightVelocity = m_sourceVelocity;
        break;
      default:
        break;
    }

    controlShooterToVelocity(leftVelocity, rightVelocity);
  }

  public void controlShooterToVelocity(double leftVelocity, double rightVelocity)
  {
    // m_leftShooterPidController.setReference(leftVelocity, ControlType.kVelocity);
    // m_rightShooterPidController.setReference(rightVelocity, ControlType.kVelocity);
    m_leftShooterWheel.set(leftVelocity);
    m_rightShooterWheel.set(rightVelocity);
  }

  public double getLeftTargetVelocity()
  {
    return m_leftTargetVelocity;
  }

  public double geRightTargetVelocity()
  {
    return m_rightTargetVelocity;
  }

  public void smartDashboardInit()
  {
    SmartDashboard.putNumber(getName() + "/left current velocity", m_leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber(getName() + "/right current velocity", m_rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber(getName() + "/left target velocity", getLeftTargetVelocity());
    SmartDashboard.putNumber(getName() + "/right target velocity", geRightTargetVelocity());

    SmartDashboard.putBoolean(getName() + "/Update PIDs", false);
    SmartDashboard.putNumber(getName() + "/shooter P", kShooterP);
    SmartDashboard.putNumber(getName() + "/shooter I", kShooterI);
    SmartDashboard.putNumber(getName() + "/shooter D", kShooterD);
    SmartDashboard.putNumber(getName() + "/shooter FF", kShooterFF);

    SmartDashboard.getNumber(getName() + "/trap target velocity", m_trapVelocity);
    SmartDashboard.getNumber(getName() + "/speaker center velocity", m_centerSpeakerVelocity);
    SmartDashboard.getNumber(getName() + "/side speaker slow velocity", m_sideSlowMotorSpeakerVelocity);
    SmartDashboard.getNumber(getName() + "/speaker left velocity", m_sideFastMotorSpeakerVelocity);
    SmartDashboard.getNumber(getName() + "/amp velocity", m_ampVelocity);
    SmartDashboard.getNumber(getName() + "/source velocity", m_sourceVelocity);
  }

  public void smartDashboardUpdate()
  {
    SmartDashboard.putNumber(getName() + "/left current velocity", m_leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber(getName() + "/right current velocity", m_rightShooterEncoder.getVelocity());
    m_leftTargetVelocity = SmartDashboard.getNumber(getName() + "/left target velocity", getLeftTargetVelocity());
    m_rightTargetVelocity = SmartDashboard.getNumber(getName() + "/right target velocity", geRightTargetVelocity());
    m_trapVelocity = SmartDashboard.getNumber(getName() + "/trap target velocity", m_trapVelocity);
    m_centerSpeakerVelocity = SmartDashboard.getNumber(getName() + "/center speaker target velocity", m_centerSpeakerVelocity);
    m_sideFastMotorSpeakerVelocity = SmartDashboard.getNumber(getName() + "/left speaker target velocity", m_sideFastMotorSpeakerVelocity);
    m_sideSlowMotorSpeakerVelocity = SmartDashboard.getNumber(getName() + "/side speaker slow target velocity", m_sideSlowMotorSpeakerVelocity);
    m_ampVelocity = SmartDashboard.getNumber(getName() + "/amp target velocity", m_ampVelocity);
    m_sourceVelocity = SmartDashboard.getNumber(getName() + "/source target velocity", m_sourceVelocity);


    if (SmartDashboard.getBoolean(getName() + "/Update PIDs", false))
    {
      double pGain = SmartDashboard.getNumber(getName() + "/shooter P", kShooterP);
      double iGain = SmartDashboard.getNumber(getName() + "/shooter I", kShooterI);
      double dGain = SmartDashboard.getNumber(getName() + "/shooter D", kShooterD);
      double ffGain = SmartDashboard.getNumber(getName() + "/shooter FF", kShooterFF);

      m_leftShooterPidController.setP(pGain);
      m_leftShooterPidController.setI(iGain);
      m_leftShooterPidController.setD(dGain);
      m_leftShooterPidController.setFF(ffGain);

      m_rightShooterPidController.setP(pGain);
      m_rightShooterPidController.setI(iGain);
      m_rightShooterPidController.setD(dGain);
      m_rightShooterPidController.setFF(ffGain);

      SmartDashboard.putBoolean(getName() + "/Update PIDs", false);
    }
  }

  public void stopShooterMotor()
  {
    m_leftShooterWheel.set(0);
    m_rightShooterWheel.set(0);
  }
}
