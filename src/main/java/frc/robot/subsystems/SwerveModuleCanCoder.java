package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class SwerveModuleCanCoder {
    private static final double kDriveP = 0.05;
    private static final double kDriveI = 0.0;
    private static final double kDriveD = 0.001;
    private static final double kDriveFF = 0.02;
    private static final double kDriveIZone = 600;
    private static final double kSteerP = 0.4;
    private static final double kSteerI = 0.00001;
    private static final double kSteerD = 0.0;
    // private static final double kSteerFF = 0.0;
    private static final double kSteerIZone = 1.0;

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_steerMotor;
    private CANcoder m_steerAbsoluteEncoder;
    private SparkRelativeEncoder m_driveEncoder;
    private SparkRelativeEncoder m_steerEncoder;
    private SparkPIDController m_drivePIDController;
    private SparkPIDController m_steerPIDController;
    private final double m_steerOffset;
    private final String m_name;

    private static final double kWheelRadius = 2.0 * 0.0254; // 2" * 0.0254 m / inch
    private static final double kGearboxRatio = 1.0 / 6.12; // One turn of the wheel is 6.86 turns of the motor
    private static final double kDrivePositionFactor = (2.0 * Math.PI * kWheelRadius * kGearboxRatio);
    private static final int kDriveCurrentLimitAmps = 70;
    private static final int kSteerCurrentLimitAmps = 40; 
    private static final double kSteerMotorRotationsPerRevolution = 12.8;

    public SwerveModuleCanCoder(int motorAssembly, double steerOffset, String name)
    {
        this(motorAssembly, motorAssembly + 10, motorAssembly, steerOffset, name);
    }

    // https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/SwerveModule.java
    private SwerveModuleCanCoder(int driveMotor, int steerMotor, int steerAbsoluteEncoder, double steerOffset, String name)
    {
        m_driveMotor = new CANSparkMax(driveMotor, CANSparkMax.MotorType.kBrushless);
        m_steerMotor = new CANSparkMax(steerMotor, CANSparkMax.MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_steerMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_steerMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit(kDriveCurrentLimitAmps);
        m_steerMotor.setSmartCurrentLimit(kSteerCurrentLimitAmps);

        m_name = name;

        m_steerOffset = steerOffset;

        m_steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoder);

        m_driveEncoder = (SparkRelativeEncoder) m_driveMotor.getEncoder();
        m_steerEncoder = (SparkRelativeEncoder) m_steerMotor.getEncoder();
    
        m_drivePIDController = m_driveMotor.getPIDController();
        m_steerPIDController = m_steerMotor.getPIDController();

        m_driveEncoder.setPositionConversionFactor(kDrivePositionFactor);
        m_driveEncoder.setVelocityConversionFactor(kDrivePositionFactor / 60.0);
        
        m_drivePIDController.setP(kDriveP);
        m_drivePIDController.setI(kDriveI);
        m_drivePIDController.setD(kDriveD);
        m_drivePIDController.setFF(kDriveFF);
        m_drivePIDController.setIZone(kDriveIZone);

        m_steerPIDController.setFeedbackDevice(m_steerEncoder);
        m_steerPIDController.setP(kSteerP);
        m_steerPIDController.setI(kSteerI);
        m_steerPIDController.setD(kSteerD);
        m_steerPIDController.setIZone(kSteerIZone);

        m_steerEncoder.setPositionConversionFactor(2 * Math.PI / kSteerMotorRotationsPerRevolution);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_steerPIDController.setPositionPIDWrappingEnabled(true);
        m_steerPIDController.setPositionPIDWrappingMinInput(0);
        m_steerPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);
    }

    public void smartDashboardInit() {
        SmartDashboard.putNumber(m_name + "/Steer Motor Position", getRelativeEncoderPosition().getDegrees());
        SmartDashboard.putNumber(m_name + "/Steer Absolute Position", getAbsoluteEncoderPosition().getDegrees());
        // SmartDashboard.putNumber(m_name + "/Drive P", m_drivePIDController.getP());
        // SmartDashboard.putNumber(m_name + "/Drive I", m_drivePIDController.getI());
        // SmartDashboard.putNumber(m_name + "/Drive D", m_drivePIDController.getD());
        // SmartDashboard.putNumber(m_name + "/Drive IZone", m_drivePIDController.getIZone());
        // SmartDashboard.putNumber(m_name + "/Drive FF", m_drivePIDController.getFF());
        // SmartDashboard.putNumber(m_name + "/Steer P", m_steerPIDController.getP());
        // SmartDashboard.putNumber(m_name + "/Steer I", m_steerPIDController.getI());
        // SmartDashboard.putNumber(m_name + "/Steer D", m_steerPIDController.getD());
        // SmartDashboard.putNumber(m_name + "/Steer IZone", m_steerPIDController.getIZone());
        // SmartDashboard.putNumber(m_name + "/Steer FF", m_steerPIDController.getFF());
    }

    public void smartDashboardUpdate() {
        // SmartDashboard.putNumber(m_name + "/Drive Encoder Velocity", m_driveEncoder.getVelocity());
        // SmartDashboard.putNumber(m_name + "/Drive Encoder Position", m_driveEncoder.getPosition()); 
        SmartDashboard.putNumber(m_name + "/Steer Motor Position", getRelativeEncoderPosition().getDegrees());
        SmartDashboard.putNumber(m_name + "/Steer Absolute Position", getAbsoluteEncoderPosition().getDegrees());

    //    m_drivePIDController.setP (SmartDashboard.getNumber(m_name + "/Drive P", kDriveP));
    //    m_drivePIDController.setI (SmartDashboard.getNumber(m_name + "/Drive I", kDriveI));
    //    m_drivePIDController.setD (SmartDashboard.getNumber(m_name + "/Drive D", kDriveD));
    //    m_drivePIDController.setIZone (SmartDashboard.getNumber(m_name + "/Drive IZone", kDriveIZone));
    //    m_drivePIDController.setFF (SmartDashboard.getNumber(m_name + "/Drive FF", kDriveFF));

    //    m_steerPIDController.setP (SmartDashboard.getNumber(m_name + "/Steer P", kSteerP));
    //    m_steerPIDController.setI (SmartDashboard.getNumber(m_name + "/Steer I", kSteerI));
    //    m_steerPIDController.setD (SmartDashboard.getNumber(m_name + "/Steer D", kSteerD));
    //    m_steerPIDController.setIZone (SmartDashboard.getNumber(m_name + "/Steer IZone", kSteerIZone));
    //    m_steerPIDController.setFF (SmartDashboard.getNumber(m_name + "/Steer FF", kSteerFF));
    }

    private Rotation2d getCanCoder()
    {
        double rotations = m_steerAbsoluteEncoder.getAbsolutePosition().getValue();
        double degrees = rotations * 360;
        if (degrees < 0)
          degrees += 360;    

        return Rotation2d.fromDegrees(degrees);
    }

    private Rotation2d getAbsoluteEncoderPosition()
    {
        double startingAngle = m_steerOffset - getCanCoder().getRadians();
    
        if (startingAngle < 0)
        {
          startingAngle = startingAngle + (2 * Math.PI);
        }

        // need to convert from absolute CAN coder turning clockwise as positive to
        //  relative encoder in assembly turning counter-clockwise as positive
        startingAngle = 2 * Math.PI - startingAngle;

        return Rotation2d.fromRadians(startingAngle);
    }

    private Rotation2d getRelativeEncoderPosition()
    {
        return Rotation2d.fromRadians(m_steerEncoder.getPosition());
    }

    public double getVelocity()
    {
        return m_driveEncoder.getVelocity();
    }

    public SwerveModuleState getSwerveModuleState()
    {
        // return new SwerveModuleState(getVelocity(), getAbsoluteEncoderPosition());
        return new SwerveModuleState(getVelocity(), getRelativeEncoderPosition());
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), getRelativeEncoderPosition());
    }

    /*
     * Sets this modules 
     */
    public void setDesiredState(SwerveModuleState referenceState)
    {
        SwerveModuleState state = SwerveModuleState.optimize(referenceState, getRelativeEncoderPosition());

        setDriveVelocity(state.speedMetersPerSecond);

        setSteerAngleInRadians(state.angle.getRadians());
    }

    private void setDriveVelocity(double targetSpeed)
    {
        m_drivePIDController.setReference(targetSpeed * DriveSubsystem.kMaxSpeedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    }

    private void setSteerAngleInRadians(double targetAngleInRadians)
    {
        SmartDashboard.putNumber(m_name + "/Setpoint", targetAngleInRadians);
        m_steerPIDController.setReference(targetAngleInRadians, CANSparkMax.ControlType.kPosition);
    }

    /*
     * Resets the SparkMax Alternative Encoder to match the absolute Mag encoder,
     * setting the position of the Mag Encoder to the SparkMax Alternative Encoder 
     */
    public void resetAngleEncoderToAbsolute()
    {
        m_steerEncoder.setPosition(getAbsoluteEncoderPosition().getRadians());
    }

    public void lockWheelAtAngleInDegrees(double degrees)
    {
        setDriveVelocity(0);
        var angleInRadians = degrees * Math.PI / 180.0;
        setSteerAngleInRadians(angleInRadians);
    }

    public void checkRelativeEncoderToAbsoluteEncoder()
    {
        //we put that the difference should be less than 0.025 which is approximately 0.5% margin of error
        // 0.025 radians is ~ 1.5 degrees
        if (Math.abs(getAbsoluteEncoderPosition().getRadians() - getRelativeEncoderPosition().getRadians()) > 0.025)
        {
            resetAngleEncoderToAbsolute();
        }
    }
}
