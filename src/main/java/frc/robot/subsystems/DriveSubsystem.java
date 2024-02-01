// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI.Port;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
    private SwerveModuleCanCoder m_frontLeft;
    private SwerveModuleCanCoder m_frontRight;
    private SwerveModuleCanCoder m_backLeft;
    private SwerveModuleCanCoder m_backRight;
    private AHRS m_navX;

    private boolean m_initalized = false;

    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;// kMaxSpeedMetersPerSecond /
                                                                          // Math.hypot(0.381, 0.381);
    // TODO: ensure this is the correct value for 2024 robot
    private static final double kHalfTrackWidthMeters = 0.571 / 2.0;
    private final Translation2d kFrontLeftLocation = new Translation2d(kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kFrontRightLocation = new Translation2d(kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
    private final Translation2d kBackLeftLocation = new Translation2d(-kHalfTrackWidthMeters, kHalfTrackWidthMeters);
    private final Translation2d kBackRightLocation = new Translation2d(-kHalfTrackWidthMeters, -kHalfTrackWidthMeters);
    private static final double kRotationP = 0.1;
    private static final double kRotationI = 0.0;
    private static final double kRotationD = 0.0;

    final PIDController m_rotationController = new PIDController(kRotationP, kRotationI, kRotationD);
    double m_angleToRotateTo = 0.0;
    boolean m_rotationAlignmentOn = false;
    
    double autoRotateValue = 0.0;
    double autoXValue = 0.0;
    double autoYValue = 0.0;

    private int m_ticks = 0;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        m_navX = new AHRS(Port.kMXP);

        m_kinematics = new SwerveDriveKinematics(kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation,
                kBackRightLocation);

        m_frontLeft = new SwerveModuleCanCoder(RobotMap.kFrontLeftModule,
                RobotMap.kZeroedFrontLeft, "Front Left");
        m_frontRight = new SwerveModuleCanCoder(RobotMap.kFrontRightModule,
                RobotMap.kZeroedFrontRight, "Front Right");
        m_backLeft = new SwerveModuleCanCoder(RobotMap.kBackLeftModule,
                RobotMap.kZeroedBackLeft, "Back Left");
        m_backRight = new SwerveModuleCanCoder(RobotMap.kBackRigvhtModule,
                RobotMap.kZeroedBackRight, "Back Right");

        m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle(), getPositions());
    }

    public void initalize() {
        if (m_initalized)
            return;
        m_navX.reset();
        m_frontLeft.resetAngleEncoderToAbsolute();
        m_frontRight.resetAngleEncoderToAbsolute();
        m_backLeft.resetAngleEncoderToAbsolute();
        m_backRight.resetAngleEncoderToAbsolute();
        m_initalized = true;
    }

    public void offsetNavX(Rotation2d offset) {
        m_navX.setAngleAdjustment(offset.getDegrees());
    }

    public void turnOnRotate() {
        double diff180 = Math.abs(180 - getAngle().getDegrees());
        double diff0 = Math.abs(getAngle().getDegrees());
        if(diff0 > diff180) {
            m_angleToRotateTo = 180;
        } else {
            m_angleToRotateTo = 0;
        }
        m_rotationAlignmentOn = true;
    }

    public void turnOffRotate() {
        m_rotationAlignmentOn = false;
    }

    @Override
    public void periodic() {
        m_odometry.update(getAngle(), getPositions());
        
        // if(m_rotationAlignmentOn) {
        //     // TODO trapezoidal
        //     autoRotateValue = m_rotationController.calculate(m_navX.getAngle());
        // }

        m_ticks++;
        if (m_ticks % 15 != 7)
            return;

        SmartDashboard.putNumber(getName() + "/Angle", getAngle().getDegrees());
        // SmartDashboard.putNumber(getName() + "/Roll", m_navX.getRoll());
        // SmartDashboard.putNumber(getName() + "/Pitch", m_navX.getPitch());
        
        // BotPose botpose = Vision.getBotPose();
        // SmartDashboard.putNumber(getName() + "/Distance X", botpose.getX());
        // SmartDashboard.putNumber(getName() + "/Distance Y", botpose.getY());
        // SmartDashboard.putNumber(getName() + "/Distance Z", botpose.getZ());
    }

    public void smartDashboardInit() {
        // m_frontLeft.smartDashboardInit();
        // m_frontRight.smartDashboardInit();
        // m_backLeft.smartDashboardInit();
        // m_backRight.smartDashboardInit();

        // SmartDashboard.putNumber(getName() + "/Smart Motion Vel", kSmartMotionVel);
        // SmartDashboard.putNumber(getName() + "/Smart Motion Accel", kSmartMotionAccel);
    }

    public void smartDashboardUpdate() {
        // var maxVel = SmartDashboard.getNumber(getName() + "/Smart Motion Vel", kSmartMotionVel);
        // var maxAccel = SmartDashboard.getNumber(getName() + "/Smart Motion Accel", kSmartMotionAccel);

        // m_frontLeft.setSmartMotion(maxVel, maxAccel);
        // m_frontRight.setSmartMotion(maxVel, maxAccel);
        // m_backLeft.setSmartMotion(maxVel, maxAccel);
        // m_backRight.setSmartMotion(maxVel, maxAccel);

        // m_frontLeft.smartDashboardUpdate();
        // m_frontRight.smartDashboardUpdate();
        // m_backLeft.smartDashboardUpdate();
        // m_backRight.smartDashboardUpdate();
    }

    public void drive(double xMetersPerSecond, double yMetersPerSecond,
            double rotationRadiansPerSecond, boolean fieldRelative) {
        // if(m_rotationAlignmentOn) {
            // var autoRotateValue = m_rotationController.calculate(getAngle().getDegrees());
            // rotationRadiansPerSecond = Math.toRadians(autoRotateValue * 360);
        // }

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond,
                        getAngle())
                : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, rotationRadiansPerSecond));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setDesiredState(SwerveModuleState[] states) {
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    public Rotation2d getAngle() {
        return m_navX.getRotation2d();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getAngle(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] swerveStates = { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_backLeft.getPosition(), m_backRight.getPosition() };
        return swerveStates;
    }

    public void stopAndLockWheels() {
        m_frontLeft.lockWheelAtAngleInDegrees(45);
        m_frontRight.lockWheelAtAngleInDegrees(-45);
        m_backLeft.lockWheelAtAngleInDegrees(-45);
        m_backRight.lockWheelAtAngleInDegrees(45);
    }

    public void resetNavX() {
        m_navX.reset();
    }
}
