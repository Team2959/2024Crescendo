package frc.robot.subsystems;

import cwtech.util.AprilTagNav;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    public AprilTagNav m_nav;

    public AprilTagNav getNav() {
        return m_nav;
    }

    // static NetworkTableEntry m_botposeEntry = NetworkTableInstance.getDefault().getTable("limelight-cwtech")
    //         .getEntry("botpose");
    // static NetworkTableEntry m_pipelineEntry = NetworkTableInstance.getDefault().getTable("limelight-cwtech")
    //         .getEntry("pipeline");
    // static NetworkTableEntry m_closestAprilTag = NetworkTableInstance.getDefault().getTable("limelight-cwtech")
    //         .getEntry("tid");

    public Vision() {
        try {
            m_nav = new AprilTagNav();
        } catch(Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    public static final String kLimeLightName = "limelight";

    public static double limelight_aim_proportional()
     {    
        double kP = .035;
        var tx = LimelightHelpers.getTX(kLimeLightName);
        SmartDashboard.putNumber("AprilTag\\tx", tx);
        double targetingAngularVelocity = tx  * kP;
        targetingAngularVelocity *= DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
     }

    public static double limelight_range_proportional(double targetTy)
    {    
        double kP = .1;
        var ty = LimelightHelpers.getTY(kLimeLightName);
        SmartDashboard.putNumber("AprilTag\\ty", ty);
        double deltaTy = targetTy - ty;
        SmartDashboard.putNumber("AprilTag\\ty delta", deltaTy);
        double targetingForwardSpeed = deltaTy * kP;
        targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
}
