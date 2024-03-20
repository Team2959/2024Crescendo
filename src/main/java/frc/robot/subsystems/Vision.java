package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {

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
