package frc.robot.subsystems;

import cwtech.util.AprilTagNav;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

public class Vision {
    public AprilTagNav m_nav;

    public AprilTagNav getNav() {
        return m_nav;
    }

    public static class BotPose {
        double m_x, m_y, m_z, m_roll, m_pitch, m_yaw, m_tid;

        BotPose(double x, double y, double z, double roll, double pitch, double yaw,
            double tid)
        {
            m_x = x;
            m_y = y;
            m_z = z;
            m_roll = roll;
            m_pitch = pitch;
            m_yaw = yaw;
            m_tid = tid;
        }

        public Pose2d toPose2d(Rotation2d currentRotation) {
            return new Pose2d(new Translation2d(m_x, m_y), currentRotation);
        }

        public double getX() {
            return m_x;
        }

        public double getY() {
            return m_y;
        }

        public double getZ() {
            return m_z;
        }

        public double getAprilTag() {
            return m_tid;
        }
    }

    static BotPose[] botposeSamples = new BotPose[2];

    public void sampleBotpose(int index) {
        index = Math.max(index, botposeSamples.length - 1);
        botposeSamples[index] = getBotPose();
    }

    static NetworkTableEntry m_botposeEntry = NetworkTableInstance.getDefault().getTable("limelight-cwtech")
            .getEntry("botpose");
    // TODO correct pipeline id
    static NetworkTableEntry m_pipelineEntry = NetworkTableInstance.getDefault().getTable("limelight-cwtech")
            .getEntry("pipeline");
    static NetworkTableEntry m_closestAprilTag = NetworkTableInstance.getDefault().getTable("limelight-cwtech")
            .getEntry("tid");

    public Vision() {
        try {
            m_nav = new AprilTagNav();
        } catch(Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
        m_botposeEntry.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    }

    public BotPose getBotPose() {
        double[] botposeRaw = m_botposeEntry.getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        double tid = m_closestAprilTag.getDouble(-1);
        var botpose = new BotPose(botposeRaw[0], botposeRaw[1], botposeRaw[2], botposeRaw[3], botposeRaw[4], botposeRaw[5], tid);
        return botpose;
    }

    public double limelight_aim_proportional()
     {    
        double kP = .035;
        double targetingAngularVelocity = 0;//LimelightHelpers.getTX("limelight") * kP;
        targetingAngularVelocity *= DriveSubsystem.kMaxAngularSpeedRadiansPerSecond;
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
     }

    public double limelight_range_proportional()
    {    
        double kP = .1;
        double targetingForwardSpeed = 0; //LimelightHelpers.getTY("limelight") * kP;
        targetingForwardSpeed *= DriveSubsystem.kMaxSpeedMetersPerSecond;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
}
