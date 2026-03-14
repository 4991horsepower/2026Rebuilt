package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.MathUtil;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Turret m_turret;

private final TimeInterpolatableBuffer<Transform2d> m_transformBuffer = 
    TimeInterpolatableBuffer.createBuffer(
        (start, end, pct) -> new Transform2d(
            // Interpolate X and Y coordinates
            new Translation2d(
                MathUtil.interpolate(start.getX(), end.getX(), pct),
                MathUtil.interpolate(start.getY(), end.getY(), pct)
            ),
            // Interpolate the Rotation
            start.getRotation().interpolate(end.getRotation(), pct)
        ),
        1.0 // History depth in seconds
    );

    // Physical Constants (Meters) - Relative to Robot Center
    private final Translation2d robotToTurretPivot = new Translation2d(LimelightConstants.kRobotToTurretX, LimelightConstants.kRobotToTurretY);
    private final Translation2d turretPivotToCamera = new Translation2d(LimelightConstants.kTurretToCameraX, LimelightConstants.kTurretToCameraY);

    public Vision(CommandSwerveDrivetrain drivetrain, Turret turret) {
        m_drivetrain = drivetrain;
        m_turret = turret;

        // Initialize static camera properties
        LimelightHelpers.setCameraPose_RobotSpace(
            LimelightConstants.kLimelightName, 
            LimelightConstants.kLimelightForward, 
            LimelightConstants.kLimelightSide, 
            LimelightConstants.kLimelightUp, 
            LimelightConstants.kLimelightRoll, 
            LimelightConstants.kLimelightPitch, 
            LimelightConstants.kLimelightYaw);
    }

    @Override
    public void periodic() {
        updateVision();
    }

    /**
     * Called from 250Hz Telemetry Thread.
     * Calculates and stores the spatial relationship between the camera and robot center.
     */
    public void addBufferSample(double timestamp, double turretAngleDegrees) {
        Rotation2d turretAngle = Rotation2d.fromDegrees(turretAngleDegrees);
        
        // Robot Center -> Turret Pivot -> Camera Lens
        Transform2d robotToCamera = new Transform2d(
            robotToTurretPivot.plus(turretPivotToCamera.rotateBy(turretAngle)),
            turretAngle
        );

        // Store the INVERSE (Camera-to-Robot) for easy application to vision results
        m_transformBuffer.addSample(timestamp, robotToCamera.inverse());
    }

    private void updateVision() {
        // Seed Orientation with current heading (Robot Yaw + Turret Yaw)
        double currentRobotYaw = m_drivetrain.getState().Pose.getRotation().getDegrees();
        double currentTurretYaw = m_turret.getTurretAngle();
        double combinedYaw = currentRobotYaw + currentTurretYaw;
        
        LimelightHelpers.SetRobotOrientation(LimelightConstants.kLimelightName, combinedYaw, 0, 0, 0, 0, 0);

        // 2. Get Pose Estimate from Limelight
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kLimelightName);

        // 3. Heuristics Rejection
        double robotAngularRate = m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double turretAngularRate = m_turret.getTurretVelocityDegreesPerSec();
        double combinedAngularRate = Math.abs(robotAngularRate) + Math.abs(turretAngularRate);

        if (mt2.tagCount == 0 || mt2.avgTagDist > 6.0 || combinedAngularRate > 720) return;

        // 4. Time Travel Geometry Correction
        // Use the buffer to find where the camera was relative to the robot when the photo was taken.
        m_transformBuffer.getSample(mt2.timestampSeconds).ifPresent(cameraToRobot -> {
            
            // Transform the reported Camera Field Pose into Robot Field Pose
            Pose2d correctedRobotPose = mt2.pose.plus(cameraToRobot);

            // 5. Inject into Drivetrain Estimator
            // StdDevs: Trust vision less if distance is high or tag count is low
            double stdDev = (mt2.tagCount > 1) ? 0.5 : 1.0;
            
            System.out.println("Vision Measurement: " + correctedRobotPose);

            m_drivetrain.addVisionMeasurement(
                correctedRobotPose, 
                mt2.timestampSeconds, 
                VecBuilder.fill(stdDev, stdDev, 9999999) // Large value for theta; trust the gyro
            );
        });
    }
}