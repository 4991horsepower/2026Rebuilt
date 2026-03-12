package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Turret m_turret;

    // Buffer stores Robot-to-Field Pose and Turret Rotations
    private final TimeInterpolatableBuffer<Pair<Pose2d, Double>> syncBuffer = 
        TimeInterpolatableBuffer.createBuffer(
            (start, end, pct) -> new Pair<>(
                start.getFirst().interpolate(end.getFirst(), pct), // Interpolate Pose
                MathUtil.interpolate(start.getSecond(), end.getSecond(), pct) // Interpolate Turret
            ),
            1.0 // History depth in seconds
        );

    // Physical Constants (Meters)
    private final Translation2d robotToTurret = new Translation2d(LimelightConstants.kRobotToTurretX, LimelightConstants.kRobotToTurretY);
    private final Translation2d turretToCamera = new Translation2d(LimelightConstants.kTurretToCameraX, LimelightConstants.kTurretToCameraY);

    public Vision(CommandSwerveDrivetrain drivetrain, Turret turret) {
        m_drivetrain = drivetrain;
        m_turret = turret;

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

    public void addBufferSample(double timestamp, Pose2d robotPose, double turretAngle) {
        // Convert degrees back to rotations for the buffer to match the interpolation function
        // or store as degrees if your interpolation function expects degrees. 
        // Here we use rotations to maintain motor-space precision.
        double turretRotations = turretAngle / 360.0;
        
        syncBuffer.addSample(timestamp, new Pair<>(robotPose, turretRotations));
    }



    private void updateVision() {
        // Get initial estimate for timestamp
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kLimelightName);

        // Heuristics Rejection
        double robotAngularRate = m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double turretAngularRate = m_turret.getTurretVelocityDegreesPerSec();
        double combinedAngularRate = Math.abs(robotAngularRate + turretAngularRate);
        if (mt2.tagCount == 0 || mt2.avgTagDist > 6.0 || combinedAngularRate > 720) return;

        syncBuffer.getSample(mt2.timestampSeconds).ifPresent(snap -> {
            Pose2d robotPoseAtT = snap.getFirst();
            double turretRotDeg = snap.getSecond();
            Rotation2d turretAngle = Rotation2d.fromDegrees(turretRotDeg);

            // Deterministic Local Math (Camera-to-Robot Transform)
            Transform2d robotToCamera = new Transform2d(
                robotToTurret.plus(turretToCamera.rotateBy(turretAngle)),
                turretAngle
            );

            // Transform the reported Camera Field Pose into Robot Field Pose
            Pose2d correctedRobotPose = mt2.pose.plus(robotToCamera.inverse());

            // Orientation Seeding (Latest best-guess for solver stability)
            double combinedYaw = robotPoseAtT.getRotation().getDegrees() + turretRotDeg;
            LimelightHelpers.SetRobotOrientation(LimelightConstants.kLimelightName, combinedYaw, combinedAngularRate, 0, 0, 0, 0);

            // Inject into Estimator
            double stdDev = (mt2.tagCount > 1) ? 0.5 : 1.0;
            m_drivetrain.addVisionMeasurement(
                correctedRobotPose, 
                mt2.timestampSeconds, 
                VecBuilder.fill(stdDev, stdDev, 9999999)
            );
        });
    }
}