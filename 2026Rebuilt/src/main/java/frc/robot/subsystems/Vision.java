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
    private final Translation2d ROBOT_TO_TURRET_PIVOT = new Translation2d(0.1, 0.0);
    private final Translation2d TURRET_PIVOT_TO_CAMERA = new Translation2d(0.05, 0.0);

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
        double robotVelocity = m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        double turretVelocity = m_turret.getTurretVelocityDegreesPerSec();
        double combinedVelocity = Math.abs(robotVelocity + turretVelocity);
        if (mt2.tagCount == 0 || mt2.avgTagDist > 6.0 || combinedVelocity > 720) return;

        syncBuffer.getSample(mt2.timestampSeconds).ifPresent(snap -> {
            Pose2d robotPoseAtT = snap.getFirst();
            double turretRot = snap.getSecond();
            Rotation2d turretAngle = Rotation2d.fromRotations(turretRot);

            // Deterministic Local Math (Camera-to-Robot Transform)
            Transform2d robotToCamera = new Transform2d(
                ROBOT_TO_TURRET_PIVOT.plus(TURRET_PIVOT_TO_CAMERA.rotateBy(turretAngle)),
                turretAngle
            );

            // Transform the reported Camera Field Pose into Robot Field Pose
            Pose2d correctedRobotPose = mt2.pose.plus(robotToCamera.inverse());

            // 3. Orientation Seeding (Latest best-guess for solver stability)
            double combinedYaw = robotPoseAtT.getRotation().getDegrees() + (turretRot * 360.0);
            LimelightHelpers.SetRobotOrientation(LimelightConstants.kLimelightName, combinedYaw, 0, 0, 0, 0, 0);

            // 4. Inject into Estimator
            double stdDev = (mt2.tagCount > 1) ? 0.5 : 1.0;
            m_drivetrain.addVisionMeasurement(
                correctedRobotPose, 
                mt2.timestampSeconds, 
                VecBuilder.fill(stdDev, stdDev, 9999999)
            );
        });
    }
}