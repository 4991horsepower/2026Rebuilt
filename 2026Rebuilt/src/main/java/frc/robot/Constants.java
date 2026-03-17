package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DebugConstants
    {
       public static final class Shooter{
        public static final boolean DebugEnable = true;
       } 

       
    }
    public final class AimingConstants{
        public static class Blue{
            public static final double edgeOfShootingArea = Units.inchesToMeters(200.);//Meters From Origin // Splits field accross bump for decidng if aiming for hub or at player wall

            public static final double midFieldSplit = Units.inchesToMeters(158.84);//Meters from origin //Splits Neutral Zone for detirmining if aiming at left or right side of hub

            public static final Pose2d hub = new Pose2d(
                Units.inchesToMeters(182.11),
                midFieldSplit,
                new Rotation2d(0));

            //.5 meter from driver station wall and 3/4 of total field length from conner
            public static final Pose2d leftWall = new Pose2d(
                1.5,
                midFieldSplit + (midFieldSplit/2),
                new Rotation2d(0));

            //.5 meter from driver station wall and 3/4 of total field length from conner
            public static final Pose2d rightWall = new Pose2d(
                0.5,
                midFieldSplit/2,
                new Rotation2d(0));
            
            public static final double launchSpeed = 10;//meters/sec //assuming relativily constant launch speed //Needs Tuned

            public static final double turretHeight = Units.inchesToMeters(21);//Meters off of Floor //Needs Tuned
        }

        public static class Red{
            public static final double edgeOfShootingArea = Units.inchesToMeters(440);//Meters From Origin // Splits field accross bump for decidng if aiming for hub or at player wall

            public static final double midFieldSplit = Units.inchesToMeters(158.84);//Meters from origin //Splits Neutral Zone for detirmining if aiming at left or right side of hub

            public static final Pose2d hub = new Pose2d(
                Units.inchesToMeters(469.11),
                midFieldSplit,
                new Rotation2d(0));

            //.5 meter from driver station wall and 3/4 of total field length from conner
            public static final Pose2d rightWall = new Pose2d(
                15,
                midFieldSplit + (midFieldSplit/2),
                new Rotation2d(0));

            //.5 meter from driver station wall and 3/4 of total field length from conner
            public static final Pose2d leftWall = new Pose2d(
                16.040988,
                midFieldSplit/2,
                new Rotation2d(0));
            
            public static final double launchSpeed = 10;//meters/sec //assuming relativily constant launch speed //Needs Tuned

            public static final double turretHeight = Units.inchesToMeters(21);//Meters off of Floor //Needs Tuned
        }
    }

    public static final class ShooterConstants{
        //Hood Motor
        public static final int hoodCANID = 6;

        public static final double hoodkP = 200;
        public static final double hoodkI = 0.00;
        public static final double hoodkD = 3;

        public static final int kHoodGearRatio = 144;//Motor Rotations to Hood Rotations

        //Shooter Motor
        public static final int shooterCANID = 7;
        
        public static final double shooterkP = 12;
        public static final double shooterkI = 0.0;
        public static final double shooterkD = 0.00;
        public static final double shooterkS = 12;
        public static final double shooterkV = 0.27;

        public static final int shooterShotSpeed = 37;

        //Limits
        public static final double maxHoodAngle = 13;
        public static final double minHoodAngle = 0;
        public static final double hoodZeroAngle = 0;

        public static final double hoodHomingVolts = 2;

        //Error
        public static final double kHoodMaxAllPosErr = 0.1; //Motor Rotations
        public static final double kShooterMaxAllVelErr = 10; //RPM
    }

    public static final class TurretConstants{
        //Turret Motor
        public static final int turretCANID = 5;

        public static final double turretkP = 500;
        public static final double turretkI = 0.00;
        public static final double turretkD = 10;

        public static final double turretGearRatio = 462; //Motor Rotations to Turret Rotations
    }

    public static final class IntakeConstants{
        //Intake Motor
        public static final int intakeCANID = 1;

        public static final int linearCANID = 2;

        public static final boolean intakeInverted = true;

        public static final boolean linInverted = true;
        
        public static final IdleMode linIdleMode = IdleMode.kBrake;

        public static final double linkP = 0.3;
        public static final double linkI = 0.00;
        public static final double linkD = 1;

        public static final double inkP = 6;
        public static final double inkI = 0;
        public static final double inkD = 0;
        public static final double inkV = 1;

        public static final double intakeRunSpeed = 42; //Rotations per second

        public static final double intakeOutPos = 56.66; //Rotations
        public static final double intakeInPos = 0; //Rotations
        public static final double intakeInterPos = 30;//Rotations

        public static final double intakeMaxAllErr = .1; // Rotations

        public static final double kHomingVolts = -4;
    }

    public static final class SpindexerConstants{
        public static final int spinCanID = 3;

        public static final double spinkP = 0.00007;
        public static final double spinkI = 0.0;
        public static final double spinkD = 0.01;
        public static final double spinkV = 0.002;

        public static final boolean spinInverted = false;

        public static final IdleMode spinIdleMode = IdleMode.kCoast;

        public static final double spinSpeed = 101;
    }

    public static final class UptakeConstants{
        public static final int upCANID = 4;

        public static final double upkP = 1;
        public static final double upkI = 0;
        public static final double upkD = 0;
        public static final double upkS = 28;
        public static final double upkV = 0.17;

        public static final boolean UpInverted = false;

        public static final double uptakeSpeed = 111;
    }

    public static final class LimelightConstants{
        public static final String kLimelightName = "limelight";
        // THESE ARE SPECIFICALLY DIMENSIONS UNAFFECTED BY TURRET MOVEMENT
        // MOTION COUPLED TO TURRET MOVEMENT SHOULD BE ACCOUNTED FOR BY POSE OFFSETS
        public static final double kLimelightForward = 0.0; // Forward offset (meters)
        public static final double kLimelightSide = 0.0; // Side offset (meters)
        public static final double kLimelightUp = 0.4253992; // Height offset (meters)
        public static final double kLimelightRoll = 0; // Roll (degrees)
        public static final double kLimelightPitch = 15.0; // Pitch (degrees)
        public static final double kLimelightYaw = 0; // Yaw (degrees)

        // Robot to Turret transform
        public static final double kRobotToTurretX = -0.10971276; // Meters forward of robot center
        public static final double kRobotToTurretY = 0.1097788; // Meters to the left of robot center
        public static final double kRobotToTurretTheta = 0.0; // Degrees, positive is CCW
       
        // Turret to Camera transform
        public static final double kTurretToCameraX = 0.2032; // Meters forward of turret
        public static final double kTurretToCameraY = 0.0; // Meters to the left of turret
        public static final double kTurretToCameraTheta = 0.0; // Degrees, positive is CCW
    }
}
