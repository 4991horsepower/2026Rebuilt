package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Unit;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DebugConstants
    {
       public static final class Shooter{
        public static final boolean DebugEnable = true;
       } 

       
    }
    public static final class AimingConstants{
        public static final double edgeOfShootingArea = 0;//Feet From Origin // Splits field accross bump for decidng if aiming for hub or at player wall

        public static final double MidFieldSplit = 0;//Feet from origin //Splits Neutral Zone for detirmining if aiming at left or right side of hub

        public static final Translation3d hub = new Translation3d(
            0,
            0,
            Units.inchesToMeters(72));

        public static final Translation3d leftWall = new Translation3d(
            0,
            .5,
            0);

        public static final Translation3d RightWall = new Translation3d(
            0,
            .5,
            0);
        
        public static final double launchSpeed = 10;//meters/sec //assuming relativily constant launch speed //Needs Tuned

        public static final double turretHeight = Units.inchesToMeters(21);//Meters off of Floor //Needs Tuned


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

        public static final int shooterShotSpeed = 0;

        //Limits
        public static final double maxHoodAngle = 60; //Degrees From Horizontal
        public static final double minHoodAngle = 15; //Degrees From Horizontal
        public static final double hoodZeroAngle = 60; //Degrees from Horizontal

        public static final double hoodHomingVolts = 2;

        //Error
        public static final double kHoodMaxAllPosErr = 0.1; //Motor Rotations
        public static final double kShooterMaxAllVelErr = 10; //RPM
    }

    public static final class TurretConstants{
        //Turret Motor
        public static final int turretCANID = 5;

        public static final double turretkP = 0.01;
        public static final double turretkI = 0.00;
        public static final double turretkD = 0.00;

        public static final double turretGearRatio = 462; //Motor Rotations to Turret Rotations
    }

    public static final class IntakeConstants{
        //Intake Motor
        public static final int intakeCANID = 1;

        public static final int linearCANID = 2;

        public static final boolean intakeInverted = false;

        public static final boolean linInverted = true;
        
        public static final IdleMode linIdleMode = IdleMode.kBrake;

        public static final double linkP = 0.3;
        public static final double linkI = 0.00;
        public static final double linkD = 1;

        public static final double inkP = 6;
        public static final double inkI = 0;
        public static final double inkD = 0;
        public static final double inkV = 1;

        public static final double intakeRunSpeed = 40; //Rotations per minute

        public static final double intakeOutPos = 56.5; //Rotations
        public static final double intakeInPos = 0; //Rotations

        public static final double intakeMaxAllErr = .1; // Rotations
    }

    public static final class SpindexerConstants{
        public static final int spinCanID = 3;

        public static final double spinkP = 0.00007;
        public static final double spinkI = 0.0;
        public static final double spinkD = 0.01;
        public static final double spinkV = 0.002;

        public static final boolean spinInverted = false;

        public static final IdleMode spinIdleMode = IdleMode.kCoast;

        public static final double spinSpeed = 2000;
    }

    public static final class UptakeConstants{
        public static final int upCANID = 4;

        public static final double upkP = 1;
        public static final double upkI = 0;
        public static final double upkD = 0;
        public static final double upkS = 28;
        public static final double upkV = 0.17;

        public static final boolean UpInverted = false;

        public static final double uptakeSpeed = 2000;
    }
}
