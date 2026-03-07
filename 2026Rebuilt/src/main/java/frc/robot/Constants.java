package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

        public static final double hubX = 0; //X Cord of Hub
        public static final double hubY = 0; //Y Cord of Hub
        public static final double hubZ = 72; //Z Offset for aiming at hub

        public static final double playerWallY = 0; //Y Cord of player wall target for better reliablity

        public static final double playerWallLeft = 0; //X cord of left side target
        public static final double playerWallRight = 0; //X cord of right side target


    }

    public static final class ShooterConstants{
        //Hood Motor
        public static final int hoodCANID = 6;

        public static final double hoodkP = 0.01;
        public static final double hoodkI = 0.00;
        public static final double hoodkD = 0.00;

        public static final int kHoodGearRatio = 144;//Motor Rotations to Hood Rotations

        //Shooter Motor
        public static final int shooterCANID = 7;
        
        public static final double shooterkP = 0.01;
        public static final double shooterkI = 0.00;
        public static final double shooterkD = 0.00;

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

        public static final boolean linInverted = false;
        
        public static final IdleMode linIdleMode = IdleMode.kBrake;

        public static final double linkP = 0.01;
        public static final double linkI = 0.00;
        public static final double linkD = 0.00;

        public static final double intakeRunVolt = 0;

        public static final double intakeOutPos = 60; //~75 set lower for tuning
        public static final double intakeInPos = 0;

        public static final double intakeMaxAllErr = .1;
    }

    public static final class SpindexerConstants{
        public static final int spinCanID = 3;

        public static final double spinVoltage = 4;

        public static final boolean spinInverted = false;

        public static final IdleMode spinIdleMode = IdleMode.kCoast;
    }

    public static final class UptakeConstants{
        public static final int upCANID = 4;

        public static final double uptakeVolt = 4;

        public static final boolean UpInverted = false;
    }
}
