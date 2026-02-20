package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class Constants {
    public static final class DebugConstants
    {
       public static final class Shooter{
        public static final boolean DebugEnable = true;
       } 

       
    }

    public static final class ShooterConstants{
        //Hood Motor
        public static final int hoodCANID = 0;

        public static final double hoodkP = 0.01;
        public static final double hoodkI = 0.00;
        public static final double hoodkD = 0.00;

        public static final int kHoodGearRatio = 144;//Motor Rotations to Hood Rotations

        //Shooter Motor
        public static final int shooterCANID = 0;
        
        public static final double shooterkP = 0.01;
        public static final double shooterkI = 0.00;
        public static final double shooterkD = 0.00;

        public static final int shooterShotSpeed = 0;

        //Limits
        public static final double maxHoodAngle = 60; //Degrees From Horizontal
        public static final double minHoodAngle = 15; //Degrees From Horizontal

        //Error
        public static final double kHoodMaxAllPosErr = 0.1; //Motor Rotations
        public static final double kShooterMaxAllVelErr = 10; //RPM
    }

    public static final class TurretConstants{
        //Turret Motor
        public static final int turretCANID = 0;

        public static final double turretkP = 0.00;
        public static final double turretkI = 0.00;
        public static final double turretkD = 0.00;
    }

    public static final class IntakeConstants{
        //Intake Motor
        public static final int intakeCANID = 0;
        
        public static final double intakekP = 0.00;
        public static final double intakekI = 0.00;
        public static final double intakekD = 0.00;

        public static final int intakeRunSpeed = 0;
    }


}
