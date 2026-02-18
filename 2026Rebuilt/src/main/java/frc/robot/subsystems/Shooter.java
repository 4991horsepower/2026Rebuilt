package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix6.hardware.TalonFX;
public class Shooter extends SubsystemBase {
    private final TalonFX m_ShooterHood = new TalonFX(0);
    private final TalonFX m_ShooterWheel = new TalonFX(0);

    private double hoodAngle = 0;
    private double shooterSpeed = 0;
}
