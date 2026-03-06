package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShooterSpinUp extends Command{
    private Shooter m_Shooter;

    public ShooterSpinUp(Shooter shot){
        m_Shooter = shot;
        addRequirements(shot);
    }

    @Override
    public void execute(){
        m_Shooter.setShooterSpeed(ShooterConstants.shooterShotSpeed);
    }

    @Override
    public boolean isFinished(){
        return m_Shooter.isShooterAtSpeed();
    }
}
