package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class stopShooter extends Command {

    private Shooter m_Shooter;

    public stopShooter(Shooter shooter){
        m_Shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute(){
        m_Shooter.setShooterSpeed(0);
    }
    
}
