package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class toggleShooter extends Command {

    private Turret m_Shooter;

    public toggleShooter(Turret shooter){
        m_Shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute(){
        m_Shooter.toggleShooter();
    }
    
}
