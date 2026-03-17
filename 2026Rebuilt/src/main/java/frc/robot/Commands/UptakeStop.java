package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Uptake;
import frc.robot.Constants.UptakeConstants;

public class UptakeStop extends Command {
    private Uptake m_up;

    public UptakeStop(Uptake up){
        addRequirements(up);
        m_up = up;
    }

    @Override 
    public void execute() {
        m_up.setSpeed(0);
    }

    @Override 
    public boolean isFinished() {
        return true;
    }   
}
