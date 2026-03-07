package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Uptake;
import frc.robot.Constants.UptakeConstants;

public class UptakeUp extends Command {
    private Uptake m_up;

    public UptakeUp(Uptake up){
        addRequirements(up);
        m_up = up;
    }

    @Override 
    public void execute() {
        m_up.setSpeed(UptakeConstants.uptakeSpeed);
    }
}
