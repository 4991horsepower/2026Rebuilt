package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.Spindexer;

public class SpindexerStop extends Command {

    private Spindexer m_Spindexer;

    public SpindexerStop(Spindexer spin){
        m_Spindexer = spin;

        addRequirements(spin);
    }

    @Override
    public void execute() {
        m_Spindexer.setVolts(0);
    }
}
