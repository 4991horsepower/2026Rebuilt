package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.Spindexer;

public class SpindexerDrive extends Command {

    private Spindexer m_Spindexer;

    public SpindexerDrive(Spindexer spin){
        m_Spindexer = spin;

        addRequirements(spin);
    }

    @Override
    public void execute() {
        m_Spindexer.setSpeed(SpindexerConstants.spinSpeed);
    }
}
