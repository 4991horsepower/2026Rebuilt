package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

public class IntakeSpinWheels extends Command {
    private Intake m_intake;

    public IntakeSpinWheels(Intake intake){
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        m_intake.setIntakeSpeed(IntakeConstants.intakeRunSpeed);
    }

    @Override 
    public boolean isFinished() {
        return m_intake.getDone();
    }
}
