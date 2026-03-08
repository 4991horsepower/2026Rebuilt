package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeOut extends Command {
    private Intake m_intake;

    public IntakeOut(Intake intake){
    m_intake = intake;
    addRequirements(intake);
}

@Override
public void execute(){
    m_intake.setIntakePosition(IntakeConstants.intakeOutPos);
    m_intake.setIntakeSpeed(IntakeConstants.intakeRunSpeed);
}

@Override 
public boolean isFinished() {
    return m_intake.getDone();
}
}
