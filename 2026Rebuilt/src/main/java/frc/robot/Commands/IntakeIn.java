package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeIn extends Command {
    private Intake m_intake;

    public IntakeIn(Intake intake){
    m_intake = intake;
    addRequirements(intake);
}

@Override
public void execute(){
    m_intake.setIntakePosition(IntakeConstants.intakeOutPos);
    m_intake.setIntakeVolts(IntakeConstants.intakeRunVolt);
}

@Override 
public boolean isFinished() {
    return m_intake.getDone();
}
}
