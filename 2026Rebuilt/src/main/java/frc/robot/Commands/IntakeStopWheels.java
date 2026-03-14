package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeStopWheels extends Command {
    private Intake m_intake;

    public IntakeStopWheels(Intake intake){
    m_intake = intake;
    addRequirements(intake);
}

@Override
public void execute(){
    m_intake.stopWheels();
}

@Override 
public boolean isFinished() {
    return m_intake.getDone();
}
}
