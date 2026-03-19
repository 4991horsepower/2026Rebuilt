package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class ResetTurret extends Command {
    private Turret m_Turret;

    public ResetTurret(Turret turret){
    m_Turret = turret;
    addRequirements(turret);
}

@Override
public void execute(){
    m_Turret.resetTurret();
}

@Override 
public boolean isFinished() {
    return true;
}
}
