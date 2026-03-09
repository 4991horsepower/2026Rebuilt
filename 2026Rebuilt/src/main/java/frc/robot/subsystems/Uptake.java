package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UptakeConstants;
public class Uptake extends SubsystemBase {
    private final TalonFX m_Uptake;
    
    private final TalonFXConfigurator m_UptakeConfigurator;

    private final SlotConfigs m_UptakeConfig;

    private VelocityTorqueCurrentFOC speedRequest;

    private double setSpeed = 0;

    public Uptake(){
        m_Uptake = new TalonFX(UptakeConstants.upCANID,"Default Name");

        m_UptakeConfigurator = m_Uptake.getConfigurator();

        m_UptakeConfig = new SlotConfigs()
            .withKP(UptakeConstants.upkP)
            .withKI(UptakeConstants.upkI)
            .withKD(UptakeConstants.upkD)
            .withKS(UptakeConstants.upkS)
            .withKV(UptakeConstants.upkV);

        m_UptakeConfigurator.apply(m_UptakeConfig);

        speedRequest = new VelocityTorqueCurrentFOC(setSpeed);
    }

    public void setSpeed(double speed){
        setSpeed = speed;

        m_Uptake.setControl(speedRequest.withVelocity(setSpeed));
    }

    public void stop(){
        setSpeed(0);
    }
}
