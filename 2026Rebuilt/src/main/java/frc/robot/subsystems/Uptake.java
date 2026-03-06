package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UptakeConstants;
public class Uptake extends SubsystemBase {
    private final TalonFX m_Uptake;
    
    private final TalonFXConfigurator m_UptakeConfigurator;

    private final SlotConfigs m_UptakeConfig;

    private double setVolts = 0;

    public Uptake(){
        m_Uptake = new TalonFX(UptakeConstants.upCANID);

        m_UptakeConfigurator = m_Uptake.getConfigurator();

        m_UptakeConfig = new SlotConfigs();

        m_UptakeConfigurator.apply(m_UptakeConfig);
    }

    public void setVoltage(double volts){
        setVolts = volts;

        m_Uptake.setVoltage(setVolts);
    }

    public void stop(){
        setVolts = 0;
        
        m_Uptake.setVoltage(setVolts);
    }
}
