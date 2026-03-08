package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UptakeConstants;
public class Uptake extends SubsystemBase {
    private final TalonFX m_Uptake;
    
    private final TalonFXConfigurator m_UptakeConfigurator;

    private final Slot0Configs m_UptakeConfig;

    private VelocityVoltage speedRequest;

    private double setSpeed = 0;

    public Uptake(){
        m_Uptake = new TalonFX(UptakeConstants.upCANID);

        m_UptakeConfigurator = m_Uptake.getConfigurator();

        m_UptakeConfig = new Slot0Configs()
            .withKP(UptakeConstants.upkP)
            .withKI(UptakeConstants.upkI)
            .withKD(UptakeConstants.upkD)
            .withKS(UptakeConstants.upkS)
            .withKV(UptakeConstants.upkV);

        m_UptakeConfigurator.apply(m_UptakeConfig);

        speedRequest = new VelocityVoltage(setSpeed);
    }

    public void setSpeed(double speed){
        setSpeed = speed;

        m_Uptake.setControl(speedRequest.withVelocity(setSpeed));
    }

    public void stop(){
        setSpeed(0);
    }
}
