package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;;

public class Spindexer extends SubsystemBase {

    private final SparkFlex m_Spin;

    //private final SparkClosedLoopController m_SpinController;

    private SparkMaxConfig m_SpinConfig;

    private double setSpinSpeed = 0;

    public Spindexer(){
        m_Spin = new SparkFlex(SpindexerConstants.spinCanID, MotorType.kBrushless);

        //m_SpinController = m_Spin.getClosedLoopController();

        m_SpinConfig = new SparkMaxConfig();

        m_SpinConfig
            .inverted(SpindexerConstants.spinInverted)
            .smartCurrentLimit(60)
            .idleMode(SpindexerConstants.spinIdleMode)
            .closedLoop.pid(SpindexerConstants.spinkP, SpindexerConstants.spinkI, SpindexerConstants.spinkD)
            .feedForward.kV(SpindexerConstants.spinkV);

        m_Spin.configure(m_SpinConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed){
        setSpinSpeed = speed;
        m_Spin.set(setSpinSpeed);
    }

    public void stop(){
        this.setSpeed(0);
    }
    
}
