package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase {
//Assuming use of Kraken for the Intake and a Vortex for the Linear Motion
    private final TalonFX m_InMotor;

    private final TalonFXConfigurator m_InConfigurator;

    private final SlotConfigs m_InConfigs;

    private final SparkFlex m_Linear;

    private final SparkClosedLoopController m_LinearController;

    private final SparkMaxConfig m_LinearConfig;

    private double setVolts = 0;
    private double setPos = 0;
    
    public Intake(){
        m_InMotor = new TalonFX(IntakeConstants.intakeCANID);

        m_InConfigurator = m_InMotor.getConfigurator();

        m_InConfigs = new SlotConfigs();

        m_Linear = new SparkFlex(IntakeConstants.linearCANID, MotorType.kBrushless);

        m_LinearController = m_Linear.getClosedLoopController();

        m_LinearConfig = new SparkMaxConfig();

        m_InConfigs.kP = IntakeConstants.intakekP;
        m_InConfigs.kI = IntakeConstants.intakekI;
        m_InConfigs.kD = IntakeConstants.intakekD;

        m_LinearConfig
        .inverted(IntakeConstants.linInverted)
        .idleMode(IntakeConstants.linIdleMode)
        .closedLoop.pid(
            IntakeConstants.linkP,
            IntakeConstants.linkI,
            IntakeConstants.linkD
        );

        m_InConfigurator.apply(m_InConfigs);
        m_Linear.configure(m_LinearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
