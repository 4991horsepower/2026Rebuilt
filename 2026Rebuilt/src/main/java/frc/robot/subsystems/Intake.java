package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase {
//Assuming use of Kraken for the Intake and a Vortex for the Linear Motion
    private final TalonFX m_InMotor;

    private final TalonFXConfigurator m_InConfigurator;

    private final Slot0Configs m_InConfigs;

    private final SparkFlex m_Linear;

    private final SparkClosedLoopController m_LinearController;

    private final SparkMaxConfig m_LinearConfig;

    private final RelativeEncoder m_LinEncoder;

    private VelocityVoltage speedRequest = new VelocityVoltage(0);

    private double setSpeed = 0;
    private double setPos = 0;
    
    public Intake(){
        m_InMotor = new TalonFX(IntakeConstants.intakeCANID);

        m_InConfigurator = m_InMotor.getConfigurator();

        m_InConfigs = new Slot0Configs()
            .withKP(IntakeConstants.inkP)
            .withKI(IntakeConstants.inkI)
            .withKD(IntakeConstants.inkD)
            .withKV(IntakeConstants.inkV);

        m_Linear = new SparkFlex(IntakeConstants.linearCANID, MotorType.kBrushless);

        m_LinearController = m_Linear.getClosedLoopController();

        m_LinearConfig = new SparkMaxConfig();

        m_LinEncoder = m_Linear.getEncoder();

        m_LinearConfig
        .inverted(IntakeConstants.linInverted)
        .idleMode(IntakeConstants.linIdleMode)
        .closedLoop.pid(
            IntakeConstants.linkP,
            IntakeConstants.linkI,
            IntakeConstants.linkD
        );

        m_LinearConfig.encoder.inverted(IntakeConstants.linInverted);

        m_InConfigurator.apply(m_InConfigs);
        m_Linear.configure(m_LinearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakePosition(double pos){
        if(pos > IntakeConstants.intakeOutPos) {setPos = IntakeConstants.intakeOutPos;}
        else if(pos < IntakeConstants.intakeInPos) {setPos = IntakeConstants.intakeInPos;}
        else{setPos = pos;}

        m_LinearController.setSetpoint(setPos, ControlType.kPosition);
    }

    public void setIntakeSpeed(double speed){
        setSpeed = speed;
        m_InMotor.setControl(speedRequest.withVelocity(setSpeed));
    }

    public void stopWheels(){
        setIntakeSpeed(0);
    }

    public boolean getDone(){
        return  Math.abs(m_LinEncoder.getPosition() - setPos) < IntakeConstants.intakeMaxAllErr;
    }
}
