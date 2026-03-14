package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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

    private final DigitalInput m_Limit;

    private final TalonFXConfigurator m_InConfigurator;

    private final SlotConfigs m_InConfigs;

    private final SparkFlex m_Linear;

    private final SparkClosedLoopController m_LinearController;

    private final SparkMaxConfig m_LinearConfig;

    private final RelativeEncoder m_LinEncoder;

    private VelocityTorqueCurrentFOC speedRequest = new VelocityTorqueCurrentFOC(0);

    private double setSpeed = 0;
    private double setPos = 0;
    
    private boolean wasResetByLimit = false;
    public Intake(){
        m_InMotor = new TalonFX(IntakeConstants.intakeCANID, "Default Name");

        m_InConfigurator = m_InMotor.getConfigurator();

        m_InConfigs = new SlotConfigs()
            .withKP(IntakeConstants.inkP)
            .withKI(IntakeConstants.inkI)
            .withKD(IntakeConstants.inkD)
            .withKV(IntakeConstants.inkV);

        m_Limit = new DigitalInput(0);

        m_Linear = new SparkFlex(IntakeConstants.linearCANID, MotorType.kBrushless);

        m_LinearController = m_Linear.getClosedLoopController();

        m_LinearConfig = new SparkMaxConfig();

        m_LinEncoder = m_Linear.getEncoder();

        m_LinearConfig
        .idleMode(IntakeConstants.linIdleMode)
        .inverted(IntakeConstants.intakeInverted)
        .closedLoop.pid(
            IntakeConstants.linkP,
            IntakeConstants.linkI,
            IntakeConstants.linkD
        );

        m_InConfigurator.apply(m_InConfigs);
        m_Linear.configure(m_LinearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Desired Position",setPos);
        SmartDashboard.putNumber("Intake Current Position", m_LinEncoder.getPosition());

        //If Limit switch not toggled set wasResetByLimit to false
        if(!m_Limit.get() && wasResetByLimit){
            wasResetByLimit = false;
        }
        else if(!wasResetByLimit){
            m_LinEncoder.setPosition(0);
            wasResetByLimit = true;
        }
        

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
