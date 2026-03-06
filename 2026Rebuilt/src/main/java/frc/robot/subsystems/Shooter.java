package frc.robot.subsystems;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;


public class Shooter extends SubsystemBase {
    private final TalonFX m_Hood;
    private final TalonFX m_Wheel;

    private final SlotConfigs m_hoodConfigs;
    private final SlotConfigs m_wheelConfigs;
    private final TalonFXConfigurator m_HoodConfigurator;
    private final TalonFXConfigurator m_WheelConfigurator;

    private final AnalogInput m_Limit;

    private double setHoodAngle = 0;
    private double setShooterSpeed = 0;

    private PositionDutyCycle positionRequest; 
    private VelocityDutyCycle speedRequest;

    private boolean isHoming = false;
    private boolean wasResetByLimit = false;

    public Shooter(){
        

        m_Hood = new TalonFX(ShooterConstants.hoodCANID);
        m_Wheel = new TalonFX(ShooterConstants.shooterCANID);

        m_Limit = new AnalogInput(0);

        m_HoodConfigurator = m_Hood.getConfigurator();
        m_WheelConfigurator = m_Wheel.getConfigurator();

        m_hoodConfigs = new SlotConfigs()
        .withKP(ShooterConstants.hoodkP)
        .withKI(ShooterConstants.hoodkI)
        .withKD(ShooterConstants.hoodkD);
        
        m_wheelConfigs = new SlotConfigs()
        .withKP(ShooterConstants.shooterkP)
        .withKI(ShooterConstants.shooterkI)
        .withKD(ShooterConstants.shooterkD);


        positionRequest = new PositionDutyCycle(setHoodAngle);
        speedRequest = new VelocityDutyCycle(setShooterSpeed);

        
        m_HoodConfigurator.apply(m_hoodConfigs);
        m_WheelConfigurator.apply(m_wheelConfigs);

        if(DebugConstants.Shooter.DebugEnable){
            SmartDashboard.putNumber("Commanded Hood Angle", setHoodAngle);
            SmartDashboard.putNumber("Actual Hood Angle", m_Hood.getPosition().getValueAsDouble());

            SmartDashboard.putNumber("Commanded Shooter Speed",setShooterSpeed);
            SmartDashboard.putNumber("Actual Shooter Speed", m_Wheel.getVelocity().getValueAsDouble());
        }

        
    }
    @Override
       public void periodic(){
        zeroHoodOnLimitSwitch();
        //If the Shooter is active and at speed send true to SmartDashboard
        SmartDashboard.putBoolean("Ready to Fire", isShooterAtSpeed() && setShooterSpeed > 0);

        if(isHoming){
            if(!(m_Limit.getValue() > 0))
            {
            m_Hood.setVoltage(ShooterConstants.hoodHomingVolts);
            }
            else{
                isHoming = false;
                setHoodAngle(0);
            }
        }
       }

        //Setters
       public void setHoodAngle(double hoodAngle){
        //if inputed angle is within bounds set desired hood angle as input otherwise set as closet allowed angle
            if(hoodAngle > ShooterConstants.maxHoodAngle) {setHoodAngle = ShooterConstants.maxHoodAngle * ShooterConstants.kHoodGearRatio;}
            else if(hoodAngle < ShooterConstants.minHoodAngle) {setHoodAngle = ShooterConstants.minHoodAngle * ShooterConstants.kHoodGearRatio;}
            else setHoodAngle = hoodAngle * ShooterConstants.kHoodGearRatio;
            //Needs Converted from Angle to Motor Rotations
        m_Hood.setControl(positionRequest.withPosition(setHoodAngle));
       }

       public void setShooterSpeed(double speed){
            setShooterSpeed = speed;

            m_Wheel.setControl(speedRequest.withVelocity(setShooterSpeed));
       }

       //Stops
       public void stopHood(){
        setHoodAngle = m_Hood.getPosition().getValueAsDouble();

        m_Hood.setControl(positionRequest.withPosition(setHoodAngle));
       }

       //Getters
       public double getHoodAngle(){
        //Returns Hood Position in Motor Rotations
        return m_Hood.getPosition().getValueAsDouble();
       }

       public double getShooterSpeed(){
        //returns Shooter speed in RPM
        return m_Wheel.getVelocity().getValueAsDouble();
       }

       public boolean isShooterAtSpeed(){
        //Return true if shooter speed is within allowed error of target
        return Math.abs(setShooterSpeed - getShooterSpeed()) < ShooterConstants.kShooterMaxAllVelErr;
       }

       public boolean isHoodAtAngle(){
        //Returns true if hood angle is within allowed error of target
        return Math.abs(setHoodAngle - getHoodAngle()) < ShooterConstants.kHoodMaxAllPosErr;
       }

       private void zeroHoodOnLimitSwitch() {
        if (!wasResetByLimit && m_Limit.getValue() > 0) {
          // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
          // prevent constant zeroing while pressed
          m_Hood.setPosition(ShooterConstants.hoodZeroAngle);
          wasResetByLimit = true;
        } else if (!(m_Limit.getValue() > 0)) {
          wasResetByLimit = false;
        }
      }
    }

