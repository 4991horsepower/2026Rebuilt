package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ShooterConstants;

import org.ejml.sparse.csc.mult.MatrixVectorMultWithSemiRing_DSCC;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;


public class Shooter extends SubsystemBase {
    private final TalonFX m_Hood;
    private final TalonFX m_Wheel;

    private final SlotConfigs m_hoodConfigs;
    private final SlotConfigs m_wheelConfigs;
    private final TalonFXConfigurator m_HoodConfigurator;
    private final TalonFXConfigurator m_WheelConfigurator;

    private double setHoodAngle = 0;
    private double setShooterSpeed = 0;
    private double setMode = 0;

    public Shooter(){
        m_Hood = new TalonFX(ShooterConstants.hoodCANID);
        m_Wheel = new TalonFX(ShooterConstants.shooterCANID);

        m_HoodConfigurator = m_Hood.getConfigurator();
        m_WheelConfigurator = m_Wheel.getConfigurator();

        m_hoodConfigs = new SlotConfigs();
        m_wheelConfigs = new SlotConfigs();

        m_hoodConfigs.kP = ShooterConstants.hoodkP;
        m_hoodConfigs.kI = ShooterConstants.hoodkI;
        m_hoodConfigs.kD = ShooterConstants.hoodkD; 
        
        
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
        //If the Shooter is active and at speed send true to SmartDashboard
        SmartDashboard.putBoolean("Ready to Fire", isShooterAtSpeed() && setShooterSpeed > 0);
       }

        //Setters
       public void setHoodAngle(double hoodAngle){
        //if inputed angle is within bounds set desired hood angle as input otherwise set as closet allowed angle
            if(hoodAngle > ShooterConstants.maxHoodAngle) {setHoodAngle = ShooterConstants.maxHoodAngle * ShooterConstants.kHoodGearRatio;}
            else if(hoodAngle < ShooterConstants.minHoodAngle) {setHoodAngle = ShooterConstants.minHoodAngle * ShooterConstants.kHoodGearRatio;}
            else setHoodAngle = hoodAngle * ShooterConstants.kHoodGearRatio;
            //Needs Converted from Angle to Motor Rotations
            m_Hood.setPosition(setHoodAngle);
       }

       public void setShooterSpeed(double speed){
            setShooterSpeed = speed;

            m_Wheel.set(setShooterSpeed);
       }

       public void setTargetingMode(int mode){
        //Mode 0 is no targeting, Mode 1 Hub Tracking Based on Position, Mode 2 Hub Tracking Based on April Tags, Mode 3 Player Station Wall Tracking
        setMode = mode;
       }

       //Stops
       public void stopHood(){
        setHoodAngle = m_Hood.getPosition().getValueAsDouble();

        m_Hood.setPosition(setHoodAngle);
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
    }

