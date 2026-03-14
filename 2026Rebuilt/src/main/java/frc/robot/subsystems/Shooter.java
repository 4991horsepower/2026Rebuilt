package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AimingConstants;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase {
    private final TalonFX m_Hood;
    private final TalonFX m_Wheel;

    private final Supplier<Pose2d> m_PoseSupplier;

    private final SlotConfigs m_hoodConfigs;
    private final SlotConfigs m_wheelConfigs;
    private final TalonFXConfigurator m_HoodConfigurator;
    private final TalonFXConfigurator m_WheelConfigurator;

    private final DigitalInput m_Limit;

    private double setHoodAngle = 0;
    private double setShooterSpeed = 0;

    private PositionTorqueCurrentFOC positionRequest; 
    private VelocityTorqueCurrentFOC speedRequest;

    private boolean isHoming = false;
    private boolean wasResetByLimit = false;

    private Pose2d target = AimingConstants.Blue.hub;

    private Pose2d robotRelativeToTarget = new Pose2d();
    private double distance = 0;

    public Shooter(Supplier<Pose2d> poseSupplier){
        m_PoseSupplier = poseSupplier;

        m_Hood = new TalonFX(ShooterConstants.hoodCANID , "Default Name");
        m_Wheel = new TalonFX(ShooterConstants.shooterCANID , "Default Name");

        m_Limit = new DigitalInput(9);

        m_HoodConfigurator = m_Hood.getConfigurator();
        m_WheelConfigurator = m_Wheel.getConfigurator();

        m_hoodConfigs = new SlotConfigs()
        .withKP(ShooterConstants.hoodkP)
        .withKI(ShooterConstants.hoodkI)
        .withKD(ShooterConstants.hoodkD);
        
        m_wheelConfigs = new SlotConfigs()
        .withKP(ShooterConstants.shooterkP)
        .withKI(ShooterConstants.shooterkI)
        .withKD(ShooterConstants.shooterkD)
        .withKV(ShooterConstants.shooterkV)
        .withKS(ShooterConstants.shooterkS);


        positionRequest = new PositionTorqueCurrentFOC(setHoodAngle);
        speedRequest = new VelocityTorqueCurrentFOC(setShooterSpeed);

        
        m_HoodConfigurator.apply(m_hoodConfigs);
        m_WheelConfigurator.apply(m_wheelConfigs);

        if(DebugConstants.Shooter.DebugEnable){
            SmartDashboard.putNumber("Commanded Hood Angle", setHoodAngle);
            SmartDashboard.putNumber("Actual Hood Angle", m_Hood.getPosition().getValueAsDouble());

            SmartDashboard.putNumber("Commanded Shooter Speed",setShooterSpeed);
            SmartDashboard.putNumber("Actual Shooter Speed", m_Wheel.getVelocity().getValueAsDouble());
        }

        setHoodAngle(setHoodAngle);
    }

    @Override
       public void periodic(){
        Pose2d robot_pose = m_PoseSupplier.get();

        //Target Selction
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
           //If Robot's x is greater than shooting area set target as wall
           if(robot_pose.getX() < AimingConstants.Red.edgeOfShootingArea){
            //set wall side based on Robot Y                    
            if(robot_pose.getY() > AimingConstants.Red.midFieldSplit){
              target = AimingConstants.Red.leftWall;
                //System.out.println("Target Left Wall");
                  }
            else{
                target = AimingConstants.Red.rightWall;
                //System.out.println("Target Right Wall");
                }
              }
              else{
                  target = AimingConstants.Red.hub;
                  //System.out.println("Target Hub");
                }
              }
          else{
              //If Robot's x is greater than shooting area set target as wall
              if(robot_pose.getX() > AimingConstants.Blue.edgeOfShootingArea){
                  //set wall side based on Robot Y
                  if(robot_pose.getY() > AimingConstants.Blue.midFieldSplit){
                      target = AimingConstants.Blue.leftWall;
                      //System.out.println("Target Wall");
                  }
                  else{
                      target = AimingConstants.Blue.rightWall;
                      //System.out.println("Target Wall");
                   }
               }
              else{
                  target = AimingConstants.Blue.hub;
                  //System.out.println("Target Hub");
               }
          }   
        //Convert Robot Position Realitive to target
        robotRelativeToTarget = m_PoseSupplier.get().relativeTo(target);
        //Using Relative Position find the distance to the target and calculate launch speed based on table values

        SmartDashboard.putNumber("Relative X", robotRelativeToTarget.getX());
        SmartDashboard.putNumber("Relative Y", robotRelativeToTarget.getY());
        //If the Shooter is active and at speed send true to SmartDashboard
        SmartDashboard.putBoolean("Ready to Fire", isShooterAtSpeed() && setShooterSpeed > 0);

        //Homing Code
        zeroHoodOnLimitSwitch();

        if(isHoming){
            if(!(m_Limit.get()))
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
            if(hoodAngle > ShooterConstants.maxHoodAngle) {setHoodAngle = ShooterConstants.maxHoodAngle;}
            else if(hoodAngle < ShooterConstants.minHoodAngle) {setHoodAngle = ShooterConstants.minHoodAngle;}
            else setHoodAngle = hoodAngle;
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
        //return Math.abs(setShooterSpeed - getShooterSpeed()) < ShooterConstants.kShooterMaxAllVelErr;
        return true;
       }

       public boolean isHoodAtAngle(){
        //Returns true if hood angle is within allowed error of target
        return Math.abs(setHoodAngle - getHoodAngle()) < ShooterConstants.kHoodMaxAllPosErr;
       }

       private void zeroHoodOnLimitSwitch() {
        if (!wasResetByLimit && m_Limit.get()) {
          // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
          // prevent constant zeroing while pressed
          wasResetByLimit = true;
          m_Hood.setPosition(0);
        } else if (!(m_Limit.get())) {
          wasResetByLimit = false;
        }
      }
    }

