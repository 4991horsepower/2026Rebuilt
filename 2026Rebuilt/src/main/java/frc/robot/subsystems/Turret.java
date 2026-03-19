package frc.robot.subsystems;



import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AimingData;
import frc.robot.Constants.AimingConstants;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final TalonFXS m_Turret;

    private final TalonFXSConfigurator m_TurretConfigurator;

    private final Slot0Configs m_TurretConfig;

    private final TalonFXSConfiguration m_TurretCurrentConfig;

    private double setTurretAngle = 0;

    private Transform2d m_TurretPosition;

    private Rotation2d m_RobotThetaToTarget;

    private Pose2d target;

    private final StatusSignal<Angle> m_positionSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    
    private volatile double m_latchedAngle = 0;
    private volatile double m_latchedVelocity = 0;

    private double distance;

    private final TalonFX m_Hood;
    private final TalonFX m_Wheel;

    private final Supplier<Pose2d> m_PoseSupplier;
    private final Supplier<Pose2d> m_VelocitySupplier;

    private final SlotConfigs m_hoodConfigs;
    private final SlotConfigs m_wheelConfigs;
    private final TalonFXConfigurator m_HoodConfigurator;
    private final TalonFXConfigurator m_WheelConfigurator;

    private double setHoodAngle = 0;
    private double setShooterSpeed = 0;

    private PositionTorqueCurrentFOC positionRequest; 
    private VelocityTorqueCurrentFOC speedRequest;

    private boolean shooterOn = true;
    private boolean aimedAtTarget = true;

    private final AimingData aimingData = new AimingData();

    private final Transform2d robotToTurretPivot = new Transform2d(LimelightConstants.kRobotToTurretX, LimelightConstants.kRobotToTurretY, new Rotation2d(0));
    

    public Turret(Supplier<Pose2d> poseSupplier, Supplier<Pose2d> velocitySupplier){
        m_Turret = new TalonFXS(TurretConstants.turretCANID,"Default Name");

        m_TurretConfigurator = m_Turret.getConfigurator();

        ExternalFeedbackConfigs m_EncoderConfig = new ExternalFeedbackConfigs()
        .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.PulseWidth)
        .withAbsoluteSensorOffset(0.027588);

        m_TurretConfig = new Slot0Configs()
        .withKP(TurretConstants.turretkP)
        .withKI(TurretConstants.turretkI)
        .withKD(TurretConstants.turretkD);

        m_TurretCurrentConfig = new TalonFXSConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLimitEnable(true))
        .withSlot0(m_TurretConfig)
        .withCommutation(new CommutationConfigs().
        withBrushedMotorWiring(BrushedMotorWiringValue.Leads_A_and_C)
        .withMotorArrangement(MotorArrangementValue.Brushed_DC))
        .withExternalFeedback(m_EncoderConfig)
        .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive));

        m_TurretConfigurator.apply(m_TurretCurrentConfig);
        m_TurretConfigurator.apply(m_EncoderConfig);

        m_Turret.setControl(new PositionVoltage(0));
        m_PoseSupplier = poseSupplier;
        m_VelocitySupplier = velocitySupplier;

        m_positionSignal = m_Turret.getPosition();
        m_velocitySignal = m_Turret.getVelocity();

        // Match the 250Hz odometry loop
        BaseStatusSignal.setUpdateFrequencyForAll(250, m_positionSignal, m_velocitySignal);

        m_Hood = new TalonFX(ShooterConstants.hoodCANID , "Default Name");
        m_Wheel = new TalonFX(ShooterConstants.shooterCANID , "Default Name");

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

        if(m_Turret.getPosition().getValueAsDouble() < -.5){
            m_Turret.setPosition(m_Turret.getPosition().getValueAsDouble() + 1);
        }
        else if (m_Turret.getPosition().getValueAsDouble() > .5){
             m_Turret.setPosition(m_Turret.getPosition().getValueAsDouble() - 1);
        }

    setHoodAngle(setHoodAngle);
    }

    public void periodic(){
        Pose2d robot_pose = m_PoseSupplier.get();

        //Target Selction
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
           //If Robot's x is greater than shooting area set target as wall
           if(robot_pose.getX() < AimingConstants.Red.edgeOfShootingArea){
            //set wall side based on Robot Y                    
            if(robot_pose.getY() < AimingConstants.Red.midFieldSplit){
                target = AimingConstants.Red.leftWall;
                setHoodAngle(6);
            }
            else{
                target = AimingConstants.Red.rightWall;
                setHoodAngle(6);
                }
            }
            else{
                target = AimingConstants.Red.hub;
                setHoodAngle(0);
            }
            }
        else{
            //If Robot's x is greater than shooting area set target as wall
            if(robot_pose.getX() > AimingConstants.Blue.edgeOfShootingArea){
                //set wall side based on Robot Y
                if(robot_pose.getY() > AimingConstants.Blue.midFieldSplit){
                    target = AimingConstants.Blue.leftWall;
                    setHoodAngle(6);
                }
                else{
                    target = AimingConstants.Blue.rightWall;
                    setHoodAngle(6);
                }
            }
            else{
                target = AimingConstants.Blue.hub;
                setHoodAngle(0);
            }
          } 

        double shot_time = aimingData.getShotTime(distance);

        Pose2d lookahead = m_VelocitySupplier.get().times(shot_time * 0.5);
        Transform2d lookahead_translation = new Transform2d(-lookahead.getX(), -lookahead.getY(), new Rotation2d());

        SmartDashboard.putNumber("Vel X", m_VelocitySupplier.get().getX());
        SmartDashboard.putNumber("Vel Y", m_VelocitySupplier.get().getY());

        //Get robot Position Relative to target
        m_TurretPosition = robot_pose.plus(robotToTurretPivot).minus(target.plus(lookahead_translation)).inverse();

        //Get angle to target
        m_RobotThetaToTarget = new Rotation2d(Math.atan2(m_TurretPosition.getY() , m_TurretPosition.getX()));

        setTurretAngle(m_RobotThetaToTarget.getRotations());

        distance = Math.sqrt(Math.pow(m_TurretPosition.getX(), 2) +  Math.pow(m_TurretPosition.getY(), 2));

        if(shooterOn && aimedAtTarget){
            setShooterSpeed(aimingData.getShotSpeed(distance));
        }
        else if(!shooterOn){
            setShooterSpeed(0);
        }


        SmartDashboard.putNumber("Distance To Target", distance);

        SmartDashboard.putNumber("Robot Real Theta", m_PoseSupplier.get().getRotation().getDegrees());
        SmartDashboard.putNumber("Robot X", robot_pose.getX());
        SmartDashboard.putNumber("Robot Y", robot_pose.getY());
        SmartDashboard.putNumber("Robot Theta", robot_pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Target Relative Theta", m_TurretPosition.getRotation().getDegrees());
        SmartDashboard.putNumber("Theta to Target", m_RobotThetaToTarget.getDegrees());
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        SmartDashboard.putNumber("Target X", target.getX());
        SmartDashboard.putNumber("Target Y", target.getY());
    }

    public void setTurretAngle(double angle){
        //receives angle in rotations
        //Keeps Turret Bound to .75 rotations 
        if(angle > Units.degreesToRotations(155)) {setTurretAngle = .431; aimedAtTarget = false;}
        else if(angle < Units.degreesToRotations(-155)) {setTurretAngle = -.431; aimedAtTarget = false;}
        else {setTurretAngle = angle; aimedAtTarget = true;}
        SmartDashboard.putNumber("Theta to Target (clipped)", Units.rotationsToDegrees(setTurretAngle));
        //Sets Turret position to the converted angle
        m_Turret.setControl(new PositionVoltage(setTurretAngle));
    }

    //Set the target for the turret to track
    public void setTarget(Translation2d target){
        //m_target = target;
    }

    public void updateSignals() {
        // refresh() pulls the latest data from the CAN bus into the signal object
        m_positionSignal.refresh();
        m_velocitySignal.refresh();
        
        // Store the raw rotations/velocity into the volatile variables
        m_latchedAngle = m_positionSignal.getValueAsDouble();
        m_latchedVelocity = m_velocitySignal.getValueAsDouble();
    }

    public double getTurretAngle() {
        //returns shooter angle in degrees
        return m_latchedAngle * 360.0;
    }

    public double getTurretVelocityDegreesPerSec() {
        return m_latchedVelocity * 360.0;
    }

    public BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[] { m_positionSignal, m_velocitySignal };
    }

    public void stop(){
        setTurretAngle(m_latchedAngle);
    }

     //Setters
    public void setHoodAngle(double hoodAngle){
        //if inputed angle is within bounds set desired hood angle as input otherwise set as closet allowed angle
        if(hoodAngle > ShooterConstants.maxHoodAngle) {setHoodAngle = ShooterConstants.maxHoodAngle;}
        else if(hoodAngle < ShooterConstants.minHoodAngle) {setHoodAngle = ShooterConstants.minHoodAngle;}
        else {setHoodAngle = hoodAngle;}
        
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

    public BooleanSupplier isShooterAtSpeed(){
        //Return true if shooter speed is within allowed error of target
        return () -> {
            return (Math.abs(setShooterSpeed - getShooterSpeed())) < ShooterConstants.kShooterMaxAllVelErr && shooterOn;
        };
    }

    public boolean isHoodAtAngle(){
        //Returns true if hood angle is within allowed error of target
        return Math.abs(setHoodAngle - getHoodAngle()) < ShooterConstants.kHoodMaxAllPosErr;
    }

    public void toggleShooter(){
        shooterOn = !shooterOn;
    }

    public void resetTurret(){
        if(m_Turret.getPosition().getValueAsDouble() < -.5){
            m_Turret.setPosition(m_Turret.getPosition().getValueAsDouble() + 1);
        }
        else if (m_Turret.getPosition().getValueAsDouble() > .5){
             m_Turret.setPosition(m_Turret.getPosition().getValueAsDouble() - 1);
        }
    }
}
