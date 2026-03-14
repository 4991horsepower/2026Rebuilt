package frc.robot.subsystems;



import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AimingConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final TalonFXS m_Turret;

    private final TalonFXSConfigurator m_TurretConfigurator;

    private final Slot0Configs m_TurretConfig;

    private double setTurretAngle = 0;

    private final Supplier<Pose2d> m_PoseSupplier;

    private Transform2d m_TurretPosition;

    private Rotation2d m_RobotThetaToTarget;

    private Pose2d target;

    public Turret(Supplier<Pose2d> poseSupplier){
        m_Turret = new TalonFXS(TurretConstants.turretCANID,"Default Name");

        m_TurretConfigurator = m_Turret.getConfigurator();

        m_TurretConfig = new Slot0Configs()
        .withKP(TurretConstants.turretkP)
        .withKI(TurretConstants.turretkI)
        .withKD(TurretConstants.turretkD);

        m_TurretConfigurator.apply(m_TurretConfig);

        m_Turret.setControl(new PositionVoltage(0));
        m_PoseSupplier = poseSupplier;

        }

        public void periodic(){
            Pose2d robot_pose = m_PoseSupplier.get();

            //Target Selction
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            {
                //If Robot's x is greater than shooting area set target as wall
                if(robot_pose.getX() > AimingConstants.Red.edgeOfShootingArea){
                    //set wall side based on Robot Y
                    if(robot_pose.getY() > AimingConstants.Red.midFieldSplit){
                        target = AimingConstants.Red.leftWall;
                    }
                    else{
                        target = AimingConstants.Red.rightWall;
                    }
                }
                else{
                    target = AimingConstants.Red.hub;
                }
            }
            else{
                //If Robot's x is greater than shooting area set target as wall
                if(robot_pose.getX() > AimingConstants.Blue.edgeOfShootingArea){
                    //set wall side based on Robot Y
                    if(robot_pose.getY() > AimingConstants.Blue.midFieldSplit){
                        target = AimingConstants.Blue.leftWall;
                    }
                    else{
                        target = AimingConstants.Blue.rightWall;
                    }
                }
                else{
                    target = AimingConstants.Blue.hub;
                }
            }   
            //Get robot Position Relative to target
            m_TurretPosition = robot_pose.minus(target).inverse();

            //Get angle to target
            m_RobotThetaToTarget = new Rotation2d(Math.atan2(m_TurretPosition.getY() , m_TurretPosition.getX()));

            setTurretAngle(m_RobotThetaToTarget.getRotations());

            SmartDashboard.putNumber("Robot Real Theta", m_PoseSupplier.get().getRotation().getDegrees());
            SmartDashboard.putNumber("Robot X", robot_pose.getX());
            SmartDashboard.putNumber("Robot Y", robot_pose.getY());
            SmartDashboard.putNumber("Robot Theta", robot_pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Target Relative Theta", m_TurretPosition.getRotation().getDegrees());
            SmartDashboard.putNumber("Theta to Target", m_RobotThetaToTarget.getDegrees());
            SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        }

        public void setTurretAngle(double angle){
            //receives angle in rotations
            //Keeps Turret Bound to .75 rotations 
            if(angle > Units.degreesToRotations(135)) {setTurretAngle = .375;}
            else if(angle < Units.degreesToRotations(-135)) {setTurretAngle = -.375;}
            else {setTurretAngle = angle;}

            //Sets Turret position to the converted angle
            m_Turret.setControl(new PositionVoltage(setTurretAngle));
        }

        //Set the target for the turret to track
        public void setTarget(Translation2d target){
            //m_target = target;
        }

        public double getTurretAngle(){
            //returns shooter angle in degrees
            return m_Turret.getPosition().getValueAsDouble();
        }

        public void stop(){
            setTurretAngle(m_Turret.getPosition().getValueAsDouble());
        }
}
