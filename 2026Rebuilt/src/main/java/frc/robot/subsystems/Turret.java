package frc.robot.subsystems;



import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

    private Pose2d m_TurretPosition;

    private double m_RobotThetaToTarget;

    private Pose2d target = AimingConstants.hub;

    private Pose2d m_TestPose;

    public Turret(Supplier<Pose2d> poseSupplier){
        m_Turret = new TalonFXS(TurretConstants.turretCANID,"Default Name");

        m_TurretConfigurator = m_Turret.getConfigurator();

        m_TurretConfig = new Slot0Configs()
        .withKP(TurretConstants.turretkP)
        .withKI(TurretConstants.turretkI)
        .withKD(TurretConstants.turretkD);

        m_TurretConfigurator.apply(m_TurretConfig);

        m_PoseSupplier = poseSupplier;

        }

        public void periodic(){
        m_TurretPosition = m_PoseSupplier.get().relativeTo(target);

        m_RobotThetaToTarget = Math.atan2(m_TurretPosition.getX() , m_TurretPosition.getY());
        setTurretAngle(m_TurretPosition.getRotation().getRotations());

        SmartDashboard.putNumber("Robot X", m_TurretPosition.getX());
        SmartDashboard.putNumber("Robot Y", m_TurretPosition.getX());
        SmartDashboard.putNumber("Robot Theta", m_TurretPosition.getRotation().getDegrees());
        SmartDashboard.putNumber("Turret Angle", getTurretAngle()*360);
        }

        public void setTurretAngle(double angle){
            //receives angle in rotations
            //Keeps Turret Bound to .75 rotations 
            if(angle > Units.degreesToRotations(135)) {setTurretAngle = .375;}
            else if(angle < Units.degreesToRotations(-135)) {setTurretAngle = -.375;}
            else {setTurretAngle = angle;}
            
            //Sets Turret position to the converted angle
            m_Turret.setPosition(setTurretAngle);
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
            m_Turret.setPosition(m_Turret.getPosition().getValueAsDouble());
        }
}
