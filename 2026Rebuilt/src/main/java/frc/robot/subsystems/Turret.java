package frc.robot.subsystems;



import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final TalonFXS m_Turret;

    private final TalonFXSConfigurator m_TurretConfigurator;

    private final Slot0Configs m_TurretConfig;

    private double setTurretAngle = 0;

    private final Supplier<Pose2d> m_poseSupplier;

    private Translation2d m_TurretPosition;

    public Turret(Supplier<Pose2d> poseSupplier){
        m_Turret = new TalonFXS(TurretConstants.turretCANID,"Default Name");

        m_TurretConfigurator = m_Turret.getConfigurator();

        m_TurretConfig = new Slot0Configs()
        .withKP(TurretConstants.turretkP)
        .withKI(TurretConstants.turretkI)
        .withKD(TurretConstants.turretkD);

        m_TurretConfigurator.apply(m_TurretConfig);

        m_poseSupplier = poseSupplier;

        }

        public void periodic(){

        }

        public void setTurretAngle(double angle){
            //receives angle in degrees
            //Keeps Turret Bound to 1 rotations 
            if(angle > 360) {setTurretAngle = 360;}
            else if(angle < 0) {setTurretAngle = 0;}
            else {setTurretAngle = angle;}
            
            //Sets Turret position to the converted angle
            m_Turret.setPosition(Units.degreesToRotations(setTurretAngle) * TurretConstants.turretGearRatio);
        }

        //Set the target for the turret to track
        public void setTarget(Translation2d target){
            //m_target = target;
        }

        public double getTurretAngle(){
            //returns shooter angle in degrees
            return m_Turret.getPosition().getValueAsDouble() / TurretConstants.turretGearRatio * 360;
        }

        public void stop(){
            m_Turret.setPosition(m_Turret.getPosition().getValueAsDouble());
        }
}
