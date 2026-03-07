package frc.robot.subsystems;



import java.util.function.Supplier;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private final TalonFXS m_Turret;

    private final TalonFXSConfigurator m_TurretConfigurator;

    private final SlotConfigs m_TurretConfig;

    private double setTurretAngle = 0;

    private Supplier<Pose2d> m_poseSupplier;

    public Turret(Supplier<Pose2d> poseSupplier){
        m_Turret = new TalonFXS(TurretConstants.turretCANID);

        m_TurretConfigurator = m_Turret.getConfigurator();

        m_TurretConfig = new SlotConfigs();

        m_TurretConfig.kP = TurretConstants.turretkP;
        m_TurretConfig.kI = TurretConstants.turretkI;
        m_TurretConfig.kD = TurretConstants.turretkD;

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

        public double getTurretAngle(){
            //returns shooter angle in degrees
            return m_Turret.getPosition().getValueAsDouble() / TurretConstants.turretGearRatio * 360;
        }

        public void stop(){
            m_Turret.setPosition(m_Turret.getPosition().getValueAsDouble());
        }
}
