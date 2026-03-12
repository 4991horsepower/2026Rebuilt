package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

public class VelocityTest extends SubsystemBase {

  private final DigitalInput trig0;
  private final DigitalInput trig1;
  private AsynchronousInterrupt interrupt0;
  private AsynchronousInterrupt interrupt1;
  final DoubleTopic m_MeanTopic;
  final DoubleTopic m_VarianceTopic;
  final DoublePublisher m_MeanVelocityPublisher;
  final DoublePublisher m_VarianceVelocityPublisher;
  private double ts0 = 0, ts1 = 0;
  private boolean new_vel = false;

  private ArrayList<Double> velocityMeasurements = new ArrayList<>();
  
  private final NetworkTable m_smartDashboardTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  private final NetworkButton m_controlButton = new NetworkButton(m_smartDashboardTable, "Reset Velocity Measurement");

  public VelocityTest(int dio0, int dio1) {
    trig0 = new DigitalInput(dio0);
    trig1 = new DigitalInput(dio1);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_MeanTopic = inst.getDoubleTopic("/SmartDashboard/velocity_mean");
    m_MeanVelocityPublisher = m_MeanTopic.publish(PubSubOption.sendAll(true));
    m_VarianceTopic = inst.getDoubleTopic("/SmartDashboard/velocity_variance");
    m_VarianceVelocityPublisher = m_VarianceTopic.publish(PubSubOption.sendAll(true));

    interrupt0 = new AsynchronousInterrupt(trig0, (rising, falling) -> {
      if (falling) {
        ts0 = interrupt0.getFallingTimestamp();
      }
    });

    interrupt1 = new AsynchronousInterrupt(trig1, (rising, falling) -> {
        ts1 = interrupt1.getFallingTimestamp();
        new_vel = true;
    });

    interrupt0.setInterruptEdges(false, true);
    interrupt0.enable();
    interrupt1.setInterruptEdges(false, true);
    interrupt1.enable();
  }

  @Override
  public void periodic()
  {
    if(m_controlButton.getAsBoolean()) {
      velocityMeasurements.clear();
    }

    if(new_vel) {
      double velocity = 0.150 / (ts1 - ts0);

      velocityMeasurements.add(velocity);

      double mean = velocityMeasurements.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
      double variance = velocityMeasurements.stream().mapToDouble(v -> Math.pow(v - mean, 2)).average().orElse(0.0);

      m_MeanVelocityPublisher.set(mean);
      m_VarianceVelocityPublisher.set(variance);
      new_vel = false;
    }

  }

  public void close() {
    m_MeanVelocityPublisher.close();
  }
}
