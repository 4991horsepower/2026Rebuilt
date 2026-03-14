// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.*;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.*;

public class RobotContainer {
  @SuppressWarnings("unused")
  private AutoBuilder autoBuilder;

  private double MaxSpeed = TunerConstants.kSpeedAtTestVolts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  
  CommandXboxController m_DriverController = new CommandXboxController(0);

  private final CommandSwerveDrivetrain m_Drive = TunerConstants.createDrivetrain();
  private final Intake m_intake = new Intake();
  private final Spindexer m_Spindexer = new Spindexer();
  private final Uptake m_Uptake = new Uptake();
  private final Turret m_Turret = new Turret(m_Drive.getPoseSupplier());
  private final Shooter m_Shooter = new Shooter(m_Drive.getPoseSupplier());
  private final Vision m_Vision = new Vision(m_Drive, m_Turret);
  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    m_Drive.registerTelemetry(state -> {
        logger.telemeterize(state);
        m_Turret.updateSignals();
        m_Vision.addBufferSample(state.Timestamp, m_Turret.getTurretAngle());
    });
    NamedCommands.registerCommand("Intake Out", new IntakeOut(m_intake));
    NamedCommands.registerCommand("Internal In", new SpindexerDrive(m_Spindexer).alongWith(new UptakeUp(m_Uptake)));
    NamedCommands.registerCommand("Intake Inter", getAutonomousCommand());

    configureBindings();
  }

  private double clip_and_rescale(double i, double dz)
  {
    if(Math.abs(i) < dz)
    {
      return 0;
    }
    else
    {
      return (i - dz) / (1 - dz);
    }
  }

  private void configureBindings() {

    //Drive Controls
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_Drive.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_Drive.applyRequest(() ->
                drive.withVelocityX(-Math.signum(clip_and_rescale(m_DriverController.getLeftY(), 0.05)) * Math.pow(clip_and_rescale(m_DriverController.getLeftY(), 0.05), 2) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(- Math.signum(clip_and_rescale(m_DriverController.getLeftX(), 0.05)) * Math.pow(clip_and_rescale(m_DriverController.getLeftX(), 0.05), 2) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-clip_and_rescale(m_DriverController.getRightX(), 0.05) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_Drive.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_DriverController.back().and(m_DriverController.y()).whileTrue(m_Drive.sysIdDynamic(Direction.kForward));
        m_DriverController.back().and(m_DriverController.x()).whileTrue(m_Drive.sysIdDynamic(Direction.kReverse));
        m_DriverController.start().and(m_DriverController.y()).whileTrue(m_Drive.sysIdQuasistatic(Direction.kForward));
        m_DriverController.start().and(m_DriverController.x()).whileTrue(m_Drive.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on a button press
        m_DriverController.a().onTrue(m_Drive.runOnce(() -> m_Drive.seedFieldCentric()));

        //Cancel All Commands
        m_DriverController.x().onTrue(new InstantCommand(()->cancelAllCommands()));


        //Intake Controls
        m_DriverController.x().onChange(new IntakeOut(m_intake));
        m_DriverController.b().onChange(new IntakeIn(m_intake));
        m_DriverController.back().onTrue(m_intake.runOnce(() -> m_intake.homeIntake()));

        //Shooter Controls
        m_DriverController.start().onChange(new stopShooter(m_Shooter));

        //While trigger is held fire balls
        m_DriverController.leftTrigger(.5)
        .toggleOnTrue(new ShooterSpinUp(m_Shooter)
        .andThen(new SpindexerDrive(m_Spindexer)
        .alongWith(new UptakeUp(m_Uptake))
        ));

        //While trigger is not held stop spindex and uptake
        m_DriverController.leftTrigger(.5)
        .toggleOnFalse(new SpindexerStop(m_Spindexer)
        .alongWith(new UptakeStop(m_Uptake)));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

    public void cancelAllCommands() {
    CommandScheduler.getInstance().cancelAll();
  }
}
