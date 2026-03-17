// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.*;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.*;

public class RobotContainer {
  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = TunerConstants.kSpeedAtTestVolts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  
  CommandXboxController m_DriverController = new CommandXboxController(0);

  private final CommandSwerveDrivetrain m_Drive = TunerConstants.createDrivetrain();
  private final Intake m_intake = new Intake();
  private final Spindexer m_Spindexer = new Spindexer();
  private final Uptake m_Uptake = new Uptake();
  private final Turret m_Turret = new Turret(m_Drive.getPoseSupplier(), m_Drive.getVelocitySupplier());
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
    NamedCommands.registerCommand("Intake Inter", new IntakeInter(m_intake));
    NamedCommands.registerCommand("Internal Stop", new SpindexerStop(m_Spindexer).alongWith(new UptakeStop(m_Uptake)));

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private void configureBindings() {

    //Drive Controls
    m_Drive.setDefaultCommand(
        m_Drive.manualDriveCommand(
            () -> -m_DriverController.getLeftY(), // Forward is negative Y
            () -> -m_DriverController.getLeftX(), // Left is negative X
            () -> -m_DriverController.getRightX(), // CCW is negative X
            MaxSpeed,
            MaxAngularRate
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
    m_DriverController.y().onTrue(new IntakeInter(m_intake));

    //Shooter Controls
    m_DriverController.start().onTrue(new InstantCommand(() -> m_Turret.toggleShooter()));

    //While trigger is held fire balls
    m_DriverController.leftTrigger(.5)
    .and(m_Turret.isShooterAtSpeed())
    .toggleOnTrue(new SpindexerDrive(m_Spindexer)
    .alongWith(new UptakeUp(m_Uptake))
    );

    //While trigger is not held stop spindex and uptake
    m_DriverController.leftTrigger(.5)
    .toggleOnFalse(new SpindexerStop(m_Spindexer)
    .alongWith(new UptakeStop(m_Uptake)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

    public void cancelAllCommands() {
    CommandScheduler.getInstance().cancelAll();
  }
}
