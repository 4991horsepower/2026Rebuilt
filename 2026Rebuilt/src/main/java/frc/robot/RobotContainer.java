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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.*;

public class RobotContainer {
  private AutoBuilder autoBuilder;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  
  CommandXboxController m_DriverController = new CommandXboxController(0);
  CommandXboxController m_CopilotController = new CommandXboxController(1);

  private final CommandSwerveDrivetrain m_Drive = TunerConstants.createDrivetrain();
  private final Intake m_intake = new Intake();
  private final Spindexer m_Spindexer = new Spindexer();
  private final Uptake m_Uptake = new Uptake();
  //private final Turret m_Turret = new Turret(m_Drive.getPoseSupplier());
  //private final Shooter m_Shooter = new Shooter(m_Drive.getPoseSupplier());


  public RobotContainer() {
    NamedCommands.registerCommand("Intake Out", new IntakeOut(m_intake));
    NamedCommands.registerCommand("Internal In", new SpindexerDrive(m_Spindexer).alongWith(new UptakeUp(m_Uptake)));
    configureBindings();
  }

  private void configureBindings() {

    //Drive Controlls
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_Drive.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_Drive.applyRequest(() ->
                drive.withVelocityX(-Math.signum(m_DriverController.getLeftY()) * Math.pow(m_DriverController.getLeftY(), 2) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(- Math.signum(m_DriverController.getLeftX()) * Math.pow(m_DriverController.getLeftX(), 2) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
        m_CopilotController.a().onChange(new IntakeOut(m_intake));
        m_CopilotController.b().onChange(new IntakeIn(m_intake));

        //Shooter Controls
        //m_CopilotController.start().onChange(new stopShooter(m_Shooter));

        //While trigger is held fire balls
        m_CopilotController.leftTrigger(.5)
        .onTrue(new SpindexerDrive(m_Spindexer)
        .alongWith(new UptakeUp(m_Uptake)));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

    public void cancelAllCommands() {
    CommandScheduler.getInstance().cancelAll();
  }
}
