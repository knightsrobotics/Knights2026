// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final double MaxSpeed =
            1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private final double MaxAngularRate =
            RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.0).withRotationalDeadband(MaxAngularRate * 0.0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    public RobotContainer() {
        // PathPlanner Named Commands (must be registered BEFORE buildAutoChooser)
        NamedCommands.registerCommand("Wait3Seconds", new WaitCommand(3.0));
        NamedCommands.registerCommand("Wait Event", new WaitCommand(3.0));
        NamedCommands.registerCommand(
            "ShootTime",
            new StartEndCommand(
                () -> {
                    shooter.setPercent(1.0);
                    feeder.set(1.0);
                },
                () -> {
                    shooter.stop();
                    feeder.stop();
                },
                shooter,
                feeder
            ).withTimeout(7.0)
        );

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Default swerve drive
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double x = -joystick.getLeftY();    // forward/back
                double y = -joystick.getLeftX();    // strafe
                double rot = -joystick.getRightX(); // rotate

                // Deadbands (stick units)
                x = MathUtil.applyDeadband(x, 0.10);
                y = MathUtil.applyDeadband(y, 0.10);
                rot = MathUtil.applyDeadband(rot, 0.08);

                // Square inputs for finer control (keep sign)
                x = Math.copySign(x * x, x);
                y = Math.copySign(y * y, y);
                rot = Math.copySign(rot * rot, rot);

                return drive.withVelocityX(x * MaxSpeed)
                            .withVelocityY(y * MaxSpeed)
                            .withRotationalRate(rot * MaxAngularRate);
            })
        );

        // Idle while disabled to ensure neutral mode applies
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Swerve extras
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5 * MaxSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
        );

        joystick.povDown().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-0.5 * MaxSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
        );

        // SysId routines (leave as-is)
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset drive orientation on right stick click (moved off Y)
        joystick.rightStick().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // -------------------------
        // Mechanism Controls (chosen to avoid swerve conflicts)
        // -------------------------
        // Intake IN/OUT on triggers
        joystick.rightTrigger(0.15).whileTrue(
            new StartEndCommand(intake::intakeIn, intake::stop, intake)
        );

        joystick.leftTrigger(0.15).whileTrue(
            new StartEndCommand(intake::intakeOut, intake::stop, intake)
        );

        // Arms on POV left/right (avoids LB which is already seedFieldCentric)
        joystick.povRight().whileTrue(
            new StartEndCommand(() -> arm.set(0.6), arm::stop, arm)
        );

        joystick.povLeft().whileTrue(
            new StartEndCommand(() -> arm.set(-0.6), arm::stop, arm)
        );

        // Feeder on RB (LB is taken)
        joystick.rightBumper().whileTrue(
            new StartEndCommand(() -> feeder.set(0.8), feeder::stop, feeder)
        );

        // Shooter spin on X (NOTE: X is used with back/start for SysId, but not alone)
        joystick.x()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .whileTrue(
            new StartEndCommand(() -> shooter.setPercent(0.85), shooter::stop, shooter)
        );

        // Shoot (shooter + feeder) on Y (same note as X)
        joystick.y()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .whileTrue(
            new StartEndCommand(
                () -> {
                    shooter.setPercent(1.0);
                    feeder.set(1.0);
                },
                () -> {
                    shooter.stop();
                    feeder.stop();
                },
                shooter,
                feeder
            )
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
