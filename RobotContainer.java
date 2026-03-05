// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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

    // -------------------------
    // Mechanism Motors (REV on rioCAN)
    // -------------------------
    private final SparkMax intakeMotor = new SparkMax(1, MotorType.kBrushless);

    private final SparkMax leftArmMotor = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax rightArmMotor = new SparkMax(3, MotorType.kBrushless);

    private final SparkMax feederMotor = new SparkMax(4, MotorType.kBrushless);

    // Shooter (Kraken X60 = TalonFX)
    // NOTE: This uses default CAN bus (roboRIO). If you want CANivore later, use:
    // new TalonFX(20, TunerConstants.kCANBus.getName());
    private final TalonFX shooterLeft = new TalonFX(20);
    private final TalonFX shooterRight = new TalonFX(21);

    private final DutyCycleOut shooterOutput = new DutyCycleOut(0);

    public RobotContainer() {
        // PathPlanner Named Commands (must be registered BEFORE buildAutoChooser)
        NamedCommands.registerCommand("Wait3Seconds", new WaitCommand(3.0));
        NamedCommands.registerCommand("Wait Event", new WaitCommand(3.0));
        NamedCommands.registerCommand(
            "ShootTime",
            new StartEndCommand(
                () -> {
                    shooterLeft.setControl(shooterOutput.withOutput(1.0));
                    shooterRight.setControl(shooterOutput.withOutput(1.0));
                    feederMotor.set(1.0);
                },
                () -> {
                    shooterLeft.setControl(shooterOutput.withOutput(0.0));
                    shooterRight.setControl(shooterOutput.withOutput(0.0));
                    feederMotor.stopMotor();
                }
            ).withTimeout(7.0)
        );

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Mechanism motor inversion defaults (adjust after quick test)
        intakeMotor.setInverted(false);

        leftArmMotor.setInverted(false);
        rightArmMotor.setInverted(true); // mirror arm

        feederMotor.setInverted(false);

        TalonFXConfiguration shooterCfg = new TalonFXConfiguration();
shooterCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
shooterRight.getConfigurator().apply(shooterCfg);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
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
            new InstantCommand(() -> intakeMotor.set(0.9))
        ).onFalse(
            new InstantCommand(intakeMotor::stopMotor)
        );

        joystick.leftTrigger(0.15).whileTrue(
            new InstantCommand(() -> intakeMotor.set(-0.6))
        ).onFalse(
            new InstantCommand(intakeMotor::stopMotor)
        );

        // Arms on POV left/right (avoids LB which is already seedFieldCentric)
        joystick.povRight().whileTrue(
            new InstantCommand(() -> {
                leftArmMotor.set(0.6);
                rightArmMotor.set(0.6);
            })
        ).onFalse(
            new InstantCommand(() -> {
                leftArmMotor.stopMotor();
                rightArmMotor.stopMotor();
            })
        );

        joystick.povLeft().whileTrue(
            new InstantCommand(() -> {
                leftArmMotor.set(-0.6);
                rightArmMotor.set(-0.6);
            })
        ).onFalse(
            new InstantCommand(() -> {
                leftArmMotor.stopMotor();
                rightArmMotor.stopMotor();
            })
        );

        // Feeder on RB (LB is taken)
        joystick.rightBumper().whileTrue(
            new InstantCommand(() -> feederMotor.set(0.8))
        ).onFalse(
            new InstantCommand(feederMotor::stopMotor)
        );

        // Shooter spin on X (NOTE: X is used with back/start for SysId, but not alone)
        joystick.x().whileTrue(
            new InstantCommand(() -> {
                shooterLeft.setControl(shooterOutput.withOutput(0.85));
                shooterRight.setControl(shooterOutput.withOutput(0.85));
            })
        ).onFalse(
            new InstantCommand(() -> {
                shooterLeft.setControl(shooterOutput.withOutput(0.0));
                shooterRight.setControl(shooterOutput.withOutput(0.0));
            })
        );

        // Shoot (shooter + feeder) on Y (same note as X)
        joystick.y().whileTrue(
            new InstantCommand(() -> {
                shooterLeft.setControl(shooterOutput.withOutput(1.0));
                shooterRight.setControl(shooterOutput.withOutput(1.0));
                feederMotor.set(1.0);
            })
        ).onFalse(
            new InstantCommand(() -> {
                shooterLeft.setControl(shooterOutput.withOutput(0.0));
                shooterRight.setControl(shooterOutput.withOutput(0.0));
                feederMotor.stopMotor();
            })
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
