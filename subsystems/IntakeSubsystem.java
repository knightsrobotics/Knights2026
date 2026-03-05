package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(1, MotorType.kBrushless);

    public IntakeSubsystem() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.inverted(false);
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.smartCurrentLimit(40);
        intakeConfig.openLoopRampRate(0.2);
        intakeConfig.voltageCompensation(12.0);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intakeIn() {
        intakeMotor.set(0.9);
    }

    public void intakeOut() {
        intakeMotor.set(-0.6);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
