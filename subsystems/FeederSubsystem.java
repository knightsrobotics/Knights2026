package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final SparkMax feederMotor = new SparkMax(4, MotorType.kBrushless);

    public FeederSubsystem() {
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.inverted(false);
        feederConfig.idleMode(IdleMode.kBrake);
        feederConfig.smartCurrentLimit(30);
        feederConfig.openLoopRampRate(0.1);
        feederConfig.voltageCompensation(12.0);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double output) {
        feederMotor.set(output);
    }

    public void stop() {
        feederMotor.stopMotor();
    }
}
