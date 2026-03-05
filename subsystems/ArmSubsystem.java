package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax leftArmMotor = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax rightArmMotor = new SparkMax(3, MotorType.kBrushless);

    public ArmSubsystem() {
        SparkMaxConfig leftArmConfig = new SparkMaxConfig();
        leftArmConfig.inverted(false);
        leftArmConfig.idleMode(IdleMode.kBrake);
        leftArmConfig.smartCurrentLimit(40);
        leftArmConfig.openLoopRampRate(0.2);
        leftArmConfig.voltageCompensation(12.0);
        leftArmMotor.configure(leftArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightArmConfig = new SparkMaxConfig();
        rightArmConfig.inverted(true);
        rightArmConfig.idleMode(IdleMode.kBrake);
        rightArmConfig.smartCurrentLimit(40);
        rightArmConfig.openLoopRampRate(0.2);
        rightArmConfig.voltageCompensation(12.0);
        rightArmMotor.configure(rightArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double output) {
        leftArmMotor.set(output);
        rightArmMotor.set(output);
    }

    public void stop() {
        leftArmMotor.stopMotor();
        rightArmMotor.stopMotor();
    }
}
