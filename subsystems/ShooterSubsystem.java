package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterLeft = new TalonFX(20);
    private final TalonFX shooterRight = new TalonFX(21);

    private final DutyCycleOut shooterOutput = new DutyCycleOut(0);

    public ShooterSubsystem() {
        TalonFXConfiguration leftCfg = new TalonFXConfiguration();
        leftCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        leftCfg.CurrentLimits.StatorCurrentLimit = 80.0;

        TalonFXConfiguration rightCfg = new TalonFXConfiguration();
        rightCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        rightCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        rightCfg.CurrentLimits.StatorCurrentLimit = 80.0;

        shooterLeft.getConfigurator().apply(leftCfg);
        shooterRight.getConfigurator().apply(rightCfg);
    }

    public void setPercent(double output) {
        shooterLeft.setControl(shooterOutput.withOutput(output));
        shooterRight.setControl(shooterOutput.withOutput(output));
    }

    public void stop() {
        setPercent(0.0);
    }
}
