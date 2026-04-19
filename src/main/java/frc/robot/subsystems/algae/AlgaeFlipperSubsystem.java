package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import net.calicoctl.bulldoglib.control.BulldogTalonFX;

public class AlgaeFlipperSubsystem extends SubsystemBase {
    
    // private final TalonFX flipMotor;
    // private final TalonFXConfiguration config;

    private final BulldogTalonFX flipMotor;

    public AlgaeFlipperSubsystem() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = AlgaeConstants.kP;
        config.Slot0.kI = AlgaeConstants.kI;
        config.Slot0.kD = AlgaeConstants.kD;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40; // 40 amp breaker on PDH
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(0).in(Units.Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(-0.275).in(Units.Rotations);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = 45; // 45:1 gear ratio

        flipMotor = new BulldogTalonFX(AlgaeConstants.flipMotorID, "AlgaeFlipMotor", config);

        flipMotor.resetPosition(0);

    }

    @Override
    public void periodic() {}

    public void setPosition(Angle angle) {
        flipMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
    }

    public void setNeutral() {
        flipMotor.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Angle setpoint) {
        flipMotor.resetPosition(setpoint.in(Units.Rotations));
    }

    public double getCurrentPosition() {
        return flipMotor.getPosition();
    }

    public double getCurrentVelocity() {
        return flipMotor.getVelocity();
    }

    public void setAlgaeFlipperManual(double speed) {
        flipMotor.set(speed);
    }

    public void stopMotorsManual() {
        flipMotor.stop();
    }
}
