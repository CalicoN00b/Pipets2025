package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import net.calicoctl.bulldoglib.control.BulldogTalonFX;

public class CoralFlipperSubsystem extends SubsystemBase {

    private final BulldogTalonFX flipMotor;

    public CoralFlipperSubsystem() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = CoralConstants.kP;
        config.Slot0.kI = CoralConstants.kI;
        config.Slot0.kD = CoralConstants.kD;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40; // 40 amp breaker on PDH
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.SensorToMechanismRatio = 36; // 36:1 gear ratio

        flipMotor = new BulldogTalonFX(CoralConstants.flipMotorID, "CoralFlipMotor", config);
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

    public double getSupplyCurrent() {
        return flipMotor.getSupplyCurrent();
    }

    public void setCoralFlipperManual(double speed) {
        flipMotor.set(speed);
    }

    public void stopMotorsManual() {
        flipMotor.stop();
    }
}
