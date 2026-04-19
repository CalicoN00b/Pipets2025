package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import net.calicoctl.bulldoglib.control.BulldogTalonFX;

public class ElevatorSubsystem extends SubsystemBase {

    private final BulldogTalonFX primaryMotor;
    private final BulldogTalonFX followerMotor;

    public ElevatorSubsystem() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40; // 40 amp breaker on PDH
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(ElevatorConstants.maxPos).in(Units.Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(ElevatorConstants.minPos).in(Units.Rotations);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        config.Feedback.SensorToMechanismRatio = 9; // 9:1 gear ratio 

        primaryMotor = new BulldogTalonFX(ElevatorConstants.primaryElevatorID, "PrimaryElevatorMotor", config);
        followerMotor = new BulldogTalonFX(ElevatorConstants.followerElevatorID, "FollowerElevatorMotor", config).withLeader(primaryMotor, false);

        primaryMotor.resetPosition(0);
        followerMotor.resetPosition(0);

    }

    @Override
    public void periodic() {}

    public void setPosition(Angle height) {
        primaryMotor.setControl(new PositionVoltage(height.in(Units.Rotations)));
    }

    public void setNeutral() {
        primaryMotor.setControl(new NeutralOut());
        followerMotor.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Angle setpoint) {
        primaryMotor.resetPosition(setpoint.in(Units.Rotations));
    }

    public double getCurrentPosition() {
        return primaryMotor.getPosition();
    }

    public double getCurrentVelocity() {
        return primaryMotor.getVelocity();
    }

    public double getSupplyCurrent() {
        return primaryMotor.getSupplyCurrent();
    }

    public void setElevatorManual(double speed) {
        primaryMotor.set(speed);
    }

    public void stopMotorsManual() {
        primaryMotor.stop();
    }
}
