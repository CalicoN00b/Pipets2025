package frc.robot.subsystems.coral;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import net.calicoctl.bulldoglib.control.BulldogSparkMax;

public class CoralWheelsSubsystem extends SubsystemBase {

    private final BulldogSparkMax wheelMotor;

    public CoralWheelsSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

        wheelMotor = new BulldogSparkMax(CoralConstants.wheelMotorID, "CoralWheelMotor", config, MotorType.kBrushless, false, Constants.globalTuning);        
    }

    public void stopWheelMotors() {
        wheelMotor.stop();
    }

    public void setWheelMotors(double speed) {
        wheelMotor.set(speed);
    }    
}
