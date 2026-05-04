package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import net.calicoctl.bulldoglib.control.BulldogSparkMax;

public class AlgaeWheelsSubsystem extends SubsystemBase {

    private final BulldogSparkMax rightMotor;
    private final BulldogSparkMax leftMotor;

    public AlgaeWheelsSubsystem() {
        rightMotor = new BulldogSparkMax(AlgaeConstants.rightMotorID, "RightAlgaeWheelMotor", new SparkMaxConfig(), MotorType.kBrushless, false, Constants.globalTuning);
        leftMotor = new BulldogSparkMax(AlgaeConstants.leftMotorID, "LeftAlgaeWheelMotor", new SparkMaxConfig(), MotorType.kBrushless, false, Constants.globalTuning);
    }

    public void stopWheelMotors() {
        rightMotor.stop();
        leftMotor.stop();
    }

    public void setWheelMotors(double speed) {
        rightMotor.set(speed);
        leftMotor.set(-speed);
    }
    
}
