package frc.robot.SubSystem.FuelControl.Hopper;

import java.beans.Encoder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubSystem.Logging.NerdLog;

public class Hopper implements HopperIO{

    //for Shuffleboard:
    
    ShuffleboardTab tab;
    double requestedVoltage;

    //ShuffleBoard Entries:

    //logging:

    //functionals:
    SparkMax motor;
    RelativeEncoder encoder;
    public Hopper(SparkMax motor, ShuffleboardTab tab) {
        this.motor = motor;
        this.tab = tab;
        configureMotor(false);
        this.encoder = motor.getEncoder();
    }

    @Override
    public void hopperin() {
        motor.setVoltage(requestedVoltage);
    }

    @Override
    public void hopperOut() {
        motor.set(-requestedVoltage);
    }

    @Override
    public void periodic() {
        NerdLog.logDouble("Fuel Control/Hopper/ Hopper Velocity RadPerSec", Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()));
    }

    private void configureMotor(boolean invert) {
        SparkMaxConfig config = new SparkMaxConfig();
         config.inverted(invert)
        .voltageCompensation(12)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);
         config.signals.absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) 1000/100)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
