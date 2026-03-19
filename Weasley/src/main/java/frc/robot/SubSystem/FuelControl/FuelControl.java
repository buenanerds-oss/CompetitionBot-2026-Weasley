package frc.robot.SubSystem.FuelControl;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubSystem.FuelControl.Hopper.Hopper;
import frc.robot.SubSystem.FuelControl.Hopper.HopperIO;
import frc.robot.SubSystem.FuelControl.Shooter.Shooter;
import frc.robot.SubSystem.FuelControl.Shooter.ShooterIO;

public class FuelControl{
    
    //for ShuffleBoard:
    ShuffleboardTab fuelTab;

    //logging:
    
    //functionals:
    HopperIO hopper;
    ShooterIO shooter;

    public FuelControl(SparkMax shootermotor, SparkMax hopperMotor) {
        fuelTab = Shuffleboard.getTab("Fuel Management");
        this.hopper = new Hopper(hopperMotor, fuelTab);
        this.shooter = new Shooter(hopperMotor, fuelTab);
    }

    public void startShooter() {
        shooter.shoot(false);
    }

    public void startShooterInverted() {
        if (!shooter.isShooting()) shooter.shoot(true);
    }


    //TODO: test if you can have it automatically start shooter when you set the hopper
    public void outtake() {
        if (shooter.isShooting()) hopper.hopperOut();
    }

    public void intake() {
        if (shooter.isShooting()) hopper.hopperin();
    }

    public void periodic() {
        shooter.periodic();
        hopper.periodic();
    }

    
}
