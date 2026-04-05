package frc.robot.SubSystem.FuelControl.Hopper;

public interface HopperIO {

    /**
     * moves Fuel out of Hopper
     */
    public default void hopperOut() {

    }

    /**
     * moves fuel into hopper
     */
    public default void hopperin() {

    }

    public default void forcesetVoltage(double voltage) {

    }

    public default void periodic() {
        
    }
}
