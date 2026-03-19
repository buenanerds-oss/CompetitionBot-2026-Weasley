package frc.robot.SubSystem.FuelControl.Shooter;

public interface ShooterIO {

    /**
     * 
     * @param invert - shoots inwards instead of outwards
     */
    public default void shoot(boolean invert) {

    }

    public default void stop() {
        
    }

    public default boolean isShooting() {
        return false;
    }

    public default void periodic() {
        
    }
}
