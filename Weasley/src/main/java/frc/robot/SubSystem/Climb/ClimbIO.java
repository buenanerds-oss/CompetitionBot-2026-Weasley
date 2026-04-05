package frc.robot.SubSystem.Climb;

public interface ClimbIO {

    /**
     * robot climbs ONTO rod
     */
    public default void climbUp() {}
    
    /**
     * robot climbs OFF OF rod
     */
    public default void climbDown() {}

    public default void stop() {}

    public default boolean atLimit(boolean checkUpLimit) {
        return false;
    }

    public default void periodic() {}
}
