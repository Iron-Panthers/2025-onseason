package frc.robot.subsystems.canWatchdog;

import java.util.stream.Stream;

public interface CANWatchdogIO {
    class CANWatchdogIOInputs {}

    default Stream<Integer> getIds(String jsonBody) {
        return Stream.of(0);
    }

    default int[] checkForMissingIds() {
        return new int[0];
    }
    
    default void sleep(final int milliseconds){}
    
} 