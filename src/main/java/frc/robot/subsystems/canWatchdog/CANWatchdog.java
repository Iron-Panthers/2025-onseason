package frc.robot.subsystems.canWatchdog;


import frc.robot.subsystems.rgb.RGB.RGBMessages;

public class CANWatchdog{
    private final Thread canWatchdogThread;
    private CANWatchdogIO io;
        
          /** Creates a new CANWatchdog. */
    public CANWatchdog(CANWatchdogIO io) {
        canWatchdogThread = new Thread(this::periodic, "CAN Watchdog Thread");
        canWatchdogThread.setDaemon(true);
        canWatchdogThread.start();
        this.io = io;
    }

    public void periodic(){
        if (io.checkForMissingIds().length == 0) {
            RGBMessages.MISSING_CAN_DEVICE.setIsExpired(true);
        } else {
            RGBMessages.MISSING_CAN_DEVICE.setIsExpired(false);
        }

    }

    public void matchStarting() {
        canWatchdogThread.interrupt();
    }
}
