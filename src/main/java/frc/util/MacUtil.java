package frc.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.ctre.phoenix6.signals.RobotEnableValue;

import frc.robot.Constants.RobotType;

public class MacUtil {
  private MacUtil() {}
  public static final String macAddress;
  public static RobotType robotType;

  //determining the type of the robot based on the mac address (from their respective rio's) default is alpha bot
  static {
    macAddress = macToString(getMacAddress());
    SmartDashboard.putString("MAC address", macAddress);
    switch(macAddress) {
      //Alphabot MAC Address Below
      case "00:80:2F:32:FD:81"-> {
        robotType = RobotType.ALPHA;
      }
      //Comp Bot MAC Address Below
      case "00:80:2F:38:81:D4" -> {
        robotType = RobotType.COMP;
      } 
      default -> {
        robotType = RobotType.ALPHA;
      }
    
    }
  }

  private static void logErr(SocketException e) {
    System.out.print(
        "mac util, which is used to toggle const values based on the robot, threw SocketException: ");
    System.out.println(e);
  }

  /**
   * Gets the first readable mac address and returns it as an array of bytes. Never throws errors,
   * instead returning an empty array of bytes
   *
   * @return byte array of first mac address, or empty byte array if none could be read
   */
  public static byte[] getMacAddress() {

    // init dance to not throw an error
    Enumeration<NetworkInterface> networkInterfaces;

    try {
      networkInterfaces = NetworkInterface.getNetworkInterfaces();
    } catch (SocketException e) {
      logErr(e);
      return new byte[0];
    }

    while (networkInterfaces.hasMoreElements()) {
      try { // get hardware address can throw
        byte[] address = networkInterfaces.nextElement().getHardwareAddress();

        // the address may be null
        if (address == null) continue;

        return address;

      } catch (SocketException e) {
        logErr(e);
      }
    }

    // we couldn't read any network interfaces
    return new byte[0];
  }

  /**
   * Takes a mac address byte array, and returns a string with the mac address formatted to be human
   * readable.
   *
   * <p><code>byte[] address = {46, -75, -30, 99, 104, 4};</code> is formatted to <code>
   * "2E:B5:E2:63:68:04"
   * </code>
   *
   * <p>Safely handles empty byte array, by returning empty string, or byte array of zeros, by
   * returning zeroed mac address for as many bytes
   *
   * @param address the address, in bytes, to format
   * @return the String human readable equivalent of the byte array
   */
  public static String macToString(byte[] address) {
    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < address.length; i++) {
      if (i != 0) {
        builder.append(':');
      }
      builder.append(String.format("%02X", address[i]));
    }
    return builder.toString();
  }
}
