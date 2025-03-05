package frc.robot.utils;

import java.util.HashMap;

public enum Reef {
  MidUpAlgae("MID-UP-REEF-ALGAE"),
  MidAlgae("MID-REEF-ALGAE"),
  MidDownAlgae("MID-DOWN-REEF-ALGAE"),
  StationUpAlgae("STATION-UP-REEF-ALGAE"),
  StationAlgae("STATION-REEF-ALGAE"),
  StationDownAlgae("STATION-DOWN-REEF-ALGAE"),
  MidUpCoralLeft("MID-UP-REEF-LEFT_CORAL"),
  MidUpCoralRight("MID-UP-REEF-RIGHT_CORAL"),
  MidCoralLeft("MID-REEF-LEFT_CORAL"),
  MidCoralRight("MID-REEF-RIGHT_CORAL"),
  MidDownCoralLeft("MID-DOWN-REEF-LEFT_CORAL"),
  MidDownCoralRight("MID-DOWN-REEF-RIGHT_CORAL"),
  StationUpCoralLeft("STATION-UP-REEF-LEFT_CORAL"),
  StationUpCoralRight("STATION-UP-REEF-RIGHT_CORAL"),
  StationCoralLeft("STATION-REEF-LEFT_CORAL"),
  StationCoralRight("STATION-REEF-RIGHT_CORAL"),
  StationDownCoralLeft("STATION-DOWN-REEF-LEFT_CORAL"),
  StationDownCoralRight("STATION-DOWN-REEF-RIGHT_CORAL");

  public final String value;
  private static HashMap<String, Reef> _map = null;

  private Reef(String value) {
    this.value = value;
  }

  public static Reef pathString(String value) {
    Reef retval = _map.get(value);
    return retval != null ? retval : values()[0];
  }

  static {
    _map = new HashMap<String, Reef>();

    for (Reef type : values()) {
      _map.put(type.value, type);
    }
  }
}
