package frc.robot.utils;

import java.util.HashMap;

public enum Reef {
  MidUpAlgae("MID-UP-REEF-ALGAE.path"),
  MidAlgae("MID-REEF-ALGAE.path"),
  MidDownAlgae("MID-DOWN-REEF-ALGAE.path"),
  StationUpAlgae("STATION-UP-REEF-ALGAE.path"),
  StationAlgae("STATION-REEF-ALGAE.path"),
  StationDownAlgae("STATION-DOWN-REEF-ALGAE.path"),
  MidUpCoralLeft("MID-UP-REEF-LEFT_CORAL.path"),
  MidUpCoralRight("MID-UP-REEF-RIGHT_Coral.path"),
  MidCoralLeft("MID-REEF-LEFT_CORAL.path"),
  MidCoralRight("MID-REEF-RIGHT_CORAL.path"),
  MidDownCoralLeft("MID-DOWN-REEF-LEFT_CORAL.path"),
  MidDownCoralRight("MID-DOWN-REEF-RIGHT_CORAL.path"),
  StationUpCoralLeft("STATION-UP-REEF-LEFT_CORAL.path"),
  StationUpCoralRight("STATION-UP-REEF-RIGHT_CORAL.path"),
  StationCoralLeft("STATION-REEF-LEFT_CORAL.path"),
  StationCoralRight("STATION-REEF-RIGHT_CORAL.path"),
  StationDownCoralLeft("STATION-DOWN-REEF-LEFT_CORAL.path"),
  StationDownCoralRight("STATION-DOWN-REEF-RIGHT_CORAL.path");

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
