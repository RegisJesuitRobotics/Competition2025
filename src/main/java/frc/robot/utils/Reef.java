package frc.robot.utils;

import java.util.HashMap;

public enum Reef {
  MidUp("MID-UP-REEF.path"),
  Mid("MID-REEF.path"),
  MidDown("MID-DOWN-REEF.path"),
  BackUp("BACK-UP-REEF.path"),
  Back("BACK-REEF.path"),
  BackDown("BACK-DOWN-REEF.path");

  public final String value;
  private static HashMap<String, Reef> _map = null;

  private Reef(String value) {
    this.value = value;
  }

  public static Reef valueOf(int value) {
    Reef retval = (Reef) _map.get(value);
    return retval != null ? retval : values()[0];
  }

  static {
    _map = new HashMap();

    for (Reef type : values()) {
      _map.put(type.value, type);
    }
  }
}
