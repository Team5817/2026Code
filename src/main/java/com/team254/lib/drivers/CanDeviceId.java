package com.team254.lib.drivers;

import com.ctre.phoenix6.CANBus;

public class CanDeviceId {
  private final int mDeviceNumber;
  private final CANBus mBus;

  public CanDeviceId(int deviceNumber, String bus) {
    mDeviceNumber = deviceNumber;
    mBus = new CANBus(bus);
  }

  // Use the default bus name (empty string).
  public CanDeviceId(int deviceNumber) {
    this(deviceNumber, "rio");
  }

  public int getDeviceNumber() {
    return mDeviceNumber;
  }

  public CANBus getBus() {
    return mBus;
  }

  public boolean equals(CanDeviceId other) {
    return other.mDeviceNumber == mDeviceNumber && other.mBus == mBus;
  }
}
