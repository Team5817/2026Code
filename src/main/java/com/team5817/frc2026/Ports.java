package com.team5817.frc2026;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
  /*
   * LIST OF CHANNEL AND CAN IDS
   *
   * Swerve Modules go:
   * 0 1
   * 2 3
   *
   * spotless:off
   */

  /* DRIVETRAIN CAN DEVICE IDS */
  public static final CanDeviceId FL_DRIVE = new CanDeviceId(1, "canivore1");
  public static final CanDeviceId FL_ROTATION = new CanDeviceId(5, "canivore1");
  public static final CanDeviceId FL_CANCODER = new CanDeviceId(1, "canivore1");

  public static final CanDeviceId FR_DRIVE = new CanDeviceId(2, "canivore1");
  public static final CanDeviceId FR_ROTATION = new CanDeviceId(6, "canivore1");
  public static final CanDeviceId FR_CANCODER = new CanDeviceId(2, "canivore1");

  public static final CanDeviceId BL_DRIVE = new CanDeviceId(3, "canivore1");
  public static final CanDeviceId BL_ROTATION = new CanDeviceId(7, "canivore1");
  public static final CanDeviceId BL_CANCODER = new CanDeviceId(3, "canivore1");

  public static final CanDeviceId BR_DRIVE = new CanDeviceId(4, "canivore1");
  public static final CanDeviceId BR_ROTATION = new CanDeviceId(8, "canivore1");
  public static final CanDeviceId BR_CANCODER = new CanDeviceId(4, "canivore1");

  /* SUBSYSTEM CAN DEVICE IDS*/
  public static final CanDeviceId INTAKE_PIVOT = new CanDeviceId(9);
  public static final CanDeviceId INTAKE_ROLLERS = new CanDeviceId(10);

  public static final CanDeviceId SPINDEXER_1 = new CanDeviceId(11);
  public static final CanDeviceId SPINDEXER_2 = new CanDeviceId(12);

  public static final CanDeviceId TURRET = new CanDeviceId(13);
  public static final CanDeviceId HOOD = new CanDeviceId(14);
  public static final CanDeviceId FLYWHEEL_1 = new CanDeviceId(15);
  public static final CanDeviceId FLYWHEEL_2 = new CanDeviceId(16);

  public static final CanDeviceId CLIMB = new CanDeviceId(17);

  public static final CanDeviceId INTAKE_CANCODER = new CanDeviceId(18);

  public static final CanDeviceId PIGEON = new CanDeviceId(19, "canivore1");

  // initally meant for rev blinkin, switch to candl?
  public static final CanDeviceId LEDS = new CanDeviceId(20, "rio");

  /* BEAM BREAK DIO CHANNELS */
  // public static final int INDEXER_BEAM_BREAK = 0;
}
