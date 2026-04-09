package com.team5817.lib.drivers.Rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.drivers.TalonFXFactory;
import com.team5817.lib.util.PhoenixUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class RollerSubsystemIOTalonFX implements RollerSubsystemIO {
  private final TalonFX mMain;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<Boolean> tempFault;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final VelocityDutyCycle velocityOut =
      new VelocityDutyCycle(0).withUpdateFreqHz(0).withSlot(0);
  private final TorqueCurrentFOC torqueCurrentOut = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0);

  private final TalonFXConfiguration config;
  private final double reduction;
  private final RollerConstantsTalonFX mConstants;

  public RollerSubsystemIOTalonFX(
      CanDeviceId id, RollerConstantsTalonFX mConstants, double reduction) {
    this.reduction = reduction;
    mMain = new TalonFX(id.getDeviceNumber(), id.getBus());

    config = TalonFXFactory.getDefaultConfig();

    config.Slot0.kP = mConstants.kKp;
    config.Slot0.kI = mConstants.kKi;
    config.Slot0.kD = mConstants.kKd;
    config.Slot0.kV = mConstants.kKv;
    config.Slot0.kA = mConstants.kKa;
    config.Slot0.kS = mConstants.kKs;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = mConstants.kRampRate;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = mConstants.kRampRate;
    config.OpenLoopRamps.TorqueOpenLoopRampPeriod = mConstants.kRampRate;

    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = mConstants.kRampRate;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = mConstants.kRampRate;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = mConstants.kRampRate;

    config.CurrentLimits.SupplyCurrentLimit = mConstants.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = mConstants.kEnableSupplyCurrentLimit;

    config.CurrentLimits.StatorCurrentLimit = mConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = mConstants.kEnableStatorCurrentLimit;

    config.Voltage.PeakForwardVoltage = mConstants.kMaxForwardOutput;
    config.Voltage.PeakReverseVoltage = mConstants.kMaxReverseOutput;

    config.MotorOutput.PeakForwardDutyCycle = mConstants.kMaxForwardOutput / 12.0;
    config.MotorOutput.PeakReverseDutyCycle = mConstants.kMaxReverseOutput / 12.0;

    config.MotorOutput.Inverted =
        (mConstants.counterClockwisePositive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive);

    config.MotorOutput.NeutralMode = mConstants.kNeutralMode;

    PhoenixUtil.tryUntilOk(5, () -> mMain.getConfigurator().apply(config));

    position = mMain.getPosition();
    velocity = mMain.getVelocity();
    appliedVoltage = mMain.getMotorVoltage();
    supplyCurrent = mMain.getSupplyCurrent();
    torqueCurrent = mMain.getTorqueCurrent();
    tempCelsius = mMain.getDeviceTemp();
    tempFault = mMain.getFault_DeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                8.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                tempFault));

    if (mConstants.kFollowerID != null) {
      TalonFXFactory.createPermanentFollowerTalon(
          mConstants.kFollowerID,
          mConstants.kMainConstants.id,
          mConstants.kFollowerOpposeMasterDirection
              ? MotorAlignmentValue.Opposed
              : MotorAlignmentValue.Aligned);
    }

    this.mConstants = mConstants;
  }

  @Override
  public void updateInputs(RollerSubsystemIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius, tempFault);

    inputs.data =
        new RollerSubsystemIOData(
            Units.rotationsToRadians(position.getValueAsDouble()) / reduction,
            Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction,
            appliedVoltage.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            tempCelsius.getValueAsDouble(),
            tempFault.getValue(),
            BaseStatusSignal.isAllGood(
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius,
                tempFault));
  }

  @Override
  public void runVolts(double volts) {
    mMain.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double amps) {
    mMain.setControl(torqueCurrentOut.withOutput(amps));
  }

  @Override
  public void runVelocity(double velocity) {
    mMain.setControl(velocityOut.withVelocity(velocity));
  }

  @Override
  public void setCurrentLimit(double currentLimit) {
    new Thread(
        () -> {
          config.withCurrentLimits(config.CurrentLimits.withStatorCurrentLimit(currentLimit));
          PhoenixUtil.tryUntilOk(5, () -> mMain.getConfigurator().apply(config));
        });
  }
}
