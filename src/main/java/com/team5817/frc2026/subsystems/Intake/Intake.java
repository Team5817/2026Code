package com.team5817.frc2026.subsystems.Intake;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.frc2026.subsystems.Intake.IntakeConstants.RollerConstants;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Intake extends Subsystem {

  private static IntakeRollers mIntakeRollers;
  private static IntakeDeploy mIntakeDeploy;
  @Getter private State mState = State.IDLE;

  public Intake(
      RollerSubsystemIO FeederIO, ServoMotorIO DeployLeftIO, ServoMotorIO DeployRightIO, DoubleSupplier velocitySupplier) {
        mIntakeRollers =
            new IntakeRollers(
                FeederIO, () -> RollerConstants.voltageMap.get(velocitySupplier.getAsDouble()));
        mIntakeDeploy = new IntakeDeploy(DeployLeftIO, DeployRightIO);
  }

  public enum State {
    IDLE(IntakeRollers.State.IDLE, IntakeDeploy.State.OUT),
    INTAKING(IntakeRollers.State.INTAKING, IntakeDeploy.State.OUT),
    SQUEEZING(IntakeRollers.State.INTAKING, IntakeDeploy.State.SQUEEZE),
    EXHAUSTING(IntakeRollers.State.EXHAUST, IntakeDeploy.State.OUT),
    STOW(IntakeRollers.State.IDLE, IntakeDeploy.State.ZERO);

    final IntakeRollers.State rollerState;
    final IntakeDeploy.State deployState;

    State(IntakeRollers.State rollerState, IntakeDeploy.State deployState) {
      this.rollerState = rollerState;
      this.deployState = deployState;
    }
  }

  @Override
  public void readPeriodicInputs() {
    mIntakeRollers.readPeriodicInputs();
    mIntakeDeploy.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    mIntakeRollers.writePeriodicOutputs();
    mIntakeDeploy.writePeriodicOutputs();
  }

  @Override
  public void stop() {
    mIntakeRollers.stop();
    mIntakeDeploy.stop();
  }

  @Override
  public boolean checkDeviceConfiguration() {
    return mIntakeRollers.checkDeviceConfiguration() && mIntakeDeploy.checkDeviceConfiguration();
  }

  @Override
  public boolean checkSystem() {
    return mIntakeRollers.checkSystem() && mIntakeDeploy.checkSystem();
  }

  public void conformToState(State state) {
    stateRequest(state).act();
  }

  public Request stateRequest(State state) {
    return new ParallelRequest(
        new LambdaRequest(() -> this.mState = state),
        mIntakeRollers.stateRequest(state.rollerState),
        mIntakeDeploy.stateRequest(state.deployState));
  }
  public void home(){
    mIntakeDeploy.home();
  }

  @Override
  public void outputTelemetry() {
    mIntakeDeploy.outputTelemetry();
    mIntakeRollers.outputTelemetry();
    RobotVisualizer.updateIntake(mIntakeDeploy.getPosition());
    Logger.recordOutput("Intake/Main State", mState);
  }
}

