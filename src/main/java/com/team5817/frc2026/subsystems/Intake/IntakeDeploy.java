package com.team5817.frc2026.subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.util.Util;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.ControlState;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends Subsystem {

    private final StateBasedServoMotorSubsystem<State> mRackLeft;
    private final StateBasedServoMotorSubsystem<State> mRackRight;

    @Getter private State mDesiredState = State.OUT;

    public IntakeDeploy(ServoMotorIO leftIO, ServoMotorIO rightIO) {
        mRackLeft  = new StateBasedServoMotorSubsystem<>(State.OUT, leftIO,  true);
        mRackRight = new StateBasedServoMotorSubsystem<>(State.OUT, rightIO, true);
    }

    public enum State implements ServoState {
        OUT(0.3175),
        SQUEEZE(0.1),
        ZERO(0),
        DISABLED();

        @Getter private double demand = 0;
        @Getter private double allowableError = 0.03;
        @Getter private boolean disabled = false;
        @Getter private NeutralModeValue neutralMode = NeutralModeValue.Brake;

        State(double position) {
            this.demand = position;
        }

        State() {
            this.disabled = true;
            this.neutralMode = NeutralModeValue.Coast;
        }

        @Override
        public ControlState getControlState() {
            return ControlState.POSITION;
        }
    }

    @Override
    public void readPeriodicInputs() {
        mRackLeft.readPeriodicInputs();
        mRackRight.readPeriodicInputs();
    }

    @Override
    public void writePeriodicOutputs() {
        boolean out = Util.epsilonEquals(
                mRackLeft.getPosition(),
                State.OUT.getDemand(),
                State.OUT.getAllowableError())
                &&
                Util.epsilonEquals(
                mRackRight.getPosition(),
                State.OUT.getDemand(),
                State.OUT.getAllowableError());
        Logger.recordOutput("Intake/Rack/Out", out);

        if (out && mDesiredState == State.OUT) {
            mRackLeft.setDesiredState(State.DISABLED);
            mRackRight.setDesiredState(State.DISABLED);
        }

        mRackLeft.writePeriodicOutputs();
        mRackRight.writePeriodicOutputs();
    }

    @Override
    public void stop() {
        mRackLeft.stop();
        mRackRight.stop();
    }

    @Override
    public boolean checkDeviceConfiguration() {
        return mRackLeft.checkDeviceConfiguration() && mRackRight.checkDeviceConfiguration();
    }

    @Override
    public boolean checkSystem() {
        return mRackLeft.checkSystem() && mRackRight.checkSystem();
    }

    @Override
    public void outputTelemetry() {
        mRackLeft.outputTelemetry();
        mRackRight.outputTelemetry();
        Logger.recordOutput("Intake/Rack/State", mDesiredState);
    }
    public void home(){
        mRackLeft.home();
        mRackRight.home();
        mRackLeft.setDesiredState(State.OUT);
        mRackRight.setDesiredState(State.OUT);
    }

    public Request stateRequest(State state) {
        return new ParallelRequest(
            new LambdaRequest(() -> {
                mDesiredState = state;
                mRackLeft.setHoming(false);
                mRackRight.setHoming(false);
                mRackLeft.setDesiredState(state);
                mRackRight.setDesiredState(state);
            })
        );
    }

    public double getPosition() {
        return (mRackLeft.getPosition() + mRackRight.getPosition()) / 2.0;
    }
}