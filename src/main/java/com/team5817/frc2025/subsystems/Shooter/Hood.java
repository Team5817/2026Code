package com.team5817.frc2025.subsystems.Shooter;

import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;

public class Hood extends StateBasedServoMotorSubsystem<Hood.State> {

    final static double kTightError = 1.3;
    final static double kLooseError = 4.0;
  
   
    public Hood(ServoMotorIO io) {
        super(State.STOW, io);
    }

  /**
     * Hood states.
     * Angles are placeholders until CAD is finalized.
     */
    public enum State implements ServoState {
        IDLE(0.0, kLooseError),
        STOW(0.0, kLooseError),
        LOW(10.0, kTightError),
        MID(25.0, kTightError),
        HIGH(40.0, kTightError),
        AIM(0.0, kTightError),
        DISABLE;



    private double demand = 0.0;
    private double allowableError = 0.0;
    private boolean disabled = false;

    State(double demand, double allowableError) {
        this.demand = demand;
        this.allowableError = allowableError;
    }

    State() {
        this.disabled = true;
    }

    @Override
    public double getDemand() {
        return demand;
    }

    @Override
    public double getAllowableError() {
        return allowableError;
    }

    @Override
    public boolean isDisabled() {
        return disabled;
    }

    @Override
    public ControlState getControlState() {
        return ControlState.POSITION;
    }
}}