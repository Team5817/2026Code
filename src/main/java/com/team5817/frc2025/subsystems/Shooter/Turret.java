package com.team5817.frc2025.subsystems.Shooter;

import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;


public class Turret extends StateBasedServoMotorSubsystem<Turret.State> {

    final static double kTightError = 1.0;
    final static double kLooseError = 4.0;

    public Turret(ServoMotorIO io) {
        super(State.IDLE, io);
    }
    
    

      public enum State implements ServoState {
            IDLE(0.0, kLooseError),
            HOLD(0.0, kTightError),
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
    }
    }