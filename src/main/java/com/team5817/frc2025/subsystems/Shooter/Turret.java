package com.team5817.frc2025.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;

public class Turret extends StateBasedServoMotorSubsystem<Turret.State> {

    private static final double kTightError = 1.0;
    private static final double kLooseError = 4.0;

        public Turret(
        ServoMotorIO io,
        DoubleSupplier hubAngleSupplier,
        DoubleSupplier lobAngleSupplier
    ) 
    {
        super(State.STOW, io);

        State.AIM.setSupplier(hubAngleSupplier);
        State.LOBBING.setSupplier(lobAngleSupplier);
    }

    public enum State implements ServoState {
        STOW(() -> 0.0, kLooseError),
        AIM(kTightError),//set in constructor
        LOBBING(kTightError);//set in constructor
        
        
        private DoubleSupplier demand;
        private final double allowableError;
        State(double allowableError) {
            this.demand = ()->0;
            this.allowableError = allowableError;
        }
        State(DoubleSupplier supplier, double allowableError) {
            this.demand = supplier;
            this.allowableError = allowableError;
        }

        void setSupplier(DoubleSupplier supplier) {
            this.demand = supplier;
        }


        @Override
        public double getDemand() {
            return demand.getAsDouble();
        }

        @Override
        public double getAllowableError() {
            return allowableError;
        }


        @Override
        public ControlState getControlState() {
            return ControlState.POSITION;
        }

        @Override
        public boolean isDisabled() {
            return false;
        }
    }


}
