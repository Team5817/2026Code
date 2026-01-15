package com.team5817.frc2025.subsystems.Shooter;

import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.requests.Request;

import lombok.Getter;

public class Shooter extends Subsystem{
    
    @Getter
    private final Turret turret;

    @Getter
    private final Hood hood;

    @Getter
    private final RollerSubsystem<ShooterConstants.FlywheelState> flywheel;


    public Shooter(
        Turret turret,
        Hood hood,
        RollerSubsystem<ShooterConstants.FlywheelState> flywheel
    ) 
    {

        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
    }

                                                                    //Preset Shots Requests//
    public Request lowShot() {
        return new Request() {
            @Override
            public void act() {
                turret.setState(Turret.State.HOLD);
                hood.setState(Hood.State.LOW);
                flywheel.setState(ShooterConstants.FlywheelState.SHOOT);
            }
        };
    }
        public Request midShot() {
        return new Request() {
            @Override
            public void act() {
                turret.setState(Turret.State.HOLD);
                hood.setState(Hood.State.MID);
                flywheel.setState(ShooterConstants.FlywheelState.SHOOT);
            }
        };
    }
      
    public Request highShot() {
        return new Request() {
            @Override
            public void act() {
                turret.setState(Turret.State.HOLD);
                hood.setState(Hood.State.HIGH);
                flywheel.setState(ShooterConstants.FlywheelState.SHOOT);
            }
        };
    }

}
