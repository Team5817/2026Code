package com.team5817.frc2025.subsystems.Shooter;


import com.team5817.frc2025.planners.ShootingPlannerI;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.requests.Request;

import lombok.Getter;

public class Shooter extends Subsystem{
    
    @Getter private final Turret turret;
    @Getter private final Hood hood;
    @Getter private final RollerSubsystem<ShooterConstants.FlywheelState> flywheel;

    ShootingPlannerI planner;
      public Shooter(
        ServoMotorIO turretIO,
        ServoMotorIO hoodIO,
        RollerSubsystem<ShooterConstants.FlywheelState> flywheel,
        ShootingPlannerI planner
        )
       
        {

        this.turret = new Turret(turretIO , planner.getHubTurretAngleSupplier(), planner.getLobTurretAngleSupplier());
        this.hood = new Hood(hoodIO, planner.getHubHoodAngleSupplier(), planner.getLobHoodAngleSupplier());
        this.flywheel = flywheel;
        this.planner = planner;

            //flywheel.setVelocitySupplier(FlywheelSpeedHub);
        }   
    
        
        //no shooter states yet, 
    public enum State {
        IDLE(Turret.State.STOW, Hood.State.STOW, ShooterConstants.FlywheelState.IDLE),
        LOW(Turret.State.STOW, Hood.State.LOW, ShooterConstants.FlywheelState.SHOOT),
        HIGH(Turret.State.STOW, Hood.State.HIGH, ShooterConstants.FlywheelState.SHOOT),
        AIM(Turret.State.AIM, Hood.State.AIM, ShooterConstants.FlywheelState.SHOOT),
        LOBBING(Turret.State.LOBBING, Hood.State.LOBBING, ShooterConstants.FlywheelState.SHOOT);

        final Turret.State turretState;
        final Hood.State hoodState;
        final ShooterConstants.FlywheelState flywheelState;

        State(Turret.State turretState, Hood.State hoodState, ShooterConstants.FlywheelState flywheelState) {
            this.turretState = turretState;
            this.hoodState = hoodState;
            this.flywheelState = flywheelState;
        }
    }





    @Override
    public void readPeriodicInputs() {
        turret.readPeriodicInputs();
        hood.readPeriodicInputs();
        flywheel.readPeriodicInputs();
        }

    @Override
    public void writePeriodicOutputs() {
        turret.writePeriodicOutputs();
        hood.writePeriodicOutputs();
        flywheel.writePeriodicOutputs();
        }

    @Override
    public void stop() {
        turret.stop();
        hood.stop();
        flywheel.stop();
        }

    @Override
    public boolean checkSystem() {
        return turret.checkSystem()
            && hood.checkSystem()
            && flywheel.checkSystem();
        }

    @Override
    public void outputTelemetry() {
        turret.outputTelemetry();
        hood.outputTelemetry();
        flywheel.outputTelemetry();
        }


//Preset Shots Requests//
    public Request lowShot() {
            return new Request() {
                @Override
                public void act() {
                    turret.setState(Turret.State.STOW);
                    hood.setState(Hood.State.LOW);
                    flywheel.setState(ShooterConstants.FlywheelState.SHOOT);
                }
            };
        }

        
    public Request highShot() {
            return new Request() {
                @Override
                public void act() {
                    turret.setState(Turret.State.STOW);
                    hood.setState(Hood.State.HIGH);
                    flywheel.setState(ShooterConstants.FlywheelState.SHOOT);
                }
            };
        }

    }
