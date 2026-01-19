package com.team5817.frc2026.subsystems.Shooter;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.team5817.frc2026.planners.ShootingPlannerI;
import com.team5817.frc2026.planners.ShootingTarget;
import com.team5817.frc2026.subsystems.Shooter.ShooterConstants.FlywheelState;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Shooter extends Subsystem{
    
    @Getter private final Turret turret;
    @Getter private final Hood hood;
    @Getter private final RollerSubsystem<ShooterConstants.FlywheelState> flywheel;

    @Getter
    private ShootingPlannerI planner;
    public Shooter(
        ServoMotorIO turretIO,
        ServoMotorIO hoodIO,
        RollerSubsystemIO flywheelIO,
        ShootingPlannerI planner
    ){
        
    this.turret = new Turret(turretIO , planner.getTurretAngleSupplier(ShootingTarget.HUB), planner.getTurretAngleSupplier(ShootingTarget.LOB));
    this.hood = new Hood(hoodIO, planner.getHoodAngleSupplier(ShootingTarget.HUB), planner.getHoodAngleSupplier(ShootingTarget.LOB));
    FlywheelState.HUB.setSupplier(planner.getFlywheelSpeedSupplier(ShootingTarget.HUB));
    FlywheelState.LOB.setSupplier(planner.getFlywheelSpeedSupplier(ShootingTarget.LOB));
        this.flywheel = new RollerSubsystem<ShooterConstants.FlywheelState>(FlywheelState.IDLE, "Shoooter/Flywheel", flywheelIO);
        this.planner = planner;
    }   
    @Getter
    @Accessors(prefix = "m")
    private State mState = State.STOW;
    @Getter
    @Setter
    private State desiredState = State.AIM;
    private boolean atState = false;
    @AutoLogOutput(key = "Shooter/ForcedStow")
    private boolean forcedStow = false;
    public enum State {
            STOW(Turret.State.STOW, Hood.State.STOW, ShooterConstants.FlywheelState.IDLE),
            CLOSE(Turret.State.STOW, Hood.State.CLOSE, ShooterConstants.FlywheelState.CLOSE),
            FAR(Turret.State.STOW, Hood.State.FAR, ShooterConstants.FlywheelState.FAR),
            AIM(Turret.State.AIM, Hood.State.AIM, ShooterConstants.FlywheelState.HUB),
            LOB(Turret.State.LOBBING, Hood.State.LOBBING, ShooterConstants.FlywheelState.LOB);

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
    public void periodic() {
        if(forcedStow){
            desiredState = State.STOW;
        }
        atState = turret.atState() && hood.atState() && flywheel.atState() && !forcedStow;
        if(mState != desiredState){
            turret.setState(desiredState.turretState);
            hood.setState(desiredState.hoodState);
            flywheel.setState(desiredState.flywheelState);
            if(atState){
                mState = desiredState;
            }
        }
    }
    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Shooter/Current State", mState);
        Logger.recordOutput("Shooter/Desired State", desiredState);
        turret.outputTelemetry();
        hood.outputTelemetry();
        flywheel.outputTelemetry();
    }
    public Request stateRequest(State state) {
        return new Request() {
            @Override
            public void act() {
                setDesiredState(state);
            }
            @Override
            public boolean isFinished() {
                return atState;
            }
        };
    }

    @Override
    public void readPeriodicInputs() {
        flywheel.readPeriodicInputs();
        turret.readPeriodicInputs();
        hood.readPeriodicInputs();
    }
    @Override
    public void writePeriodicOutputs() {
        flywheel.writePeriodicOutputs();
        turret.writePeriodicOutputs();
        hood.writePeriodicOutputs();
    }
    public void forceStow(boolean forced) {
        forcedStow = forced;
    }
    }
