package com.team5817.lib.requests;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2026.planners.ShootingPlannerI;
import com.team5817.frc2026.subsystems.Superstructure;

public class AutoShootRequest extends Request {
	ShootingPlannerI planner;
	Superstructure s;
	public AutoShootRequest(ShootingPlannerI planner,Superstructure s){
		this.planner = planner;
		this.s = s;
	}
	@Override
	public void act() {
		s.mShooter.followPlan(true);
		Logger.recordOutput("Shooter/Should Shoot", planner.shouldShoot());
		// if(planner.shouldShoot())
		// 	s.mIndexer.setDesiredState(Superstructure.IndexerState.INDEX);
		// else 
		// 	s.mIndexer.setDesiredState(Superstructure.IndexerState.OFF);
	}
	@Override
	public void cleanup() {
		s.mShooter.followPlan(false);
	}
	@Override
	public boolean isFinished() {
		return false;//runs until interrupted
	}
}
