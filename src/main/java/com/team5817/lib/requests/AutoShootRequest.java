package com.team5817.lib.requests;

import java.util.ArrayList;
import java.util.List;

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
		s.mShooter.setDesiredState(planner.recommendedShooterState());
		if(planner.shouldShoot())
			s.mIndexer.setDesiredState(Superstructure.IndexerState.INDEX);
		else 
			s.mIndexer.setDesiredState(Superstructure.IndexerState.OFF);
	}
	@Override
	public boolean isFinished() {
		return false;//runs until interrupted
	}
}
