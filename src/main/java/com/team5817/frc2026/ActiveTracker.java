package com.team5817.frc2026;

import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.experimental.Accessors;

public class ActiveTracker {
    private static Boolean wonAuto;
    @Getter
    @Accessors(prefix = "is")
    private static boolean isActive = false;
    public static void updateActive(){
        if(wonAuto == null){
            String gameData = DriverStation.getGameSpecificMessage();
            if(gameData.length() > 0){
                wonAuto = gameData.charAt(0) == 'R';
            }else{
                return;
            }
        }
        double matchTime = DriverStation.getMatchTime();
        
        if(DriverStation.isAutonomous()){
            isActive = true;
        } else if(DriverStation.isTeleopEnabled()){
           if(matchTime < 30){
            isActive = true;
            return;
           }

           if(matchTime<55){
            isActive = wonAuto;
            return;
           }

           if(matchTime<80){
            isActive = !wonAuto;
            return;
           }

           if(matchTime<105){
            isActive = wonAuto;
            return;
           }

           if(matchTime<130){
            isActive = !wonAuto;
            return;
           }

            isActive = true;
        } else {
            isActive = false;
        }
    }
}
