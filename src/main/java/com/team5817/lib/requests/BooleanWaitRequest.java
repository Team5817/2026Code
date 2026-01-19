package com.team5817.lib.requests;

import java.util.function.BooleanSupplier;

public class BooleanWaitRequest extends Request {
    boolean target;
    BooleanSupplier booleanSupplier;
    public BooleanWaitRequest(BooleanSupplier booleanSupplier, boolean target){
        this.booleanSupplier = booleanSupplier;
        this.target = target;
    }

      @Override
      public void act() {
      }

      @Override
      public boolean isFinished() {
        return target ? booleanSupplier.getAsBoolean() : !booleanSupplier.getAsBoolean();
      }
  }