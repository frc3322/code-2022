// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

class State {
  private BooleanSupplier condition;
  private double ledPattern;

  private State(BooleanSupplier condition, double ledPattern) {
    this.condition = condition;
    this.ledPattern = ledPattern;
  }

  public static State defaultState(double pattern) {
    return new State(() -> true, pattern);
  }

  public static State conditionalState(double pattern) {
    return new State(() -> false, pattern);
  }

  public void setCondition(BooleanSupplier condition) {
    this.condition = condition;
  }

  public boolean conditionMet() {
    return condition.getAsBoolean();
  }

  public double getPattern() {
    return ledPattern;
  }
}

public class LEDs extends SubsystemBase {
  public enum Modes {
    BASE,
    ANGLE_GOOD,
    DIST_GOOD,
    ANGLE_AND_DIST,
    TRANSFER_FULL
  }

  // Color constants here: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  private final double SOLIDRED = 0.61;
  private final double SOLIDBLUE = 0.85;
  private final double SOLIDGREEN = 0.71;
  private final double RAINBOW = -0.99;

  private static final LEDs instance = new LEDs();
  private final Spark blinkin = new Spark(0);
  // First state listed is default, last state listed is highest priority
  private Modes[] priority =
      new Modes[] {
        Modes.BASE, Modes.ANGLE_GOOD, Modes.TRANSFER_FULL, Modes.DIST_GOOD, Modes.ANGLE_AND_DIST
      };

  private HashMap<Modes, State> stateMap = new HashMap<>();

  /** Creates a new LEDs. */
  public LEDs() {
    stateMap.put(Modes.BASE, State.defaultState(SOLIDBLUE));
    stateMap.put(Modes.ANGLE_GOOD, State.conditionalState(RAINBOW));
    stateMap.put(Modes.TRANSFER_FULL, State.conditionalState(SOLIDGREEN));
    stateMap.put(Modes.DIST_GOOD, State.conditionalState(SOLIDBLUE));
    stateMap.put(Modes.ANGLE_AND_DIST, State.conditionalState(RAINBOW));
  }

  public void setCondition(Modes mode, BooleanSupplier condition) {
    stateMap.get(mode).setCondition(condition);
  }

  @Override
  public void periodic() {
    double output = 0;
    for (Modes mode : priority) {
      State state = stateMap.get(mode);
      if (state.conditionMet()) {
        output = state.getPattern();
      }
    }
    blinkin.set(output);
  }

  public static LEDs get() {
    return instance;
  }
}
