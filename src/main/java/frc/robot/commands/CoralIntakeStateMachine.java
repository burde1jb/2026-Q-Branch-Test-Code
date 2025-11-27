package frc.robot.commands;

import frc.robot.subsystems.CoralIntakeSubsystem;

public class CoralIntakeStateMachine {
    public enum IntakeState {
        Unloaded,
        Intaking,
        Loaded,
        Ejecting,
    }

    private IntakeState currentState;
    private CoralIntakeSubsystem subsystem;

    // Use hash?
    // Unloaded -> Intaking
    // Intaking -> Unloaded, *Loaded*
    // Loaded -> Ejecting
    // Ejecting -> Loaded, *Unloaded*
    public CoralIntakeStateMachine(CoralIntakeSubsystem subsystem) {
        this.currentState = IntakeState.Unloaded;
        this.subsystem = subsystem;
        subsystem.limitSwitchOff(true);
    }

    public void requestStateTransition(IntakeState requestedState) {
        if (transitionValid(requestedState)) {
            this.currentState = requestedState;
            if (requestedState == IntakeState.Loaded) {
                subsystem.limitSwitchOff(false);
            } 
            if (requestedState == IntakeState.Unloaded) {
                subsystem.limitSwitchOff(true);
            }
        }
    }

    public IntakeState getCurrentState() {
        return this.currentState;
    }

    private boolean transitionValid(IntakeState requestedState) {
        if (currentState == requestedState) {
            return true;
        }
        switch (currentState) {
            case Unloaded:
                return requestedState == IntakeState.Intaking;
            case Intaking:
                return requestedState == IntakeState.Loaded || requestedState == IntakeState.Unloaded;
            case Loaded:
                return requestedState == IntakeState.Ejecting;
            case Ejecting:
                return requestedState == IntakeState.Loaded ||  requestedState == IntakeState.Unloaded;
            default:
                return false;
        }
    }
}
