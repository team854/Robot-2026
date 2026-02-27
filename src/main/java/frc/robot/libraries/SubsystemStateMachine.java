package frc.robot.libraries;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemStateMachine<E extends Enum<E>> extends SubsystemBase {

    protected E currentState;
    protected E desiredState;

    protected final Timer stateTimer = new Timer();

    public SubsystemStateMachine(E startingState) {
        this.currentState = startingState;
        this.desiredState = startingState;

        this.stateTimer.restart();
    }

    public E getCurrentState() {
        return currentState;
    }

    public void setDesiredState(E newState) {
        this.desiredState = newState;
    }

    public E getDesiredState() {
        return this.desiredState;
    }

    public double getStateTimer() {
        return this.stateTimer.get();
    }

    public void restartStateTimer() {
        this.stateTimer.restart();
    }

    /**
     * Transitions the current state to the desired state
     **/
    protected void transitionTo() {
        transitionTo(this.desiredState);
    }

    /**
     * Transitions the current state to the specified state
     * 
     * @param newState The state to transition too
     **/
    protected void transitionTo(E newState) {
        if (newState != currentState) {
            this.currentState = newState;
            
            this.stateTimer.restart();
        }
    }
}
