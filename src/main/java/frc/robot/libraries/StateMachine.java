package frc.robot.libraries;

import edu.wpi.first.wpilibj.Timer;

public class StateMachine<E extends Enum<E>> {
    private E currentState;
    private E desiredState;

    private E defaultState;

    private int pendingPriority = Integer.MIN_VALUE;
    private E pendingState;

    private final Timer stateTimer = new Timer();

    /**
     * 
     * @param startingState The state to start the statemachine in
     * @param defaultState The state to transition to if no state requests have been made in a tick. If null then it will stay on the last desired state.
     */
    public StateMachine(E startingState, E defaultState) {
        this.currentState = startingState;
        this.desiredState = startingState;

        this.defaultState = defaultState;

        if (this.defaultState != null) {
            this.pendingState = this.defaultState;
        } else {
            this.pendingState = startingState;
        }

        this.stateTimer.restart();
    }

    public E getCurrentState() {
        return currentState;
    }

    /**
     * Requests the state machine transitions to a new desired state
     * 
     * @param newState The new state to transfer to
     * @param priority An int representing the priority of the state change. Priorities should generally follow this format:
     * <ul>
     *      <li>0: Idle state</li>
     *      <li>1-10: Normal commands</li>
     *      <li>11-20: Manual overrides</li>
     *      <li>21-30: Safety overrides</li>
     * </ul>
     */
    public void requestDesiredState(E newState, int priority) {
        if (newState == null) {
            return;
        }

        if (priority >= pendingPriority) {
            this.pendingPriority = priority;
            this.pendingState = newState;
        }
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
    public void transitionTo() {
        transitionTo(getDesiredState());
    }

    /**
     * Transitions the current state to the specified state
     * 
     * @param newState The state to transition too
     **/
    public void transitionTo(E newState) {
        if (newState != currentState && newState != null) {
            this.currentState = newState;
            
            this.stateTimer.restart();
        }
    }

    /**
     * Must be called for the requested state to be applied
     **/
    public void updateDesiredState() {
        if (pendingState != null) {
            this.desiredState = this.pendingState;
        }
        this.pendingPriority = Integer.MIN_VALUE;
        this.pendingState = this.defaultState;
    }
}
