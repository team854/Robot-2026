package frc.robot.libraries;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemStateMachine<E extends Enum<E>> extends SubsystemBase {
    private final StateMachine<E> stateMachine;

    /**
     * 
     * @param startingState The state to start the statemachine in
     * @param defaultState The state to transition to if no state requests have been made in a tick. If null then it will stay on the last desired state.
     */
    public SubsystemStateMachine(E startingState, E defaultState) {
        stateMachine = new StateMachine<E>(startingState, defaultState);
    }

    public E getCurrentState() {
        return stateMachine.getCurrentState();
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
        stateMachine.requestDesiredState(newState, priority);
    }

    public E getDesiredState() {
        return stateMachine.getDesiredState();
    }

    public double getStateTimer() {
        return stateMachine.getStateTimer();
    }

    public void restartStateTimer() {
        stateMachine.restartStateTimer();
    }

    /**
     * Transitions the current state to the desired state
     **/
    protected void transitionTo() {
        stateMachine.transitionTo();
    }

    /**
     * Transitions the current state to the specified state
     * 
     * @param newState The state to transition too
     **/
    protected void transitionTo(E newState) {
        stateMachine.transitionTo(newState);
    }

    @Override
    public final void periodic() {
        statePeriodicBefore();
        stateMachine.updateDesiredState();
        statePeriodic();
    }

    protected void statePeriodicBefore() {};
    protected void statePeriodic() {};
}
