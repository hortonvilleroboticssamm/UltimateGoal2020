package org.firstinspires.ftc.teamcode;

import hortonvillerobotics.State;

public class StateMachine {
    private int current_number = 0;
    public int state_in_progress = 1;

    boolean next_state_to_execute() {
        return  ++current_number == state_in_progress;
    }

    public void runStates(State...states){
        current_number = 0;
        for(State s : states) if(next_state_to_execute()) s.run();
    }

    public void incrementState(){
        state_in_progress++;
    }

    public void reset(){
        state_in_progress = 1;
    }

    public void initializeMachine(){
        current_number = 0;
    }

    public void setState_in_progress(int state_in_progress){
        this.state_in_progress = state_in_progress;
    }
}
