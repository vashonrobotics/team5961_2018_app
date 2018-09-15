package org.firstinspires.ftc.teamcode;

public class Idea<Type> {
    public Type value;
    public Type trajectoryDirection;
    public Idea(Type value, Type trajectoryDirection){
        this.value = value;
        this.trajectoryDirection = trajectoryDirection;
    }
}
