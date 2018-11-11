package org.firstinspires.ftc.teamcode;

public class BlobDetectorCandidate {
    private double x;
    private double y;
    private double width;
    private double height;

    public double getHeight() {
        return height;
    }

    public double getWidth() {
        return width;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public BlobDetectorCandidate(double x, double y, double width, double height){
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }
}
