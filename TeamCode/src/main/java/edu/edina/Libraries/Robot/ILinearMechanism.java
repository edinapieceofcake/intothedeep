package edu.edina.Libraries.Robot;

public interface ILinearMechanism{
    void setPower(double power);
    double getPosition(boolean raw);
}