package edu.edina.Libraries.LinearMotion;

public interface ILinearMechanism {
    void setPower(double power);

    double getPosition(boolean raw);

    LinearMechanismSettings getSettings();
}

