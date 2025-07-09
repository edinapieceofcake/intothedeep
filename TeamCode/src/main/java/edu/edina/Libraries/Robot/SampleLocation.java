package edu.edina.Libraries.Robot;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

public class SampleLocation {
    private final double relativeScreenLocation;

    public SampleLocation(double relativeScreenLocation) {
        this.relativeScreenLocation = relativeScreenLocation;
    }

    public double getRelativeScreenLocation() {
        return relativeScreenLocation;
    }

    @SuppressLint("DefaultLocale")
    @Override
    @NonNull
    public String toString() {
        return String.format("sample @ %f", relativeScreenLocation);
    }
}
