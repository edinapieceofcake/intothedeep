package edu.edina.Libraries.Robot;

import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

/*
 * Copyright (c) 2024 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * An {@link org.firstinspires.ftc.vision.opencv.ColorRange represents a 3-channel minimum/maximum
 * range for a given color space}
 */
public class ColorRangeTwo
{
    protected final ColorSpace colorSpace;
    protected final Scalar min;
    protected final Scalar max;

    // -----------------------------------------------------------------------------
    // DEFAULT OPTIONS
    // -----------------------------------------------------------------------------

    public static final ColorRangeTwo BLUE = new ColorRangeTwo(
            ColorSpace.YCrCb,
            new Scalar( 16,   0, 155),
            new Scalar(255, 127, 255)
    );

    public static final ColorRangeTwo RED = new ColorRangeTwo(
            ColorSpace.YCrCb,
            new Scalar( 32, 176,  0),
            new Scalar(255, 255, 132)
    );

    public static final ColorRangeTwo YELLOW = new ColorRangeTwo(
            ColorSpace.YCrCb,
            new Scalar( 32, 128,   0),
            new Scalar(255, 170, 120)
    );

    public static final ColorRangeTwo GREEN = new ColorRangeTwo(
            ColorSpace.YCrCb,
            new Scalar( 32,   0,   0),
            new Scalar(255, 120, 133)
    );

    // -----------------------------------------------------------------------------
    // ROLL YOUR OWN
    // -----------------------------------------------------------------------------

    public ColorRangeTwo(ColorSpace colorSpace, Scalar min, Scalar max)
    {
        this.colorSpace = colorSpace;
        this.min = min;
        this.max = max;
    }
}
