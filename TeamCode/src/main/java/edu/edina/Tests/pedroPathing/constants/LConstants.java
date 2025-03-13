package edu.edina.Tests.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001989436789;
        ThreeWheelConstants.strafeTicksToInches = .001989436789;
        ThreeWheelConstants.turnTicksToInches = .001989436789;
        ThreeWheelConstants.leftY = 6.953125;
        ThreeWheelConstants.rightY = -6.953125;
        ThreeWheelConstants.strafeX = -6.46875;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "left_front_drive";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "right_back_drive";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "right_front_drive";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




