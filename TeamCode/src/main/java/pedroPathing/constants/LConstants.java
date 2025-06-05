package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.002;
        ThreeWheelConstants.strafeTicksToInches = -.002;
        ThreeWheelConstants.turnTicksToInches = 0.002;
        ThreeWheelConstants.leftY = 4.625;
        ThreeWheelConstants.rightY = -4.625;
        ThreeWheelConstants.strafeX = 5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "frontLeftMotor";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "backRightMotor";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "frontRightMotor";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




