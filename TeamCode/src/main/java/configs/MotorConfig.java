package configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MotorConfig {
    
    public static DcMotorEx frontLeftMotor;
    public static DcMotorEx backLeftMotor;
    public static DcMotorEx frontRightMotor;
    public static DcMotorEx backRightMotor;

    public static DcMotorEx frontSlideMotor;
    public static DcMotorEx backSlideMotor;

    public static DcMotorEx intakeMotor;

    public MotorConfig(HardwareMap hardwareMap) {
        frontLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backRightMotor");

        frontSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontSlideMotor");
        backSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "backSlideMotor");

        intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakeMotor");

        //drivetrain
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //slides
        frontSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //intake
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




        MotorConfigurationType configFrontSlideMotor = frontSlideMotor.getMotorType().clone();
        configFrontSlideMotor.setAchieveableMaxRPMFraction(1.0);
        frontSlideMotor.setMotorType(configFrontSlideMotor);

        MotorConfigurationType configBackSlideMotor = backSlideMotor.getMotorType().clone();
        configBackSlideMotor.setAchieveableMaxRPMFraction(1.0);
        backSlideMotor.setMotorType(configBackSlideMotor);

        MotorConfigurationType configFrontLeftMotor = frontLeftMotor.getMotorType().clone();
        configFrontLeftMotor.setAchieveableMaxRPMFraction(1.0);
        frontLeftMotor.setMotorType(configFrontLeftMotor);

        MotorConfigurationType configBackLeftMotor = backLeftMotor.getMotorType().clone();
        configBackLeftMotor.setAchieveableMaxRPMFraction(1.0);
        backLeftMotor.setMotorType(configBackLeftMotor);

        MotorConfigurationType configFrontRightMotor = frontRightMotor.getMotorType().clone();
        configFrontRightMotor.setAchieveableMaxRPMFraction(1.0);
        frontRightMotor.setMotorType(configFrontRightMotor);

        MotorConfigurationType configBackRightMotor = backRightMotor.getMotorType().clone();
        configBackRightMotor.setAchieveableMaxRPMFraction(1.0);
        backRightMotor.setMotorType(configBackRightMotor);

        MotorConfigurationType configIntakeMotor = intakeMotor.getMotorType().clone();
        configIntakeMotor.setAchieveableMaxRPMFraction(1.0);
        intakeMotor.setMotorType(configIntakeMotor);
    }

    public void setMotorPowers(double frontLeftPower, double backLeftPower, double frontRightPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

}
