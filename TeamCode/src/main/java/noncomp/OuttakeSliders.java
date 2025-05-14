package noncomp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;

import configs.MotorConfig;

@Config 
@TeleOp(name = "Outtake PIDF", group = "NonComp")
public class OuttakeSliders extends OpMode {

    private FtcDashboard dashboard;
    private MotorConfig motorConfig;
    private PIDFController pidfController;

    private boolean areSlidesDown = true;
    
    private static final int MIN_POSITION = 0;     
    private static final int MAX_POSITION = 3990;  
    private static final int TOLERANCE = 10;       


    
    public static double P = 0.01;
    public static double I = 0.0001;
    public static double D = 0.0003;
    public static double F = 0.008;
    public static double K =0;

    public static double targetPosition = 0;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);

        MotorConfigurationType motorConfigurationTypeUpMotor = MotorConfig.frontSlideMotor.getMotorType().clone();
        motorConfigurationTypeUpMotor.setAchieveableMaxRPMFraction(1.0);
        MotorConfig.frontSlideMotor.setMotorType(motorConfigurationTypeUpMotor);

        MotorConfigurationType motorConfigurationTypeDownMotor = MotorConfig.backSlideMotor.getMotorType().clone();
        motorConfigurationTypeDownMotor.setAchieveableMaxRPMFraction(1.0);
        MotorConfig.backSlideMotor.setMotorType(motorConfigurationTypeDownMotor);

        updatePIDFController();

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();

        telemetry.addLine("Outtake Sliders Initialized with PIDF and Joystick Control");
    }

    @Override
    public void loop() {

        targetPosition = Range.clip(targetPosition, MIN_POSITION, MAX_POSITION);
        pidfController.setTargetPosition(targetPosition);
        pidfController.updatePosition(motorConfig.frontSlideMotor.getCurrentPosition());
        double powerLeft = pidfController.runPIDF() + K;

        if (Math.abs(motorConfig.frontSlideMotor.getCurrentPosition() - targetPosition) <= TOLERANCE) {
            powerLeft = 0;
        }
        if(motorConfig.frontSlideMotor.getCurrentPosition()>10)
            areSlidesDown = false;
        if(motorConfig.frontSlideMotor.getVelocity()<0.1 && motorConfig.frontSlideMotor.getCurrentPosition()<10){
            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            areSlidesDown = true;
        }
        else {
            motorConfig.frontSlideMotor.setPower(powerLeft);
            motorConfig.backSlideMotor.setPower(powerLeft);
        }

        if(-gamepad1.left_stick_y > 0.1) {
            targetPosition +=40;
        }
        else if(-gamepad1.left_stick_y < -0.1) {
            targetPosition -=40;
        }

        telemetry.addData("velocity", MotorConfig.frontSlideMotor.getVelocity());
        telemetry.addData("areSlidesDown", areSlidesDown);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", motorConfig.frontSlideMotor.getCurrentPosition());


        telemetry.addData("Left Motor Error", motorConfig.frontSlideMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", motorConfig.backSlideMotor.getCurrentPosition() - targetPosition);

        telemetry.addData("Right Motor Power", motorConfig.backSlideMotor.getPower());
        telemetry.addData("Left Motor Power", motorConfig.frontSlideMotor.getPower());
        telemetry.update();
    }

    private void updatePIDFController() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(P, I, D, F);
        pidfController = new PIDFController(coefficients);
    }
}
