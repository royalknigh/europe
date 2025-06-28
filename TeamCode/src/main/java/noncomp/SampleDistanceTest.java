package noncomp;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;

@TeleOp(name = "Limelight Sample Detection")
public class SampleDistanceTest extends LinearOpMode {

    private Limelight3A limelight;
    ServoConfig servoConfig;
    MotorConfig motorConfig;

    private static final double CAMERA_HEIGHT_IN = 8; // inches
    private static final double CAMERA_TILT_DEG = 42.5;  // degrees
    private static final double TURRET_LENGTH = 5.5;
    private static final double MAX_SLIDER_INCHES = 19.0;
    private static final int MAX_SLIDER_TICKS = 650;
    private static final double SERVO_MIN = 0.05;
    private static final double SERVO_MAX = 0.5;
    private static final double MAX_XOFFSET = 4.4;
    private static final double MIN_XOFFSET = -4.4;
    private static final double TICKS_PER_INCH = MAX_SLIDER_TICKS / MAX_SLIDER_INCHES;
    private double sliderTargetInches = 0;
    private boolean pressed = false;

    // PIDF
    public static double iP = 0.005, iI = 0, iD = 0, iF = 0;
    public  int intTargetPosition = OutConst.slidesDown;
    private boolean isIntSlideDown;
    private PIDFController intPID;

    // New variables for tracking control
    private boolean isTracking = false;
    private int trackingTargetTicks = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);
        limelight.start();

        servoConfig = new ServoConfig(hardwareMap);
        motorConfig = new MotorConfig(hardwareMap);

        servoConfig.intY.setPosition(IntConst.y_MIDDLE);
        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
        updatePIDFController();

        waitForStart();

        while (opModeIsActive()) {
            updatePIDFController();
            setIntPID();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
                LLResultTypes.DetectorResult target = result.getDetectorResults().get(0);

                double txDeg = target.getTargetXDegrees();
                double tyDeg = target.getTargetYDegrees();

                double totalVertRad = Math.toRadians(CAMERA_TILT_DEG + tyDeg);
                double txRad = Math.toRadians(txDeg);

                double yInches = CAMERA_HEIGHT_IN * Math.tan(totalVertRad);
                double xInches = yInches * Math.tan(txRad);
                yInches += 2.7;
                boolean aspectRatio = getAspectRatio(target);
//                double xCorrection = 0.0;
//                if (xInches < -0.5) {
//                    if (aspectRatio) xCorrection = 1.75;
//                    else xCorrection = 0.75;
//                }
//                xInches -= xCorrection;

                // On button press, update the tracking target and turret position
                if (gamepad1.a && !pressed) {
                    if(aspectRatio)
                        servoConfig.intClawRot.setPosition(IntConst.clawRot_90);
                    else servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                    pressed = true;

                    sliderTargetInches = yInches - TURRET_LENGTH;
                    trackingTargetTicks = (int) (sliderTargetInches * TICKS_PER_INCH);
                    trackingTargetTicks = Math.max(0, Math.min(MAX_SLIDER_TICKS, trackingTargetTicks));
                    intTargetPosition = trackingTargetTicks;
                    double turretServo = map(xInches, MIN_XOFFSET, MAX_XOFFSET, SERVO_MIN, SERVO_MAX);
                    turretServo = Math.max(SERVO_MIN, Math.min(SERVO_MAX, turretServo));
                    servoConfig.intRot.setPosition(turretServo);
// start PID control to this target
                } else if (!gamepad1.a) {
                    pressed = false;
                }

                // Run PID control continuously when tracking

                telemetry.addData("pressed", pressed);
                telemetry.addData("trackingTargetTicks", trackingTargetTicks);
                telemetry.addData("intakeMotorPos", motorConfig.intakeMotor.getCurrentPosition());
                telemetry.addData("tx (deg)", "%.2f", txDeg);
                telemetry.addData("ty (deg)", "%.2f", tyDeg);
                telemetry.addData("Y Distance (in)", "%.2f", yInches);
                telemetry.addData("X Distance (in)", "%.2f", xInches);
                telemetry.addData("AspectRatio Horizontal?", aspectRatio);

            } else {
                telemetry.addLine("No valid target detected");
            }
            telemetry.update();
        }

        // stop intake motor on exit
        motorConfig.intakeMotor.setPower(0);
        limelight.stop();
    }

    public void updatePIDFController() {
        CustomPIDFCoefficients intCoefficients = new CustomPIDFCoefficients(iP, iI, iD, iF);
        intPID = new PIDFController(intCoefficients);
    }
    public void setIntPID() {
        intPID.setTargetPosition(intTargetPosition);
        intPID.updatePosition(motorConfig.intakeMotor.getCurrentPosition());

        double intakePower = intPID.runPIDF();

        if (Math.abs(motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition) < 10)
            intakePower = 0;

        if (motorConfig.intakeMotor.getCurrentPosition() > 10) isIntSlideDown = false;

        if (motorConfig.intakeMotor.getCurrentPosition() < 20 && motorConfig.intakeMotor.getVelocity() < 0.05 && intTargetPosition == IntConst.slideRetracted) {
            motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            isIntSlideDown = true;
        }
        if (intTargetPosition == IntConst.slideRetracted && isIntSlideDown)
            motorConfig.intakeMotor.setPower(0);

        if (intTargetPosition == IntConst.slideRetracted && motorConfig.intakeMotor.getCurrentPosition() > 10)
            motorConfig.intakeMotor.setPower(-1);

        else motorConfig.intakeMotor.setPower(intakePower);

        motorConfig.intakeMotor.setPower(intakePower);
    }
    private boolean getAspectRatio(LLResultTypes.DetectorResult target) {
        List<List<Double>> corners = target.getTargetCorners();
            double x0 = corners.get(0).get(0);
            double y0 = corners.get(0).get(1);
            double x1 = corners.get(1).get(0);
            double y1 = corners.get(1).get(1);
            double x2 = corners.get(2).get(0);
            double y2 = corners.get(2).get(1);
            double x3 = corners.get(3).get(0);
            double y3 = corners.get(3).get(1);

            // Calculate width as distance between corner 0 and corner 1
            double width = Math.hypot(x1 - x0, y1 - y0);
            // Calculate height as distance between corner 1 and corner 2
            double height = Math.hypot(x2 - x1, y2 - y1);

            telemetry.addData("Width", width);
            telemetry.addData("Height", height);

//            double ratio = width / height;
            return width>height;

    }

    private double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (val - inMin) * (outMax - outMin) / (inMax - inMin);
    }
}