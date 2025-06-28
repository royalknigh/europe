package noncomp;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;

@TeleOp(name = "Limelight Sample Pickup Yellow Only")
public class SampleGrabTest extends LinearOpMode {

    private Limelight3A limelight;
    ServoConfig servoConfig;
    MotorConfig motorConfig;

    private static final double CAMERA_HEIGHT_IN = 8; // inches
    private static final double CAMERA_TILT_DEG = 41;  // degrees
    private static final double TURRET_LENGTH = 5.5;
    private static final double MAX_SLIDER_INCHES = 19.0;
    private static final int MAX_SLIDER_TICKS = 650;
    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.45;
    private static final double MAX_XOFFSET = 2.5;
    private static final double MIN_XOFFSET = -2.5;
    private static final double TICKS_PER_INCH = MAX_SLIDER_TICKS / MAX_SLIDER_INCHES;
    private double sliderTargetInches = 0;
    private static int selectMode = 0;
    private boolean pressed = false;
    private boolean buttonPressed = false;
    private boolean readyToGrab = false;

    // PIDF
    public static double iP = 0.005, iI = 0, iD = 0, iF = 0;
    public int intTargetPosition = OutConst.slidesDown;
    private boolean isIntSlideDown;
    private PIDFController intPID;

    private int trackingTargetTicks = 0;

    private ElapsedTime grabTimerLL = new ElapsedTime();

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);
        limelight.start();

        servoConfig = new ServoConfig(hardwareMap);
        motorConfig = new MotorConfig(hardwareMap);

        servoConfig.intY.setPosition(IntConst.y_MIDDLE);
//        servoConfig.intY.setPosition(IntConst.y_GRAB);
        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
        updatePIDFController();

        String[] modeNames = {"Yellow Only", "Yellow + Blue", "Yellow + Red"};
        boolean leftPressed = false;
        while(!isStarted() && !isStopRequested()){
            if(gamepad1.dpad_left && !leftPressed){
                selectMode = (selectMode+1)%3;
                leftPressed = true;
            }else if(!gamepad1.dpad_left) {
                leftPressed = false;
            }
            telemetry.addData("Mode: ", modeNames[selectMode]);
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            updatePIDFController();
            setIntPID();

            detectAndTrackYellowSample();

            // Execute grab once the intake is in place
            if (readyToGrab) {
                if (grabTimerLL.milliseconds() > 500) servoConfig.intY.setPosition(IntConst.y_GRAB);
                if (grabTimerLL.milliseconds() > 1000) {
                    servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                    readyToGrab = false;
                }
            }

            telemetry.update();
        }

        motorConfig.intakeMotor.setPower(0);
        limelight.stop();
    }

    private void detectAndTrackYellowSample() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            LLResultTypes.DetectorResult sampleTarget = null;

            // Find the first yellow target
            for (LLResultTypes.DetectorResult target : result.getDetectorResults()) {
                String detectedClass = target.getClassName();

                boolean valid = detectedClass.contains("yellow") ||
                        (selectMode == 1 && detectedClass.contains("blue")) ||
                        (selectMode == 2 && detectedClass.contains("red"));
                if (valid ) {
                    sampleTarget = target;
                    break;
                }
            }

            if (sampleTarget == null) {
                telemetry.addLine("No target detected");
                return; // no yellow target found, skip rest
            }

            double txDeg = sampleTarget.getTargetXDegrees();
            double tyDeg = sampleTarget.getTargetYDegrees();

            double totalVertRad = Math.toRadians(CAMERA_TILT_DEG + tyDeg);
            double txRad = Math.toRadians(txDeg);

            double yInches = CAMERA_HEIGHT_IN * Math.tan(totalVertRad);
            double xInches = yInches * Math.tan(txRad);
            yInches += 3.7
            ;
//            if(yInches>16)
//                yInches += 0.5;
            boolean horizontal = isHorizontal(sampleTarget,yInches);

            if (gamepad1.a && !pressed) {
                double turretServo = map(xInches, MIN_XOFFSET, MAX_XOFFSET, SERVO_MIN, SERVO_MAX);
                turretServo = Math.max(SERVO_MIN, Math.min(SERVO_MAX, turretServo));
                servoConfig.intRot.setPosition(turretServo);
                buttonPressed = true;
                if (horizontal)
                    servoConfig.intClawRot.setPosition(map(turretServo,SERVO_MIN,SERVO_MAX,0.44,0.1));
                else
                    servoConfig.intClawRot.setPosition(map(turretServo,SERVO_MIN,SERVO_MAX,0.81,0.48));

                pressed = true;
                double turretAngleRad = Math.toRadians(map(turretServo,SERVO_MIN,SERVO_MAX,-39,39));
                double verticalCompensation = Math.abs(Math.sin(turretAngleRad)) * 2.0;
                sliderTargetInches = (yInches - TURRET_LENGTH)+ verticalCompensation;
                trackingTargetTicks = (int) (sliderTargetInches * TICKS_PER_INCH);
                trackingTargetTicks = Math.max(0, Math.min(MAX_SLIDER_TICKS, trackingTargetTicks));
                intTargetPosition = trackingTargetTicks;


            } else if (!gamepad1.a) {
                pressed = false;
            }

            if ((motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition) < 20 && buttonPressed) {
                readyToGrab = true;
                buttonPressed = false;
                grabTimerLL.reset();
            }

            telemetry.addData("pressed", pressed);
            telemetry.addData("trackingTargetTicks", trackingTargetTicks);
            telemetry.addData("intakeMotorPos", motorConfig.intakeMotor.getCurrentPosition());
            telemetry.addData("tx (deg)", "%.2f", txDeg);
            telemetry.addData("ty (deg)", "%.2f", tyDeg);
            telemetry.addData("Y Distance (in)", "%.2f", yInches);
            telemetry.addData("X Distance (in)", "%.2f", xInches);
            telemetry.addData("Detected Class", "yellow");
            telemetry.addData("isHorizontal", horizontal);
            telemetry.addData("Grab timer ", grabTimerLL.milliseconds());
        } else {
            telemetry.addLine("No valid target detected");
        }
    }

    private boolean isHorizontal(LLResultTypes.DetectorResult target, double yInches) {
        List<List<Double>> corners = target.getTargetCorners();
        if (corners.size() == 4) {
            double x0 = corners.get(0).get(0), y0 = corners.get(0).get(1);
            double x1 = corners.get(1).get(0), y1 = corners.get(1).get(1);
            double x2 = corners.get(2).get(0), y2 = corners.get(2).get(1);

            double width = Math.hypot(x1 - x0, y1 - y0);
            double height = Math.hypot(x2 - x1, y2 - y1);

            double ratio = width / height;
            if(yInches<10){
                return ratio>1.3;

            }
            // Tuned exponential ratio threshold
            double A = 1.3;
            double B = 0.05;
            double threshold = A + B*Math.pow((yInches-10), 1.4);

            telemetry.addData("Width", width);
            telemetry.addData("Height", height);
            telemetry.addData("Ratio", ratio);
            telemetry.addData("Threshold", threshold);

            return ratio > threshold;
        }
        return false;
    }

    private double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (val - inMin) * (outMax - outMin) / (inMax - inMin);
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

        if (motorConfig.intakeMotor.getCurrentPosition() < 20 && motorConfig.intakeMotor.getVelocity() < 0.05 &&
                intTargetPosition == IntConst.slideRetracted) {
            motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            isIntSlideDown = true;
        }

        if (intTargetPosition == IntConst.slideRetracted && isIntSlideDown)
            motorConfig.intakeMotor.setPower(0);
        else if (intTargetPosition == IntConst.slideRetracted && motorConfig.intakeMotor.getCurrentPosition() > 10)
            motorConfig.intakeMotor.setPower(-1);
        else
            motorConfig.intakeMotor.setPower(intakePower);
    }
}