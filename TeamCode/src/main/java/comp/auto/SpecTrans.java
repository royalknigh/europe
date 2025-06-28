package comp.auto;

import com.google.gson.annotations.JsonAdapter;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "SpecTrans", group = ".Comp")
public class SpecTrans extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public ServoConfig servoConfig;
    public MotorConfig motorConfig;

    private PIDFController outPID, intPID;

    public static double oP = 0.01, oI = 0, oD = 0, oF = 0.03;
    public static double iP = 0.005, iI = 0, iD = 0, iF = 0;

    public double outTargetPosition = OutConst.slidesDown;
    public double intTargetPosition = IntConst.slideRetracted;

    private static final double CAMERA_HEIGHT_IN = 8; // inches
    private static final double CAMERA_TILT_DEG = 41;  // degrees
    private static final double TURRET_LENGTH = 5.5;
    private static final double MAX_SLIDER_INCHES = 19.0;
    private static final int MAX_SLIDER_TICKS = 650;
    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.45;
    private static final double MAX_XOFFSET = 3.6;//4.4
    private static final double MIN_XOFFSET = -3.6;
    private static final double TICKS_PER_INCH = MAX_SLIDER_TICKS / MAX_SLIDER_INCHES;
    private double sliderTargetInches = 0;
    private boolean pressed = false;
    private int selectMode = 0;
    private boolean detected = false;
    private int trackingTargetTicks = 0;
    private boolean startSearchTimer = true;
    private boolean leftPressed = false;
    private boolean readyToGrab = false;
    private boolean score = false;
    private boolean grabbed = false;

    private ElapsedTime searchTimer = new ElapsedTime();
    private ElapsedTime grabTimerLL = new ElapsedTime();

    private boolean isOutSlideDown, isIntSlideDown;

    private Limelight3A limelight;
    private PathChain preload, secondPreload, scorePickup, thirdSamp;
    private Pose firstSample = new Pose(22, 18, 0);
    private Pose secondSample = new Pose(22, 10, 0);
    private Pose firstPick= new Pose(10, 25, 0);

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PoseSpec.startPose), new Point(PoseSpec.scorePose)))
                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(3.0)
//                .addParametricCallback(0, () -> follower.setMaxPower(0.8))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.5))
                .addParametricCallback(0, () -> place())
                .addParametricCallback(0.95, () -> servoConfig.outClaw.setPosition(OutConst.claw_OPEN))
//                .addParametricCallback(0.8, () -> follower.setMaxPower(0.2))

//                .setPathEndTimeoutConstraint(0)
                .build();

        secondPreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(PoseSpec.scorePose), new Point(PoseSpec.pickupPoseControl1), new Point(PoseSpec.pickupPose)))
                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, ()-> outtakeReset())
                .addParametricCallback(0.6, ()-> servoConfig.intRot.setPosition(IntConst.rot_DROP))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.4))
                .addParametricCallback(0.95, ()-> servoConfig.intClaw.setPosition(IntConst.claw_OPEN))
//                .addParametricCallback(0, () -> pickup())
//                .addParametricCallback(0.5, () -> follower.setMaxPower(0.3))
//                .addParametricCallback(0.95, () -> servoConfig.outClaw.setPosition(OutConst.claw_CLOSED))
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PoseSpec.pickupPose), new Point(PoseSpec.scorePose2)))
                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(3)
//                .setPathEndTimeoutConstraint(0)
//                .addParametricCallback(0.1, () -> place())

                .addPath(new BezierLine(new Point(PoseSpec.scorePose2), new Point(firstSample)))

                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(3)
//                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0., () -> follower.setMaxPower(0.6))



//                .setPathEndTimeoutConstraint(0)

                /*.addPath(new BezierCurve(new Point(PoseSpec.firstPush), new Point(PoseSpec.secondSampleControl), new Point(PoseSpec.secondSample)))
                .setConstantHeadingInterpolation(0)
//                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(PoseSpec.secondSample),new Point(PoseSpec.secondPush)))
                .setConstantHeadingInterpolation(0)
//                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierCurve(
                        new Point(PoseSpec.secondPush), new Point(PoseSpec.thirdSampleControl), new Point(PoseSpec.thirdSample)))
                .setConstantHeadingInterpolation(0)
//                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(PoseSpec.thirdSample),new Point(PoseSpec.thirdPush)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)
*/
                .build();

                thirdSamp = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(firstSample), new Point(secondSample)))
                        .setConstantHeadingInterpolation(0)
                        .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload);
                setPathState(1);
                break;
            case 1:
            {
                if(!follower.isBusy()){
                    if(startSearchTimer){
                        startSearchTimer = false;
                        searchTimer.reset();
                    }
                    if(searchTimer.milliseconds()>500 && !detected)
                        detectAndTrackYellowSample();
                    if (motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition < 20 && detected && !readyToGrab && !grabbed) {
                        readyToGrab = true;
                        grabbed = true;
                        grabTimerLL.reset();
                    }
                    if (readyToGrab) {
                        if (grabTimerLL.milliseconds() > 700)
                            servoConfig.intY.setPosition(IntConst.y_GRAB);
                        if (grabTimerLL.milliseconds() > 1000) {
                            servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                            readyToGrab = false;
                        }
                    }
                    if (!readyToGrab && !score && detected) {
                        if (grabTimerLL.milliseconds() > 1200) {
                            servoConfig.intY.setPosition(IntConst.y_DROP);
                        }
                        if (grabTimerLL.milliseconds() > 1300) {
                            servoConfig.intRot.setPosition(0.1);
                            intTargetPosition = IntConst.slideRetracted;
                            follower.setMaxPower(0.8);
                            follower.followPath(secondPreload);
                            setPathState(2);
                        }
                    }
                }
                break;
            }
            case 2:
                if(!follower.isBusy()){

                }
                break;
            case 3:
                if(!follower.isBusy()){

                    follower.followPath(scorePickup);
                    setPathState(3);
                }
            case 4:
                if(!follower.isBusy())
                {
                    intTargetPosition = IntConst.slideExtended;
                    servoConfig.intY.setPosition(IntConst.y_HOVER);
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        updatePIDFController();
        setIntPID();
        setOutPID();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("T Value", follower.getCurrentTValue());
        telemetry.addData("Out position", motorConfig.frontSlideMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);
        limelight.start();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(PoseSpec.startPose);
        initialise();
        updatePIDFController();

        buildPaths();
    }

    @Override
    public void init_loop(){
        if(gamepad1.dpad_left && !leftPressed){
            selectMode = (selectMode+1)%2;
            leftPressed = true;
        }else if(!gamepad1.dpad_left) {
            leftPressed = false;
        }

        String modeName;
        if(selectMode == 1)
            modeName = "Red";
        else modeName = "Blue";
        telemetry.addData("Mode: ",modeName);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
    private void detectAndTrackYellowSample() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            LLResultTypes.DetectorResult sampleTarget = null;

            // Find the first yellow target
            for (LLResultTypes.DetectorResult target : result.getDetectorResults()) {
                String detectedClass = target.getClassName();
                boolean valid = (selectMode == 0 && detectedClass.contains("blue")) ||
                        (selectMode == 1 && detectedClass.contains("red"));
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
            yInches += 3.7;

            boolean horizontal = isHorizontal(sampleTarget, yInches);

            double turretServo = map(xInches, MIN_XOFFSET, MAX_XOFFSET, SERVO_MIN, SERVO_MAX);
            turretServo = Math.max(SERVO_MIN, Math.min(SERVO_MAX, turretServo));
            servoConfig.intRot.setPosition(turretServo);
            detected = true;
            if (horizontal)
                servoConfig.intClawRot.setPosition(map(turretServo,SERVO_MIN,SERVO_MAX,0.44,0.1));
            else
                servoConfig.intClawRot.setPosition(map(turretServo,SERVO_MIN,SERVO_MAX,0.81,0.48));

            pressed = true;

            double turretAngleRad = Math.toRadians(map(turretServo,SERVO_MIN,SERVO_MAX,-39,39));
            double verticalCompensation = Math.abs(Math.sin(turretAngleRad)) * 2;
            sliderTargetInches = (yInches - TURRET_LENGTH)+ verticalCompensation;
            trackingTargetTicks = (int) (sliderTargetInches * TICKS_PER_INCH);
            trackingTargetTicks = Math.max(0, Math.min(MAX_SLIDER_TICKS, trackingTargetTicks));
            intTargetPosition = trackingTargetTicks;


            telemetry.addData("pressed", pressed);
            telemetry.addData("trackingTargetTicks", trackingTargetTicks);
            telemetry.addData("intakeMotorPos", motorConfig.intakeMotor.getCurrentPosition());
            telemetry.addData("tx (deg)", "%.2f", txDeg);
            telemetry.addData("ty (deg)", "%.2f", tyDeg);
            telemetry.addData("Y Distance (in)", "%.2f", yInches);
            telemetry.addData("X Distance (in)", "%.2f", xInches);
            telemetry.addData("isHorizontal", horizontal);
            telemetry.addData("Detected Class", "yellow");
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

            // Tuned exponential ratio threshold
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
    public void extend(){
        servoConfig.setIntakePos(IntConst.rot_GRAB, IntConst.y_GRAB, IntConst.clawRot_INIT, IntConst.claw_OPEN);
    }

    public void pickup(){
        servoConfig.setOuttakePos(OutConst.lr_PICK, OutConst.y_PICK, OutConst.link_INIT, OutConst.claw_OPEN);
        outTargetPosition = OutConst.slidesDown;
    }

    public void place(){
        servoConfig.setOuttakePos(OutConst.lr_SPEC, OutConst.y_SPEC, OutConst.link_INIT, OutConst.claw_CLOSED);
        outTargetPosition = OutConst.slideSpecimen;
    }

    public void initialise(){
        servoConfig.intY.setPosition(IntConst.y_MIDDLE);
        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
        servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);

        servoConfig.outRight.setPosition(OutConst.lr_INIT);
        servoConfig.outLeft.setPosition(OutConst.lr_INIT);

        servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);
        servoConfig.outLink.setPosition(OutConst.link_INIT);
        servoConfig.outY.setPosition(OutConst.y_INIT);


        servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
        servoConfig.ptoRight.setPosition(IntConst.ptoLegsUp);
        servoConfig.ptoLeft.setPosition(IntConst.ptoLegsUp);
    }

    public void updatePIDFController() {
        CustomPIDFCoefficients outCoefficients = new CustomPIDFCoefficients(oP, oI, oD, oF);
        CustomPIDFCoefficients intCoefficients = new CustomPIDFCoefficients(iP, iI, iD, iF);
        outPID = new PIDFController(outCoefficients);
        intPID = new PIDFController(intCoefficients);
    }

    public void setOutPID() {
        outPID.setTargetPosition(outTargetPosition);
        outPID.updatePosition(motorConfig.frontSlideMotor.getCurrentPosition());

        double outtakePower = outPID.runPIDF();

        if (Math.abs(motorConfig.frontSlideMotor.getCurrentPosition() - outTargetPosition) < 10) {
            outtakePower = 0;
        }
        if (motorConfig.frontSlideMotor.getCurrentPosition() > 10) isOutSlideDown = false;

        if (motorConfig.frontSlideMotor.getCurrentPosition() < 60 && motorConfig.frontSlideMotor.getVelocity() < 0.05 && outTargetPosition == OutConst.slidesDown) {
            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            isOutSlideDown = true;
        }

        if (outTargetPosition == OutConst.slidesDown && isOutSlideDown) {
            motorConfig.frontSlideMotor.setPower(0);
            motorConfig.backSlideMotor.setPower(0);
        }

        if (outTargetPosition == OutConst.slidesDown && motorConfig.frontSlideMotor.getCurrentPosition() > 10) {
            motorConfig.frontSlideMotor.setPower(-1);
            motorConfig.backSlideMotor.setPower(-1);

        } else {
            motorConfig.frontSlideMotor.setPower(outtakePower);
            motorConfig.backSlideMotor.setPower(outtakePower);
        }

        motorConfig.frontSlideMotor.setPower(outtakePower);
        motorConfig.backSlideMotor.setPower(outtakePower);
    }

    public void setIntPID() {
        intPID.setTargetPosition(intTargetPosition);
        intPID.updatePosition(motorConfig.intakeMotor.getCurrentPosition());

        double intakePower = intPID.runPIDF();

        if (Math.abs(motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition) < 10)
            intakePower = 0;

        if (motorConfig.intakeMotor.getCurrentPosition() > 10) isIntSlideDown = false;

        if (motorConfig.intakeMotor.getCurrentPosition() < 10 && motorConfig.intakeMotor.getVelocity() < 0.05 && intTargetPosition == IntConst.slideRetracted) {
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
    public void outtakeReset(){
        outTargetPosition = OutConst.slidesDown;

        servoConfig.outLeft.setPosition(OutConst.lr_PICK);
        servoConfig.outRight.setPosition(OutConst.lr_PICK);
        servoConfig.outY.setPosition(OutConst.y_PICK);
        servoConfig.outLink.setPosition(OutConst.link_INIT);
        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);

    }
}