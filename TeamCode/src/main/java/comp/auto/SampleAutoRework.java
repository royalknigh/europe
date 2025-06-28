package comp.auto;


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

@Autonomous(name = "SampleAuto", group = ".Comp")
public class SampleAutoRework extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean slides = true;
    private boolean ok1 = true;
    private boolean ok2 = true;
    private boolean ok3 = true;
    private boolean outtakeResetted = false;
    private boolean detected = false;
    private boolean da = false;
    private int nr = 0;

    private Pose startPose = new Pose(7, 112.5, Math.toRadians(270));

    private Pose firstSample = new Pose(15.5, 132, Math.toRadians(339));

    private Pose secondSample = new Pose(16, 134.5, Math.toRadians(354));

    private Pose thirdSample = new Pose(16, 135.5, Math.toRadians(3));

    private Pose sub = new Pose(60, 95, Math.toRadians(-90));

    private Pose sub2 = new Pose(65, 96, Math.toRadians(-90));

    private Pose subControl = new Pose(60, 125, Math.toRadians(-75));

    private Pose dropPose = new Pose(16.5, 132, Math.toRadians(320));


    //configs
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;

    private PIDFController outPID, intPID;

    public static double oP = 0.008, oI = 0.0013, oD = 0.000005, oF = 0.03;
    public static double iP = 0.005, iI = 0, iD = 0, iF = 0;

    public double outTargetPosition = OutConst.slidesDown;
    public double intTargetPosition = IntConst.slideRetracted;

    private boolean isOutSlideDown, isIntSlideDown;


    // vision
    private Limelight3A limelight;

    private static final double CAMERA_HEIGHT_IN = 8; // inches
    private static final double CAMERA_TILT_DEG = 41;  // degrees
    private static final double TURRET_LENGTH = 5.5;
    private static final double MAX_SLIDER_INCHES = 19.0;
    private static final int MAX_SLIDER_TICKS = 650;
    private static final double SERVO_MIN = 0.1;
    private static final double SERVO_MAX = 0.45;
    private static final double MAX_XOFFSET = 2.5;//4.4
    private static final double MIN_XOFFSET = -2.5;
    private static final double TICKS_PER_INCH = MAX_SLIDER_TICKS / MAX_SLIDER_INCHES;
    private double sliderTargetInches = 0;
    private boolean pressed = false;
    private boolean buttonPressed = false;
    private boolean readyToGrab = false;
    private boolean startSearchTimer = true;
    private boolean dontSearchAnymore = false;
    private boolean canExtend = true;
    private boolean transfer = false;
    private boolean leftPressed = false;
    private boolean followPath = true;
    private int trackingTargetTicks = 0;
    private int selectMode = 0;

    private ElapsedTime grabTimer = new ElapsedTime();
    private ElapsedTime startSearching = new ElapsedTime();




    private boolean closedIntake = false;
    private boolean dropped= false;
    private boolean score = false;
    private boolean grabTimerReset = true;


    //  public ElapsedTime transferTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime resetTimer = new ElapsedTime();
    public ElapsedTime grabTimerLL = new ElapsedTime();
    public ElapsedTime dropTimer = new ElapsedTime();

    private PathChain preloadScore, secondExtend, initPath,
            thirdExtend, subPath, dropPath, subPath2, dropPath2;



    public int check = 500;

    public void buildPaths() {
        preloadScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(firstSample)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstSample.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTimeoutConstraint(0)

                .addParametricCallback(0, () -> follower.setMaxPower(0.7))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.5))
                .build();

        secondExtend = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSample), new Point(secondSample)))
                .setLinearHeadingInterpolation(firstSample.getHeading(), secondSample.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, () -> scoreSample())
                .addParametricCallback(0, () -> follower.setMaxPower(0.7))
                .addParametricCallback(0.8, () -> follower.setMaxPower(0.5))

                .build();

        thirdExtend = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSample), new Point(thirdSample)))
                .setLinearHeadingInterpolation(secondSample.getHeading(), thirdSample.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0, () -> scoreSample())
                .addParametricCallback(0, () -> follower.setMaxPower(0.7))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.5))

                .build();

        subPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdSample), new Point(subControl), new Point(sub)))
                .setLinearHeadingInterpolation(thirdSample.getHeading(), sub.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)


                .addParametricCallback(0.05, ()-> outtakeReset())
                .addParametricCallback(0.6, ()-> follower.setMaxPower(0.6))
                .addParametricCallback(0.8, ()-> follower.setMaxPower(0.5))
                .addParametricCallback(0.95, ()-> follower.setMaxPower(0.3))
                .build();

        subPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropPose), new Point(subControl), new Point(sub2)))
                .setLinearHeadingInterpolation(dropPose.getHeading(), sub2.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)

                .addParametricCallback(0, () -> servoConfig.intRot.setPosition(IntConst.rot_GRAB))
                .addParametricCallback(0.05, ()-> outtakeReset())
                .addParametricCallback(0.6, ()-> follower.setMaxPower(0.6))
                .addParametricCallback(0.7, ()-> follower.setMaxPower(0.4))
                .addParametricCallback(0.9, ()-> follower.setMaxPower(0.2))
                .build();

        dropPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub), new Point(subControl), new Point(dropPose)))
                .setLinearHeadingInterpolation(sub.getHeading(), dropPose.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTimeoutConstraint(0)

                .addParametricCallback(0.7, ()-> follower.setMaxPower(0.7))
                .addParametricCallback(0.9, ()-> follower.setMaxPower(0.4))
                .build();

        dropPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub2), new Point(subControl), new Point(dropPose)))
                .setLinearHeadingInterpolation(sub2.getHeading(), dropPose.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTimeoutConstraint(0)

                .addParametricCallback(0.7, ()-> follower.setMaxPower(0.7))
                .addParametricCallback(0.9, ()-> follower.setMaxPower(0.4))
                .build();
        initPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(dropPose), new Point(subControl), new Point(sub2)))
                .setLinearHeadingInterpolation(dropPose.getHeading(), sub2.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTimeoutConstraint(0)

                .addParametricCallback(0, () -> servoConfig.intRot.setPosition(IntConst.rot_GRAB))
                .addParametricCallback(0.02, ()-> initReset())
                .addParametricCallback(0, () -> follower.setMaxPower(0.9))
                .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                .addParametricCallback(0.4, () -> follower.setMaxPower(0.4))
                .addParametricCallback(0.2, () -> follower.setMaxPower(0.2))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloadScore);
                dropped = false;
                intakeTimer.reset();
                resetTimer.reset();
                setPathState(1);
                break;
            case 1:
                if(follower.getCurrentTValue()>0.8 && ok1) scoreSample();
                if(!follower.isBusy()){
                    if(canExtend){

                        canExtend = false;
                    }
                    if(resetTimer.milliseconds() > 2100 && ok1){
                        extend(700);
                        predrop();
                        ok1 = false;
                    }
                    if(resetTimer.milliseconds() > 2200)
                        servoConfig.outY.setPosition(0.3);
                    if(resetTimer.milliseconds() > 2400&& ok2){
                        drop();
                        ok2 = false;
                    }
                    if(resetTimer.milliseconds() > 2900 && ok3){
                        if(!outtakeResetted) {
                            resetOuttake();
                            outtakeResetted = true;
                        }
                        grabFunction(0.54,0.26,0.05,100);
                    }
                    if(score){
                        follower.followPath(secondExtend);
                        dropped = false;
                        intakeTimer.reset();
                        resetTimer.reset();
                        ok1 = true;
                        ok2 = true;
                        ok3 = true;
                        closedIntake = false;
                        outtakeResetted = false;
                        grabTimerReset = true;
                        canExtend = true;
                        score = false;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    if(canExtend){

                        canExtend = false;
                    }
                    if(resetTimer.milliseconds() > 300 && ok1){
                        extend(640);
                        predrop();
                        ok1 = false;
                    }
                    if(resetTimer.milliseconds() > 900)
                        servoConfig.outY.setPosition(0.35);
                    if(resetTimer.milliseconds() > 1100 && ok2){
                        drop();
                        ok2 = false;
                    }
                    if(resetTimer.milliseconds() > 1400 && ok3){
                        if(!outtakeResetted) {
                            resetOuttake();
                            outtakeResetted = true;
                        }
                        grabFunction(0.64,0.26,0.05,200);
                    }
                    if(score){
                        follower.followPath(thirdExtend);
                        dropped = false;
                        intakeTimer.reset();
                        resetTimer.reset();
                        ok1 = true;
                        ok2 = true;
                        ok3 = true;
                        score = false;
                        grabTimerReset = true;
                        outtakeResetted = false;
                        closedIntake = false;
                        canExtend = true;
                        boolean leftPressed = false;
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if(canExtend){

                        canExtend = false;

                    }
                    if (resetTimer.milliseconds() > 600 && ok1) {
                        extend(670);
                        predrop();
                        ok1 = false;
                    }
                    if(resetTimer.milliseconds() > 900)
                        servoConfig.outY.setPosition(0.35);
                    if (resetTimer.milliseconds() > 1200 && ok2) {
                        drop();
                        ok2 = false;
                    }
                    if(resetTimer.milliseconds() > 1700 && ok3){
                        if(!outtakeResetted) {
                            resetOuttake();
                            outtakeResetted = true;
                        }
                        grabFunction(0.87,0.07,0.13,100);
                    }
                    if(score){
                        dropped = false;
                        intakeTimer.reset();
                        resetTimer.reset();
                        ok1 = true;
                        ok2 = true;
                        ok3 = true;
                        score = false;
                        grabTimerReset = true;
                        outtakeResetted = false;
                        closedIntake = false;
                        dropped = false;
                        scoreSample();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (resetTimer.milliseconds() > 600 && ok1) {
                    predrop();
                    servoConfig.intRot.setPosition(IntConst.rot_GRAB);
                    ok1 = false;
                }
                if(resetTimer.milliseconds() > 900)
                    servoConfig.outY.setPosition(0.3);
                if (resetTimer.milliseconds() > 1200 && ok2) {
                    drop();
                    servoConfig.intY.setPosition(IntConst.y_MIDDLE);
                    ok2 = false;
                }
                if(dropped) {
                    score = false;
                    follower.followPath(subPath);
                    follower.setMaxPower(0.8);
                    transferred = false;
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy() || transfer) {
                    if (startSearchTimer) {
                        transfer = true;
                        startSearchTimer = false;
                        startSearching.reset();
                    }
                    if (!detected && startSearching.milliseconds() > 650 && !readyToGrab && !dontSearchAnymore)  {
                        detectAndTrackYellowSample();
                        dontSearchAnymore = true;
                    }
                    if (motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition < 20 && detected) {
                        da = true;
                        detected = false;
                        readyToGrab = true;
                        buttonPressed = false;
                        grabTimerLL.reset();
                    }
                    if (readyToGrab) {
                        if (grabTimerLL.milliseconds() > 500)
                            servoConfig.intY.setPosition(IntConst.y_GRAB);
                        if (grabTimerLL.milliseconds() > 1000) {
                            servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                            readyToGrab = false;
                        }
                    }
                    if (!readyToGrab && !score && da) {
                        if (grabTimerLL.milliseconds() > 1200){
                            servoConfig.intY.setPosition(IntConst.y_TRANSFER);
                            servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                            intTargetPosition = IntConst.slideRetracted + 300;

                        }
                        if(grabTimerLL.milliseconds()>1450 && followPath) {
                            followPath = false;
                            servoConfig.intRot.setPosition(IntConst.rot_INIT);
                            follower.followPath(dropPath);
                            follower.setMaxPower(0.9);
                        }
                        if (grabTimerLL.milliseconds() > 1850) {
                            intTargetPosition = IntConst.slideRetracted ;
                        }

                        if (grabTimerLL.milliseconds() > 2100)
                            servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
                        if (grabTimerLL.milliseconds() > 2250) {
                            servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                            servoConfig.intY.setPosition(IntConst.y_INIT);

                            setPathState(6);
                            score = true;
                        }
                    }
                }
                    break;
            case 6:
                if(follower.getCurrentTValue()>0.6) {
                    scoreSample();
                }
                if(follower.getCurrentTValue()>0.95) {
                    predrop();
                    dropTimer.reset();
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    servoConfig.outY.setPosition(0.3);
                    if(dropTimer.milliseconds()>600) {
                        drop();
                        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
                    }
                    if(dropTimer.milliseconds()>800){
                        servoConfig.intY.setPosition(IntConst.y_MIDDLE);
                        follower.followPath(subPath2);
                        setPathState(8);
                        transfer = false;
                        startSearchTimer = true;
                        detected = false;
                        readyToGrab = false;
                        dontSearchAnymore = false;
                        da = false;
                        score = false;
                        followPath = true;
                    }

                }
                break;
            case 8:
                if(!follower.isBusy() || transfer){
                    if (startSearchTimer) {
                        transfer = true;
                        startSearchTimer = false;
                        startSearching.reset();
                    }
                    if (!detected && startSearching.milliseconds() > 750 && !readyToGrab && !dontSearchAnymore)  {
                        detectAndTrackYellowSample();
                        dontSearchAnymore = true;
                    }
                    if (motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition < 20 && detected) {
                        da = true;
                        detected = false;
                        readyToGrab = true;
                        buttonPressed = false;
                        grabTimerLL.reset();
                    }
                    if (readyToGrab) {
                        if (grabTimerLL.milliseconds() > 500)
                            servoConfig.intY.setPosition(IntConst.y_GRAB);
                        if (grabTimerLL.milliseconds() > 1000) {
                            servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                            readyToGrab = false;
                        }
                    }
                    if (!readyToGrab && !score && da) {
                        if (grabTimerLL.milliseconds() > 1200){
                            servoConfig.intY.setPosition(IntConst.y_TRANSFER);
                            servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                            intTargetPosition = IntConst.slideRetracted + 300;

                        }
                        if(grabTimerLL.milliseconds()>1450 && followPath) {
                            followPath = true;
                            servoConfig.intRot.setPosition(IntConst.rot_INIT);
                            follower.followPath(dropPath2);
                            follower.setMaxPower(0.9);
                        }
                        if (grabTimerLL.milliseconds() > 1850) {
                            intTargetPosition = IntConst.slideRetracted ;
                        }

                        if (grabTimerLL.milliseconds() > 2100)
                            servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
                        if (grabTimerLL.milliseconds() > 2250) {
                            servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                            servoConfig.intY.setPosition(IntConst.y_INIT);

                            setPathState(9);
                            score = true;
                        }
                    }
                }
                break;
            case 9:
                if(follower.getCurrentTValue()>0.6) {
                    scoreSample();
                }
                if(follower.getCurrentTValue()>0.95) {
                    predrop();
                    dropTimer.reset();
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    servoConfig.outY.setPosition(0.3);
                    if(dropTimer.milliseconds()>600) {
                        drop();
                        setPathState(-1);
                        follower.followPath(initPath);
                    }
                }
                break;
        }
    }
    private boolean transferred = false;

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        updatePIDFController();
        if(slides) setIntPID();
        setOutPID();

        telemetry.addData("path state", pathState);
        telemetry.addData("nr", nr);
        telemetry.addData("timer", grabTimer);
        telemetry.addData("da",da);
        telemetry.addData("slider",motorConfig.intakeMotor.getCurrentPosition());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("intake motor position", motorConfig.intakeMotor.getCurrentPosition());
//        telemetry.addData("slide motor position", motorConfig.frontSlideMotor.getCurrentPosition());
//        telemetry.addData("reset", resetTimer);
        telemetry.update();
    }

    @Override
    public void init() {
        updatePIDFController();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);

        initialise();

        buildPaths();

        //vision
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);
        limelight.start();

        //TODO : STOP THE LIMELIGHT AT THE END
    }

    /////////////////////// VISION

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
            yInches += 3.7;
            if(yInches>16)
                yInches += 1;
            boolean horizontal = isHorizontal(sampleTarget, yInches);

            double turretServo = map(xInches, MIN_XOFFSET, MAX_XOFFSET, SERVO_MIN, SERVO_MAX);
            turretServo = Math.max(SERVO_MIN, Math.min(SERVO_MAX, turretServo));
            servoConfig.intRot.setPosition(turretServo);
            detected = true;
            if (horizontal)
                servoConfig.intClawRot.setPosition(map(turretServo,0.1,SERVO_MAX,0.44,0.1));
            else
                servoConfig.intClawRot.setPosition(map(turretServo,0.1,SERVO_MAX,0.81,0.48));

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

    ///////////////////////////////////////



    @Override
    public void init_loop() {
        if(gamepad1.dpad_left && !leftPressed){
            selectMode = (selectMode+1)%3;
            leftPressed = true;
        }else if(!gamepad1.dpad_left) {
            leftPressed = false;
        }

        String modeName;
        if(selectMode == 1)
            modeName = "Yellow + Blue";
        else if(selectMode ==2)
            modeName = "Yellow + Red";
        else modeName = "Yellow Only";
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

    public void setIntake(){
        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
        servoConfig.intY.setPosition(IntConst.y_MIDDLE);
    }

    public void outtakeReset(){
        outTargetPosition = OutConst.slideTransfer;

        servoConfig.outLeft.setPosition(OutConst.lr_TRANSFER);
        servoConfig.outRight.setPosition(OutConst.lr_TRANSFER);
        servoConfig.outY.setPosition(OutConst.y_TRANSFER);
        servoConfig.outLink.setPosition(OutConst.link_INIT);
        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);

        follower.setMaxPower(0.9);
    }
    public void predrop(){
        servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);
        servoConfig.outLeft.setPosition(OutConst.lr_SAMPLE_AUTO);
        servoConfig.outRight.setPosition(OutConst.lr_SAMPLE_AUTO);
    }

    public void drop(){
        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
        dropped = true;
    }

    public void extend(int extension){
        intTargetPosition = extension;
        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
        servoConfig.intY.setPosition(IntConst.y_TRANSFER);
        servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
    }

    public void scoreSample() {
        outTargetPosition = OutConst.slidesSampleHigh;
        servoConfig.outLeft.setPosition(OutConst.lr_INIT_AUTO);
        servoConfig.outRight.setPosition(OutConst.lr_INIT_AUTO);
        servoConfig.outY.setPosition(OutConst.y_SAMPLE);
        servoConfig.outLink.setPosition(OutConst.link_PLACE);
    }

    public void resetOuttake(){
        outTargetPosition = OutConst.slideTransfer;
        servoConfig.outLeft.setPosition(OutConst.lr_TRANSFER);
        servoConfig.outRight.setPosition(OutConst.lr_TRANSFER);
        servoConfig.outY.setPosition(OutConst.y_TRANSFER);
        servoConfig.outLink.setPosition(OutConst.link_INIT);
        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
    }


    public void resetExtend() {
        servoConfig.setOuttakePos(OutConst.lr_TRANSFER, OutConst.y_TRANSFER, OutConst.link_INIT, OutConst.claw_OPEN);
    }


    public void grabFunction(double clawRot,double rot,double int_Y, int plusTimer){
        if(grabTimerReset) {
            grabTimerReset = false;
            grabTimer.reset();
        }
        if (!closedIntake) {
            servoConfig.intClawRot.setPosition(clawRot);
            servoConfig.intRot.setPosition(rot);
            if(grabTimer.milliseconds()>300)
                servoConfig.intY.setPosition(int_Y);
            if (grabTimer.milliseconds() > 800) {
                servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                closedIntake = true;
            }
        }
        if (closedIntake && !score) {
            if (grabTimer.milliseconds() > 1000){
                servoConfig.intY.setPosition(IntConst.y_TRANSFER);
                servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                servoConfig.intRot.setPosition(IntConst.rot_INIT);
            }
            if (grabTimer.milliseconds() > 1400+plusTimer) {
                intTargetPosition = IntConst.slideRetracted;
            }

            if (grabTimer.milliseconds() > 1900)
                servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
            if (grabTimer.milliseconds() > 2050) {
                servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                servoConfig.intY.setPosition(IntConst.y_INIT);
                score = true;
            }
        }
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
            motorConfig.frontSlideMotor.setPower(-0.9);
            motorConfig.backSlideMotor.setPower(-0.9);

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
            motorConfig.intakeMotor.setPower(-0.8);

        else motorConfig.intakeMotor.setPower(intakePower);

        motorConfig.intakeMotor.setPower(intakePower);
    }

    public void initialise(){
//        servoConfig.intY.setPosition(IntConst.y_TRANSFER);
        servoConfig.intY.setPosition(IntConst.y_MIDDLE);
        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
        servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);

//        servoConfig.outRight.setPosition(OutConst.lr_INIT);
//        servoConfig.outLeft.setPosition(OutConst.lr_INIT);

        servoConfig.outLeft.setPosition(OutConst.lr_INIT_AUTO);
        servoConfig.outRight.setPosition(OutConst.lr_INIT_AUTO);

        servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);
        servoConfig.outLink.setPosition(OutConst.link_INIT);
        servoConfig.outY.setPosition(OutConst.y_INIT);


        servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
        servoConfig.ptoRight.setPosition(IntConst.ptoLegsUp);
        servoConfig.ptoLeft.setPosition(IntConst.ptoLegsUp);
    }
    public void initReset(){
        outTargetPosition = OutConst.slidesDown;
        servoConfig.outLeft.setPosition(OutConst.lr_INIT_AUTO);
        servoConfig.outRight.setPosition(OutConst.lr_INIT_AUTO);
        servoConfig.outY.setPosition(OutConst.y_TRANSFER);
        servoConfig.outLink.setPosition(OutConst.link_INIT);
        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
    }

}