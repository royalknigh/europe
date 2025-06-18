package comp.auto;


import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.*;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "SampleAuto", group = ".Comp")
public class SampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Pose startPose = new Pose(7, 112.5, Math.toRadians(270));

    private Pose scorePose = new Pose(12, 127, Math.toRadians(325));

    private Pose scorePose1 = new Pose(12, 128, Math.toRadians(310));

    private Pose scorePose2 = new Pose(12, 129, Math.toRadians(330));


    private Pose firstSample = new Pose(12, 127.1, Math.toRadians(-6));

    private Pose secondSample = new Pose(12, 127.1, Math.toRadians(12));

    private Pose thirdSample = new Pose(15, 132, Math.toRadians(20));

    private Pose sub = new Pose(60, 100, Math.toRadians(-75));

    private Pose subControl = new Pose(60, 125, Math.toRadians(-75));


    //configs
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;

    private PIDFController outPID, intPID;

    public static double oP = 0.008, oI = 0, oD = 0, oF = 0.007;
    public static double iP = 0.005, iI = 0, iD = 0, iF = 0;

    public double outTargetPosition = OutConst.slidesDown;
    public double intTargetPosition = IntConst.slideRetracted;

    private boolean isOutSlideDown, isIntSlideDown;

    private PathChain preloadScore, firstExtend, firstScore,secondExtend, secondScore, thirdExtend, thirdScore, subPath;

    public void buildPaths() {
        preloadScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

        firstExtend = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(firstSample)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstSample.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

        firstScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSample), new Point(scorePose1)))
                .setLinearHeadingInterpolation(firstSample.getHeading(), scorePose1.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

        secondExtend = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(secondSample)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), secondSample.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

        secondScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSample), new Point(scorePose1)))
                .setLinearHeadingInterpolation(secondSample.getHeading(), scorePose1.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

        thirdExtend = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(thirdSample)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), thirdSample.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

        thirdScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSample), new Point(scorePose2)))
                .setLinearHeadingInterpolation(thirdSample.getHeading(), scorePose2.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

        subPath = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose2), new Point(subControl), new Point(sub)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), sub.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)
                .build();

    }

    private boolean closedIntake = false;
    private boolean dropped= false;
    private boolean score = false;


    //  public ElapsedTime transferTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime resetTimer = new ElapsedTime();

    public int check = 500;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloadScore);
                outTargetPosition = OutConst.slidesSampleHigh;
                scoreSample();
                servoConfig.outY.setPosition(OutConst.y_PICK);
                setPathState(1);
                dropped = false;
                resetTimer.reset();
                follower.setMaxPower(0.7);
                break;
            case 1:
                if(!follower.isBusy()){
                    if(!dropped) {
                        if(resetTimer.milliseconds() > 500){
                            drop();
                            resetTimer.reset();
                        }
                    }
                    else{
                        if(resetTimer.milliseconds()>500) resetExtend();
                        if(resetTimer.milliseconds()>800) {
                            outTargetPosition  = OutConst.slideTransfer;
                            follower.followPath(firstExtend);
                            score = false;
                            closedIntake = false;
                            intakeTimer.reset();
                            setPathState(2);
                        }
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    if(!score)
                        grab();
                    if(score) {
                        scoreSample();
                        follower.followPath(firstScore);
                        dropped  =false;
                        resetTimer.reset();
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy() ){
                    if(!dropped && resetTimer.milliseconds()>1000 ){
                        drop();
                        resetTimer.reset();
                    }
                    if(dropped){
                        if(resetTimer.milliseconds()>check) resetExtend();
                        if(resetTimer.milliseconds()>check +400) {
                            outTargetPosition  = OutConst.slideTransfer;
                            follower.followPath(secondExtend);
                            score = false;
                            closedIntake = false;
                            intakeTimer.reset();
                            setPathState(4);
                        }
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    if(!score)
                        grab();
                    if(score) {
                        scoreSample();
                        follower.followPath(secondScore);
                        dropped  =false;
                        resetTimer.reset();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy() ){
                    if(!dropped && resetTimer.milliseconds()>1000 ){
                        drop();
                        resetTimer.reset();
                    }
                    if(dropped){
                        if(resetTimer.milliseconds()>check) resetExtend();
                        if(resetTimer.milliseconds()>check +400) {
                            outTargetPosition  = OutConst.slideTransfer;
                            follower.followPath(thirdExtend);
                            score = false;
                            closedIntake = false;
                            intakeTimer.reset();
                            setPathState(6);
                        }
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    if(!score)
                        grab();
                    if(score) {
                        scoreSample();
                        follower.followPath(thirdScore);
                        dropped  =false;
                        resetTimer.reset();
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy() ){
                    if(!dropped && resetTimer.milliseconds()>1200 ){
                        drop();
                        resetTimer.reset();
                    }
                    if(dropped){
                        if(resetTimer.milliseconds()>check) resetExtend();
                        if(resetTimer.milliseconds()>check +400) {
                            outTargetPosition  = OutConst.slideTransfer;
                            score = false;
                            closedIntake = false;
                            intakeTimer.reset();
                            setPathState(8);
                        }
                    }
                }
                break;
            /*case 1:
                if (!follower.isBusy()) grab();
                follower.followPath(firstScore);
                setPathState(2);
                intakeTimer.reset();
                closedIntake = false;
                break;
            case 2:
                if (!follower.isBusy()) grab();
                follower.followPath(secondScore);
                setPathState(3);
                intakeTimer.reset();
                closedIntake = false;
                break;
            case 3:
                if (!follower.isBusy()) grab();
                follower.followPath(thirdScore);
                setPathState(4);
                intakeTimer.reset();
                closedIntake = false;
                break;*/
        }
    }

    public int one =0;
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        updatePIDFController();

        setIntPID();
        setOutPID();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("intake motor position", motorConfig.intakeMotor.getCurrentPosition());
        telemetry.addData("slide motor position", motorConfig.frontSlideMotor.getCurrentPosition());
        telemetry.addData("reset", resetTimer);
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
    }

    @Override
    public void init_loop() {

    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {

    }


    public void drop(){
        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
        dropped = true;
    }

    public void scoreSample() {
        outTargetPosition = OutConst.slidesSampleHigh;
            servoConfig.outLeft.setPosition(OutConst.lr_SAMPLE_AUTO);
            servoConfig.outRight.setPosition(OutConst.lr_SAMPLE_AUTO);
            servoConfig.outY.setPosition(OutConst.y_SAMPLE);
            servoConfig.outLink.setPosition(OutConst.link_PLACE);

    }

    public void resetExtend() {
        servoConfig.setOuttakePos(OutConst.lr_TRANSFER, OutConst.y_TRANSFER, OutConst.link_INIT, OutConst.claw_OPEN);
    }

    public void grab() {
        if (!closedIntake) {
            intTargetPosition = IntConst.slideExtended;
            servoConfig.intRot.setPosition(IntConst.rot_GRAB);
            servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
            servoConfig.intClaw.setPosition(IntConst.claw_OPEN);

            if(intakeTimer.milliseconds()>750)
                servoConfig.intY.setPosition(IntConst.y_GRAB);
            if (intakeTimer.milliseconds() > 1250) {
                servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                intakeTimer.reset();
                closedIntake = true;
            }
        }
        if (closedIntake) {
            if (intakeTimer.milliseconds() > 50)
                servoConfig.intY.setPosition(IntConst.y_TRANSFER);
            if (intakeTimer.milliseconds() > 100) {
                servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                servoConfig.intRot.setPosition(IntConst.rot_INIT);
            }
            if (intakeTimer.milliseconds() > 600) {
                intTargetPosition = IntConst.slideRetracted;
            }

            if (intakeTimer.milliseconds() > 1000)
                servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
            if (intakeTimer.milliseconds() > 1200) {
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
        servoConfig.intY.setPosition(IntConst.y_INIT);
        servoConfig.intRot.setPosition(IntConst.rot_INIT);
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

}