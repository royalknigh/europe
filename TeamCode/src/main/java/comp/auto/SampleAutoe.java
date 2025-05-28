package comp.auto;


import com.pedropathing.localization.Pose;
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

@Autonomous(name = "Sample eAuto", group = "Auto")
public class SampleAutoe extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Pose startPose = new Pose(7,112, Math.toRadians(-90));
    private Pose scorePose = new Pose(14,130, Math.toRadians(-45));

    private Pose firstSample = new Pose(20, 121, Math.toRadians(0));
    private Pose secondSample = new Pose(14, 130, Math.toRadians(5));
    private Pose thirdSample = new Pose(20, 130, Math.toRadians(25));

    //configs
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;

    private PIDFController outPID, intPID;

    public static double oP = 0.007, oI = 0, oD = 0, oF = 0.1;
    public static double iP = 0.01, iI = 0, iD = 0, iF = 0;

    public double outTargetPosition = OutConst.slidesDown;
    public double intTargetPosition = IntConst.slideRetracted;

    private boolean isOutSlideDown, isIntSlideDown;

    private PathChain preloadScore, firstScore, secondScore, thirdScore;

    public void buildPaths() {
        preloadScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.9, this::scoreSample)
                .setPathEndTimeoutConstraint(500)

                .addPath(new BezierLine(new Point(scorePose), new Point(firstSample)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstSample.getHeading())
                .addParametricCallback(0.2, this::resetExtend)
                .setPathEndTimeoutConstraint(500)
                .build();

        firstScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSample), new Point(scorePose)))
                .setLinearHeadingInterpolation(firstSample.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.9, this::scoreSample)
                .setPathEndTimeoutConstraint(500)

                .addPath(new BezierLine(new Point(scorePose), new Point(secondSample)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondSample.getHeading())
                .addParametricCallback(0.2, this::resetExtend)
                .setPathEndTimeoutConstraint(500)
                .build();
    }

    private boolean closedIntake = false;

    public ElapsedTime transferTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloadScore);
                setPathState(1);
                intakeTimer.reset();
                break;
            case 1:
                if(!follower.isBusy()){
                    if(!closedIntake) {
                        intTargetPosition = IntConst.slideExtended;
                        servoConfig.setIntakePos(IntConst.rot_GRAB,IntConst.y_GRAB,IntConst.clawRot_INIT,IntConst.claw_OPEN);
                        if(intakeTimer.milliseconds() > 500) {
                            servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                            intakeTimer.reset();
                            closedIntake = true;
                        }
                    }
                        if (closedIntake) {
                            outTargetPosition = OutConst.slideTransfer;

                            if (intakeTimer.milliseconds() > 50) servoConfig.intY.setPosition(IntConst.y_INIT);
                            if (intakeTimer.milliseconds() > 100) {
                                servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                                servoConfig.intRot.setPosition(IntConst.rot_INIT);
                            }
                            if (intakeTimer.milliseconds() > 200 && intakeTimer.milliseconds() < 300) {
                                transferTimer.reset();
                                intTargetPosition = IntConst.slideRetracted;
                                servoConfig.setOuttakePos(OutConst.lr_TRANSFER, OutConst.y_TRANSFER, OutConst.link_INIT, OutConst.claw_OPEN);
                            }

                            if (transferTimer.milliseconds() > 300)
                                servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
                            if (transferTimer.milliseconds() > 500) {
                                servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                            }
                        }

                    setPathState(2);
                }
                break;
            case 2:
                scoreSample();
                setPathState(3);
                break;
            case 3:

                break;
        }
    }


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
        telemetry.addData("heading", follower.getPose().getHeading());
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

        servoConfig.setInitPos();

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
    
    public void scoreSample(){
        outTargetPosition = OutConst.slidesSampleHigh;
        if(motorConfig.frontSlideMotor.getCurrentPosition()>800){
            servoConfig.outLeft.setPosition(OutConst.lr_SAMPLE);
            servoConfig.outRight.setPosition(OutConst.lr_SAMPLE);
            servoConfig.outY.setPosition(OutConst.y_SAMPLE);
            servoConfig.outLink.setPosition(OutConst.link_PLACE);
        }

    }
    
    public void resetExtend() {
        outTargetPosition = OutConst.slideTransfer;
        servoConfig.setOuttakePos(OutConst.lr_TRANSFER, OutConst.y_TRANSFER, OutConst.link_INIT, OutConst.claw_OPEN);
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
}