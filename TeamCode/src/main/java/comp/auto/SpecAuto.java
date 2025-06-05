package comp.auto;

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

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "SpecAuto", group = ".Comp")
public class SpecAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public ServoConfig servoConfig;
    public MotorConfig motorConfig;

    private PIDFController outPID, intPID;

    public static double oP = 0.007, oI = 0, oD = 0, oF = 0.1;
    public static double iP = 0.01, iI = 0, iD = 0, iF = 0;

    public double outTargetPosition = OutConst.slidesDown;
    public double intTargetPosition = IntConst.slideRetracted;

    private boolean isOutSlideDown, isIntSlideDown;


    private PathChain preload, secondPreload, scorePickup;

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PoseSpec.startPose), new Point(PoseSpec.scorePose)))
                .setConstantHeadingInterpolation(0)

                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)

                .addParametricCallback(0.05, () -> place())
                .addParametricCallback(0.95, () -> servoConfig.outClaw.setPosition(OutConst.claw_OPEN))
                .build();

                secondPreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PoseSpec.scorePose), new Point(PoseSpec.pickupPose)))
                .setConstantHeadingInterpolation(0)

                .setZeroPowerAccelerationMultiplier(2)
                .setPathEndTimeoutConstraint(0)

                .addParametricCallback(0.3, () -> pickup())
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(PoseSpec.pickupPose), new Point(PoseSpec.scorePose)))
                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0.3, () -> place())

                .addPath(new BezierCurve(
                        new Point(PoseSpec.scorePose), new Point(PoseSpec.firstPushControl1),
                        new Point(PoseSpec.firstPushControl2), new Point(PoseSpec.firstSample)))
                .setConstantHeadingInterpolation(0)
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndTimeoutConstraint(0)
                .addParametricCallback(0., () -> pickup())


                .addPath(new BezierLine(new Point(PoseSpec.firstSample), new Point(PoseSpec.firstPush)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierCurve(new Point(PoseSpec.firstPush), new Point(PoseSpec.secondSampleControl), new Point(PoseSpec.secondSample)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierLine(new Point(PoseSpec.secondSample),new Point(PoseSpec.secondPush)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierCurve(
                        new Point(PoseSpec.secondPush), new Point(PoseSpec.thirdSampleControl), new Point(PoseSpec.thirdSample)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)

                .addPath(new BezierCurve(new Point(PoseSpec.thirdSample),new Point(PoseSpec.thirdPushControl), new Point(PoseSpec.thirdPush)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0)

                //need to start cylcing
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preload);
                follower.setMaxPower(0.4);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(secondPreload);
                    follower.setMaxPower(0.7);
                    setPathState(2);
                }
                break;
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
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }

    public void extend(){
        //TODO : ADD PIDF FOR INTAKE AND OUTTAKE THEN SET POSITIONS
        servoConfig.setIntakePos(IntConst.rot_GRAB, IntConst.y_GRAB, IntConst.clawRot_INIT, IntConst.claw_OPEN);
    }

    public void pickup(){
         servoConfig.setOuttakePos(OutConst.lr_PICK, OutConst.y_PICK, OutConst.link_INIT, OutConst.claw_OPEN);
         outTargetPosition = OutConst.slidesDown;
    }

    public void place(){
        servoConfig.setOuttakePos(OutConst.lr_SPEC, OutConst.y_SPEC, OutConst.link_PLACE, OutConst.claw_CLOSED);
        outTargetPosition = OutConst.slideSpecimen;
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