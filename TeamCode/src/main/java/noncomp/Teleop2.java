package noncomp;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;

@TeleOp(name = "\uD83E\uDDDC Teleop2", group = "TeleOp")
public class Teleop2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    boolean specimen = true;
    boolean reset;

    //outPID and intPID
    public static double oP = 0.0075, oI = 0, oD = 0, oF = 0.007;
    public static double iP = 0.003, iI = 0, iD = 0, iF = 0;

    public static int outTargetPosition = OutConst.slidesDown;
    public static int intTargetPosition = OutConst.slidesDown;

    private PIDFController outPID, intPID;

    private int fraction;
    private boolean isOutSlideDown, isIntSlideDown;

    //configs
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;

    private enum RobotStates {
        INIT,
        GRAB,
        TRANSFER,
        RETRACT_SAMPLE,
        RETRACT_SPECIMEN,
        INTERMEDIATE,
        SAMPLE_HIGH,
        SAMPLE_LOW,
        SPECIMEN_PICKUP,
        PICKUP,
        SPECIMEN_PLACE
    }

    private RobotStates state = RobotStates.INIT;


    @Override
    public void runOpMode() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        fraction = 1;

        if(gamepad1.right_bumper){
            specimen = true;
        }
        if(gamepad1.left_bumper){
            specimen = false;
        }

        updatePIDFController();

        servoConfig.setInitPos();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            movement();
            robotState();
            setOutPID();
            setIntPID();
            updatePIDFController();
            panicButton();
            lowerLegs();

            telemetry.addData("state ", state);
            telemetry.addData("intake motor position", motorConfig.intakeMotor.getCurrentPosition());
            telemetry.addData("slide motor position", motorConfig.frontSlideMotor.getCurrentPosition());
            telemetry.addData("servo angle", angle);
            telemetry.update();
        }
    }

    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime transferTimer = new ElapsedTime();
    public ElapsedTime placeTimer = new ElapsedTime();
    public ElapsedTime pickupTimer = new ElapsedTime();
    public ElapsedTime grabTimer = new ElapsedTime();

    private boolean outClosed = false;

    public double angle = IntConst.clawRot_INIT;

    public void robotState() {
        switch (state) {
            case INIT: {
                intTargetPosition = IntConst.slideRetracted;
                servoConfig.setInitPos();
                if(motorConfig.intakeMotor.getCurrentPosition()>10)
                    motorConfig.intakeMotor.setPower(-1);
                if(motorConfig.intakeMotor.getCurrentPosition()<20 && motorConfig.intakeMotor.getVelocity() < 0.05){
                    motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    motorConfig.intakeMotor.setPower(0);
                }
                outTargetPosition = OutConst.slidesDown;
                fraction = 1;
                if (gamepad1.a) {
                    state = RobotStates.GRAB;
                    angle = IntConst.clawRot_INIT;
                    grabTimer.reset();
                }
                if(gamepad1.right_bumper){
                    specimen = true;
                }
                if(gamepad1.left_bumper){
                    specimen = false;
                }
                if (gamepad2.left_bumper) {
                    servoConfig.setOuttakePos(OutConst.lr_PICK, OutConst.y_PICK, OutConst.link_INIT, OutConst.claw_OPEN);
                    servoConfig.intY.setPosition(IntConst.y_INIT);
                    outTargetPosition = OutConst.slidesDown;
                    pickupTimer.reset();
                    outClosed = true;                               //gotta work on it and test
                    state = RobotStates.SPECIMEN_PICKUP;
                }
                break;
            }
            case GRAB: {
                fraction = 2;
                intTargetPosition = IntConst.slideExtended;
                outTargetPosition=OutConst.slideTransfer;

                servoConfig.intRot.setPosition(IntConst.rot_GRAB);
                servoConfig.setOuttakePos(OutConst.lr_TRANSFER, OutConst.y_TRANSFER, OutConst.link_INIT, OutConst.claw_OPEN);

                if (grabTimer.milliseconds() > 400) {
                    servoConfig.intY.setPosition(IntConst.y_HOVER);
                    servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                }
                if(gamepad1.left_stick_x<-0.1)
                    angle-= 0.03;
                if(gamepad1.left_stick_x>0.1)
                    angle+= 0.03;
                if(gamepad1.right_bumper){
                    specimen = true;
                }
                if(gamepad1.left_bumper){
                    specimen = false;
                }
                if (gamepad1.right_trigger > 0) {
                    intakeTimer.reset();
                    servoConfig.intY.setPosition(IntConst.y_GRAB);
                    state = RobotStates.PICKUP;
                }
                servoConfig.intClawRot.setPosition(MathFunctions.clamp(angle, 0, 1));

                break;
            }
            case PICKUP:{
                if(intakeTimer.milliseconds()>100)
                    servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                if(intakeTimer.milliseconds()>150)
                    servoConfig.intY.setPosition(IntConst.y_HOVER);
                if(gamepad1.right_bumper){
                    specimen = true;
                }
                if(gamepad1.left_bumper){
                    specimen = false;
                }
                if(intakeTimer.milliseconds()>250){
                    if(specimen){
                        state = RobotStates.RETRACT_SPECIMEN;
                        reset = true;
                    }
                    if(!specimen){
                        state = RobotStates.RETRACT_SAMPLE;
                        reset = true;
                    }
                }
                if(gamepad1.left_stick_x<-0.1)
                    angle-= 0.03;
                if(gamepad1.left_stick_x>0.1)
                    angle+= 0.03;
                servoConfig.intClawRot.setPosition(MathFunctions.clamp(angle, 0, 1));


                break;
            }


            case RETRACT_SAMPLE: {
                fraction = 1;
                if(reset){
                    intakeTimer.reset();
                    reset = false;
                }

                if (intakeTimer.milliseconds() > 50)
                    servoConfig.intY.setPosition(IntConst.y_TRANSFER);
                if (intakeTimer.milliseconds() > 100) {
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                    servoConfig.intRot.setPosition(IntConst.rot_INIT);
                }
                if (intakeTimer.milliseconds() > 600) {
                    transferTimer.reset();
                    intTargetPosition = IntConst.slideRetracted;
                }

                if (motorConfig.intakeMotor.getCurrentPosition() < 250) {
                    transferTimer.reset();
                    state = RobotStates.TRANSFER;
                }

                if (gamepad1.a) {
                    state = RobotStates.GRAB;
                    angle = IntConst.clawRot_INIT;
                    grabTimer.reset();
                }
                break;
            }
            case RETRACT_SPECIMEN: {
                fraction = 1;
                if(reset){
                    intakeTimer.reset();
                    reset = false;
                }
                intTargetPosition = 0;
                if (intakeTimer.milliseconds() > 0)
                    servoConfig.intY.setPosition(IntConst.y_DROP);
                if (intakeTimer.milliseconds() > 100) {
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_90);
                    servoConfig.intRot.setPosition(IntConst.rot_DROP);
                }
                if (motorConfig.intakeMotor.getCurrentPosition() < 100) {
                    if (gamepad1.left_bumper) {
                        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                        servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                        servoConfig.intY.setPosition(IntConst.y_TRANSFER);
                        state = RobotStates.INTERMEDIATE;
                    }
                }

                if (gamepad1.a) {
                    state = RobotStates.GRAB;
                    angle = IntConst.clawRot_INIT;
                    grabTimer.reset();
                }
                break;
            }
            case TRANSFER: {
                //TODO: REDO TRANSFER
                if (transferTimer.milliseconds() > 700) servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
                if (transferTimer.milliseconds() > 900){
                    servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                    servoConfig.intY.setPosition(IntConst.y_INIT);
                }
                if (transferTimer.milliseconds() > 1000) {
                    servoConfig.setOuttakePos(OutConst.lr_SAMPLE, OutConst.y_SAMPLE, OutConst.link_INIT, OutConst.claw_PRESSED);
                    state = RobotStates.INTERMEDIATE;
                }
                break;
            }
            case INTERMEDIATE: {
                if (-gamepad1.left_stick_y > 0.3) {
                    placeTimer.reset();
                    servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);
                    state = RobotStates.SAMPLE_HIGH;
                }
                if (-gamepad1.left_stick_y < -0.3) {
                    placeTimer.reset();
                    servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);
                    state = RobotStates.SAMPLE_LOW;
                }
                if (gamepad2.left_bumper) {
                    servoConfig.setOuttakePos(OutConst.lr_PICK, OutConst.y_PICK, OutConst.link_INIT, OutConst.claw_OPEN);
                    outClosed = true;
                    servoConfig.intY.setPosition(IntConst.y_INIT);
                    outTargetPosition = OutConst.slidesDown;
                    state = RobotStates.SPECIMEN_PICKUP;
                }
                if (gamepad1.a) {
                    state = RobotStates.GRAB;
                    angle = IntConst.clawRot_INIT;
                    grabTimer.reset();
                }
                break;
            }
            case SAMPLE_HIGH: {
                outTargetPosition = OutConst.slidesSampleHigh;
                servoConfig.outLink.setPosition(OutConst.link_PLACE);

                if (gamepad1.left_trigger > 0) {
                    servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
                }
                break;
            }
            case SAMPLE_LOW: {
                outTargetPosition = OutConst.slidesSampleLow;
                servoConfig.outLink.setPosition(OutConst.link_PLACE);

                if (gamepad1.left_trigger > 0) {
                    servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
                }
                break;
            }
            case SPECIMEN_PICKUP: {
                if (outClosed) {
                    if (gamepad2.left_trigger > 0) {
                        servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
                        pickupTimer.reset();
                        outClosed = false;
                    }
                }
                if (pickupTimer.milliseconds() > 100 && !outClosed) {
                    servoConfig.setOuttakePos(OutConst.lr_SPEC, OutConst.y_SPEC, OutConst.link_PLACE, OutConst.claw_CLOSED);
                    outTargetPosition = OutConst.slideSpecimen;
                    if (gamepad2.right_trigger > 0) {
                        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
                        servoConfig.outLink.setPosition(OutConst.link_INIT);
                        state = RobotStates.INTERMEDIATE;
                    }
                }
                break;
            }
        }
    }

    public void movement() {
        double y = -gamepad2.left_stick_y;
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorConfig.setMotorPowers(frontLeftPower / fraction, backLeftPower / fraction, frontRightPower / fraction, backRightPower / fraction);


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
            motorConfig.backSlideMotor.setPower(-0.9 );

        } else {
            motorConfig.frontSlideMotor.setPower(outtakePower);
            motorConfig.backSlideMotor.setPower(outtakePower);
        }
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

    public void panicButton() {
        if (gamepad2.dpad_down || gamepad1.dpad_down)
            state = RobotStates.INIT;
    }

    public void lowerLegs() {
        if (gamepad2.dpad_left) {
            servoConfig.ptoLeft.setPosition(IntConst.ptoLegsDown);
            servoConfig.ptoRight.setPosition(IntConst.ptoLegsDown);
            servoConfig.ptoRot.setPosition(IntConst.ptoLock);
        }
        if (gamepad2.dpad_right) {
            servoConfig.ptoLeft.setPosition(IntConst.ptoLegsUp);
            servoConfig.ptoRight.setPosition(IntConst.ptoLegsUp);
            servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
        }

    }

}

