package comp.tele;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

////////// Testing FPS only
import java.util.concurrent.TimeUnit;
///////////////////////////

import Components.DistanceSensorComponent;
import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;

@TeleOp(name = "\uD83E\uDDDC Merman Solo", group = "TeleOp")
public class MermanSolo extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //outPID and intPID
    public static double oP = 0.008, oI = 0.0013, oD = 0.000005, oF = 0.03;
    public static double iP = 0.003, iI = 0, iD = 0, iF = 0;

    public static int outTargetPosition = OutConst.slidesDown;
    public static int intTargetPosition = OutConst.slidesDown;

    private PIDFController outPID, intPID;

    private int fraction;
    private boolean isOutSlideDown, isIntSlideDown;

    public ElapsedTime initTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime transferTimer = new ElapsedTime();
    public ElapsedTime placeTimer = new ElapsedTime();
    public ElapsedTime pickupTimer = new ElapsedTime();
    public ElapsedTime grabTimer = new ElapsedTime();

    private boolean outClosed = false;
    private boolean hang = false;
    private boolean slides = false;
    private boolean hanged = false;

    private boolean startTimer = true;
    private ElapsedTime loopTime = new ElapsedTime();
    private int looptime = 0;
    public int sensorDetect = 0;
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
        HANG, RETRY, SPECIMEN_PLACE
    }

    private RobotStates state = RobotStates.INIT;

    int timer = 0;

    @Override
    public void runOpMode() {

        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        motorConfig.intakeMotor.setCurrentAlert(9., CurrentUnit.AMPS);
        fraction = 1;

        int frameAcc = 0;
        long timePrev = 0;
        long timeAcc = 0;

        updatePIDFController();

        servoConfig.setInitPos();
        servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
        servoConfig.ptoLeft.setPosition(IntConst.ptoLegsUp);
        servoConfig.ptoRight.setPosition(IntConst.ptoLegsUp);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(startTimer) {
                loopTime.reset();
                startTimer = false;
            }
            looptime++;
            if(loopTime.seconds()>1){
                telemetry.addData("loops", looptime);
                telemetry.update();
                loopTime.reset();
                looptime = 0;
            }
            // TRYING STUFF OUT
            timer++;
            frameAcc += 1;
            timeAcc = runtime.time(TimeUnit.MILLISECONDS) - timePrev;
            if (timeAcc >= 1000)
            {
                //telemetry.addData("FPS", frameAcc);
                frameAcc = 0;
                timeAcc -= 1000;
            }
            ///////////////////////////////////////////
            if (!hang)
                movement();
            robotState();
            if (!slides)
                setOutPID();
            setIntPID();
            updatePIDFController();
            panicButton();
            lowerLegs();


//            telemetry.addData("state ", state);
//            telemetry.addData("intake motor position", motorConfig.intakeMotor.getCurrentPosition());
            telemetry.addData("slide motor position ", motorConfig.frontSlideMotor.getCurrentPosition());
            telemetry.addData("hanged", hanged);
//            telemetry.addData("servo angle", angle);
//            telemetry.addData("timer", timer);
//            telemetry.addData("isOverCurrent", motorConfig.backRightMotor.isOverCurrent());
            telemetry.update();
        }
    }

    public double angle = IntConst.clawRot_INIT;

    public void robotState() {
        switch (state) {
            case INIT: {
                intTargetPosition = IntConst.slideRetracted;
                hang = false;
                slides = false;
                setInitPos();
                if (motorConfig.intakeMotor.getCurrentPosition() > 5)
                    motorConfig.intakeMotor.setPower(-1);
                if (motorConfig.intakeMotor.getCurrentPosition() < 20 && motorConfig.intakeMotor.getVelocity() < 0.05) {
                    motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    motorConfig.intakeMotor.setPower(0);
                }
                if (initTimer.milliseconds() > 500)
                    outTargetPosition = OutConst.slidesDown;
                fraction = 1;
                if (gamepad2.a) {
                    state = RobotStates.GRAB;
                    angle = IntConst.clawRot_INIT;
                    grabTimer.reset();
                }
                if (gamepad2.left_bumper) {
                    servoConfig.setOuttakePos(OutConst.lr_PICK, OutConst.y_PICK, OutConst.link_INIT, OutConst.claw_OPEN);
                    servoConfig.intY.setPosition(IntConst.y_INIT);
                    outTargetPosition = OutConst.slidesDown;
                    pickupTimer.reset();
                    outClosed = true;
                    state = RobotStates.SPECIMEN_PICKUP;
                }
                break;
            }
            case GRAB: {
                fraction = 2;
                intTargetPosition = IntConst.slideExtended;
                outTargetPosition = OutConst.slideTransfer;

                servoConfig.intRot.setPosition(IntConst.rot_GRAB);

                if (grabTimer.milliseconds() > 400) {
                    servoConfig.intY.setPosition(IntConst.y_HOVER);
                    servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                }
                if (-gamepad2.right_stick_y < -0.5)
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_90);
                if (-gamepad2.right_stick_y > 0.5)
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                if (gamepad2.right_trigger > 0 && intakeTimer.milliseconds() > 200) {
                    intakeTimer.reset();
                    servoConfig.intY.setPosition(IntConst.y_GRAB);
                    state = RobotStates.PICKUP;
                }
                break;
            }
            case PICKUP: {
                if (intakeTimer.milliseconds() > 100)
                    servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                if (intakeTimer.milliseconds() > 150)
                    servoConfig.intY.setPosition(IntConst.y_HOVER);
                if (intakeTimer.milliseconds() > 250) {
                    if (gamepad2.right_trigger > 0) {
                        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                        intakeTimer.reset();
                        state = RobotStates.GRAB;
                    }
                }
                if (gamepad2.right_bumper) {
                    intakeTimer.reset();
                    state = RobotStates.RETRACT_SAMPLE;
                }
                if (gamepad2.left_trigger > 0) {
                    intakeTimer.reset();
                    state = RobotStates.RETRACT_SPECIMEN;
                }

                if (-gamepad2.right_stick_y < -0.5)
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_90);
                if (-gamepad2.right_stick_y > 0.5)
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);

                break;
            }
            case RETRACT_SAMPLE:
            {
                fraction = 1;
                servoConfig.setOuttakePos(OutConst.lr_TRANSFER, OutConst.y_TRANSFER, OutConst.link_INIT, OutConst.claw_OPEN);

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

                if (motorConfig.intakeMotor.getCurrentPosition() < 10) {
                    transferTimer.reset();
                    state = RobotStates.TRANSFER;
                }
                if (gamepad2.a) {
                    state = RobotStates.GRAB;
                    angle = IntConst.clawRot_INIT;
                    grabTimer.reset();
                }
                break;
            }
            case RETRACT_SPECIMEN: {
                fraction = 1;
                intTargetPosition = 0;
                servoConfig.setOuttakePos(OutConst.lr_PICK, OutConst.y_PICK, OutConst.link_INIT, OutConst.claw_OPEN);
                if (intakeTimer.milliseconds() > 0)
                    servoConfig.intY.setPosition(IntConst.y_DROP);
                if (intakeTimer.milliseconds() > 100) {
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_90);
                    servoConfig.intRot.setPosition(IntConst.rot_DROP);
                }
                if (motorConfig.intakeMotor.getCurrentPosition() < 100) {
                    if (gamepad2.left_trigger > 0) {
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
                if (transferTimer.milliseconds() > 100) servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
                if (transferTimer.milliseconds() > 250) servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                if (transferTimer.milliseconds() > 400) {
                    servoConfig.setOuttakePos(OutConst.lr_SAMPLE, OutConst.y_SAMPLE, OutConst.link_INIT, OutConst.claw_PRESSED);
                    outTargetPosition = OutConst.slidesSampleLow;
                }
                if (transferTimer.milliseconds() > 400) {
                    servoConfig.intRot.setPosition(IntConst.rot_GRAB);
                }
                if (transferTimer.milliseconds()>500)
                {servoConfig.intY.setPosition(IntConst.y_DROP);
                    state = RobotStates.INTERMEDIATE;}
                break;
            }
            case INTERMEDIATE: {
                if (gamepad2.right_trigger > 0) {
                    placeTimer.reset();
//                    servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);
                    state = RobotStates.SAMPLE_HIGH;
                }
                if (gamepad2.right_bumper) {
                    placeTimer.reset();
//                    servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);
                    state = RobotStates.SAMPLE_LOW;
                }
                if (gamepad2.left_bumper) {
                    servoConfig.setOuttakePos(OutConst.lr_PICK, OutConst.y_PICK, OutConst.link_INIT, OutConst.claw_OPEN);
                    outClosed = true;
                    servoConfig.intY.setPosition(IntConst.y_INIT);
                    outTargetPosition = OutConst.slidesDown;
                    state = RobotStates.SPECIMEN_PICKUP;
                    pickupTimer.reset();
                }

                if (gamepad2.a) {
                    state = RobotStates.GRAB;
                    angle = IntConst.clawRot_INIT;
                    grabTimer.reset();
                }
                break;
            }
            case SAMPLE_HIGH: {
                fraction = 2;
                outTargetPosition = OutConst.slidesSampleHigh;
                servoConfig.outLink.setPosition(OutConst.link_PLACE);
                if (motorConfig.frontSlideMotor.getCurrentPosition() > 800)
                    servoConfig.outClaw.setPosition(OutConst.claw_CLOSED);

                if (gamepad2.left_trigger > 0) {
                    servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
                    state = RobotStates.INTERMEDIATE;
                }
                if (gamepad2.right_bumper)
                    state = RobotStates.SAMPLE_LOW;
                break;
            }
            case SAMPLE_LOW: {
                fraction = 2;
                outTargetPosition = OutConst.slidesSampleLow;
                servoConfig.outLink.setPosition(OutConst.link_PLACE);

                if (gamepad2.left_trigger > 0) {
                    servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
                    state = RobotStates.INTERMEDIATE;

                }

                if (gamepad2.right_bumper)
                    state = RobotStates.SAMPLE_HIGH;
                break;
            }
            case SPECIMEN_PICKUP: {
                if (outClosed) {
                    if (gamepad2.left_trigger>0) {
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
            case HANG:
                servoConfig.intY.setPosition(IntConst.y_INIT);

                if (gamepad2.triangle) {
                    outTargetPosition = OutConst.firstBarHang;
                    servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
                    servoConfig.ptoLeft.setPosition(IntConst.ptoLegsDown);
                    servoConfig.ptoRight.setPosition(IntConst.ptoLegsDown);
                }

                if (gamepad2.a) {
                    slides = true;
                    pull = true;
                }
                if (gamepad2.square) {
                    slides = true;
                    pull = false;
                    secondHang = true;
                }

                if(secondHang){
                    servoConfig.ptoRot.setPosition(IntConst.ptoLock);
                    motorConfig.frontSlideMotor.setMotorDisable();
                    motorConfig.backSlideMotor.setMotorDisable();


                    motorConfig.backLeftMotor.setPower(1);
                    motorConfig.backRightMotor.setPower(1);
                }
                if (pull) {
                    servoConfig.ptoRot.setPosition(IntConst.ptoLock);
                    motorConfig.frontSlideMotor.setMotorDisable();
                    motorConfig.backSlideMotor.setMotorDisable();

                    motorConfig.backLeftMotor.setPower(1);
                    motorConfig.backRightMotor.setPower(1);

                    if(motorConfig.frontSlideMotor.getCurrentPosition()<5){
                        motorConfig.backLeftMotor.setMotorDisable();
                        motorConfig.backRightMotor.setMotorDisable();
                        outTargetPosition = OutConst.slidesDown;
                        hanged = true;
                        pull = false;
                        slides = false;
                        MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                        MotorConfig.frontSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        MotorConfig.backSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }
                if(hanged){
                    servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
                }
                if (gamepad2.circle) {
                    motorConfig.frontSlideMotor.setMotorEnable();
                    motorConfig.backSlideMotor.setMotorEnable();
                    outTargetPosition = OutConst.slidesSampleHigh-100;
                    servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
                    servoConfig.ptoLeft.setPosition(IntConst.ptoLegsUp);
                    servoConfig.ptoRight.setPosition(IntConst.ptoLegsUp);
                    servoConfig.intY.setPosition(IntConst.y_INIT);

                    hanged = false;
                }

                if(gamepad2.dpad_left){
                    motorConfig.backLeftMotor.setMotorDisable();
                    motorConfig.backRightMotor.setMotorDisable();
                }
                break;
        }
    }

    private boolean pull = false;
    private boolean tug = false;
    private boolean secondHang = false;

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
            motorConfig.backSlideMotor.setPower(-0.9);

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

        if (motorConfig.intakeMotor.isOverCurrent() && intTargetPosition == IntConst.slideRetracted) {
            motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motorConfig.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            isIntSlideDown = true;
        }
        if (intTargetPosition == IntConst.slideRetracted && isIntSlideDown)
            motorConfig.intakeMotor.setPower(0);

        if (intTargetPosition == IntConst.slideRetracted && motorConfig.intakeMotor.getCurrentPosition() > 5)
            motorConfig.intakeMotor.setPower(-1);

        else motorConfig.intakeMotor.setPower(intakePower);

        motorConfig.intakeMotor.setPower(intakePower);
    }

    public void panicButton() {
        if (gamepad2.dpad_down || gamepad1.dpad_down) {
            initTimer.reset();
            servoConfig.ptoRight.setPosition(IntConst.ptoLegsUp);
            servoConfig.ptoLeft.setPosition(IntConst.ptoLegsUp);
            servoConfig.ptoRot.setPosition(IntConst.ptoUnlock);
            state = RobotStates.INIT;
        }
    }

    public void lowerLegs() {
        if (gamepad2.dpad_up) {
            state = RobotStates.HANG;
            hang = true;
        }
    }

    public void setInitPos() {
        servoConfig.intRot.setPosition(IntConst.rot_GRAB);
        servoConfig.intY.setPosition(IntConst.y_MIDDLE);
        servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);

        servoConfig.outRight.setPosition(OutConst.lr_INIT_TELE);
        servoConfig.outLeft.setPosition(OutConst.lr_INIT_TELE);
        servoConfig.outY.setPosition(OutConst.y_INIT);
        servoConfig.outLink.setPosition(OutConst.link_INIT);
        servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
    }

}

