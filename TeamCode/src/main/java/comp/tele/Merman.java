package comp.tele;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;
import consts.OutConst;

@TeleOp(name = "\uD83E\uDDDC Merman", group = "TeleOp")
@Disabled
public class Merman extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //outPID and intPID
    public static double oP = 0, oI = 0, oD = 0, oF = 0;
    public static double iP = 0, iI = 0, iD = 0, iF = 0;

    public static int outTargetPosition = OutConst.slidesDown;
    public static int intTargetPosition = OutConst.slidesDown;

    private PIDFController outPID;
    private PIDFController intPID;

    private int fraction;
    private boolean isOutSlideDown;
    private boolean isIntSlideDown;

    //configs
    private MotorConfig motorConfig;
    private ServoConfig servoConfig;

    private enum RobotStates{
        INIT,
        GRAB,
        GRAB_SPECIMEN,
        TRANSFER,
        RETRACT_SAMPLE,
        RETRACT_SPECIMEN,
        INTERMEDIATE,
        SAMPLE_HIGH,
        SAMPLE_LOW,
        SPECIMEN_PICKUP,
        SPECIMEN_PLACE
    }
    private RobotStates state = RobotStates.INIT;


    @Override
    public void runOpMode() {
        motorConfig = new MotorConfig(hardwareMap);
        servoConfig = new ServoConfig(hardwareMap);
        fraction = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        updatePIDFController();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            movement();
            robotState();
            setOutPID();
            setIntPID();
            updatePIDFController();
            panicButton();

            telemetry.addData("state", state);
            telemetry.update();
        }
    }

    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime transferTimer = new ElapsedTime();

    public void robotState(){
        switch (state){
            case INIT:
            {
                servoConfig.setInitPos();
                intTargetPosition = IntConst.slideRetracted;
                outTargetPosition = OutConst.slidesDown;
                fraction = 1;
                if(gamepad1.right_trigger>0){
                    state = RobotStates.GRAB;
                }
                break;
            }
            case GRAB:
            {
                extendIntake();
                fraction = 4;
                if (gamepad1.square)
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                if (gamepad1.circle)
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_90);
                if(gamepad1.right_trigger>0){
                    intakeTimer.reset();
                    state = RobotStates.RETRACT_SAMPLE;
                }
                if(gamepad1.left_trigger>0){
                    intakeTimer.reset();
                    state = RobotStates.RETRACT_SAMPLE;
                }
                break;
            }
            case RETRACT_SAMPLE:
            {
                servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                fraction = 1;
                if(intakeTimer.milliseconds()>300) {
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                    servoConfig.intY.setPosition(IntConst.y_INIT);
                }
                if(intakeTimer.milliseconds()>600){
                    servoConfig.intRot.setPosition(IntConst.rot_INIT);
                    servoConfig.outClaw.setPosition(OutConst.claw_OPEN);
                    transferTimer.reset();
                    state = RobotStates.TRANSFER;
                }
                break;
            }
            case RETRACT_SPECIMEN:
            {
                servoConfig.intClaw.setPosition(IntConst.claw_CLOSED);
                fraction = 1;
                if(intakeTimer.milliseconds()>300) {
                    servoConfig.intClawRot.setPosition(IntConst.clawRot_INIT);
                    servoConfig.intY.setPosition(IntConst.y_DROP);
                }
                if(intakeTimer.milliseconds()>600){
                    servoConfig.intRot.setPosition(IntConst.rot_DROP);
                    if(gamepad1.left_trigger>0){
                        servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                        state = RobotStates.INIT;
                    }
                }
                break;

            }
            case TRANSFER:
            {
                if(transferTimer.milliseconds()>100)
                    servoConfig.outClaw.setPosition(OutConst.claw_PRESSED);
                if(transferTimer.milliseconds()>200)
                    servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
                if(transferTimer.milliseconds()>300) {
                    servoConfig.setOuttakePos(OutConst.lr_SAMPLE, OutConst.link_INIT, OutConst.y_SAMPLE, OutConst.claw_CLOSED);
                    state = RobotStates.INTERMEDIATE;
                }
            }
            case INTERMEDIATE:
            {
                if(-gamepad1.right_stick_y >0.4)
                    state = RobotStates.SAMPLE_HIGH;
                if(-gamepad1.right_stick_y <-0.4)
                    state = RobotStates.SAMPLE_LOW;
                if(gamepad2.left_trigger>0)
                    state = RobotStates.SPECIMEN_PICKUP;
            }
        }
    }

    public void extendIntake(){
        intTargetPosition = IntConst.slideExtended;
        if(motorConfig.frontSlideMotor.getCurrentPosition()>IntConst.slideExtended-100){
            servoConfig.intRot.setPosition(IntConst.rot_GRAB);
            servoConfig.intY.setPosition(IntConst.y_GRAB);
            servoConfig.intClaw.setPosition(IntConst.claw_OPEN);
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

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
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

        if (motorConfig.intakeMotor.getCurrentPosition() < 60 && motorConfig.intakeMotor.getVelocity() < 0.05 && intTargetPosition == IntConst.slideRetracted) {
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

    public void panicButton(){
        if(gamepad2.dpad_down)
            state = RobotStates.INIT;
    }
}

