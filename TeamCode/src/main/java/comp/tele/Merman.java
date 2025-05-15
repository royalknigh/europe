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
        TRANSFER,
        RETRACT
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
        }
    }

    public ElapsedTime intakeTimer = new ElapsedTime();

    public void robotState(){
        switch (state){
            case INIT:
            {
                servoConfig.setInitPos();
                fraction =1;
                if(gamepad1.a){
                    state = RobotStates.GRAB;
                }
            }
            case GRAB:
            {
                extend();
                servoConfig.setOuttakePos(OutConst.Lr_TRANSFER, OutConst.Y_TRANSFER, OutConst.Link_TRANSFER, OutConst.Claw_TRANSFER);
                fraction = 4;

                if (gamepad1.square)
                    servoConfig.intClawRot.setPosition(IntConst.ClawRot_INIT);
                if (gamepad1.circle)
                    servoConfig.intClawRot.setPosition(IntConst.ClawRot_90);

                if(gamepad1.right_trigger>0){
                    intakeTimer.reset();
                    state = RobotStates.RETRACT;
                }
            }
            case RETRACT:
            {
                servoConfig.intClaw.setPosition(IntConst.Claw_CLOSED);

                if(intakeTimer.milliseconds()>100)
                    servoConfig.intY.setPosition(IntConst.Y_INIT);
                if(intakeTimer.milliseconds()>300){
                    servoConfig.intRot.setPosition(IntConst.Rot_INIT);
                    servoConfig.intClawRot.setPosition(IntConst.ClawRot_INIT);
                }


            }

        }
    }

    public void extend(){
        intTargetPosition = 900;
        servoConfig.intRot.setPosition(IntConst.Rot_GRAB);
        servoConfig.intY.setPosition(IntConst.Y_GRAB);
        servoConfig.intClaw.setPosition(IntConst.Claw_OPEN);

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

        double OuttakePower = outPID.runPIDF();

        if (Math.abs(motorConfig.frontSlideMotor.getCurrentPosition() - outTargetPosition) < 10) {
            OuttakePower = 0;
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
            motorConfig.frontSlideMotor.setPower(OuttakePower);
            motorConfig.backSlideMotor.setPower(OuttakePower);
        }

        motorConfig.frontSlideMotor.setPower(OuttakePower);
        motorConfig.backSlideMotor.setPower(OuttakePower);
    }

    public void setIntPID() {
        intPID.setTargetPosition(intTargetPosition);
        intPID.updatePosition(motorConfig.intakeMotor.getCurrentPosition());

        double IntakePower = intPID.runPIDF();

        if (Math.abs(motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition) < 10)
            IntakePower = 0;

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

        else motorConfig.intakeMotor.setPower(IntakePower);

        motorConfig.intakeMotor.setPower(IntakePower);

    }
}

