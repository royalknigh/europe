package noncomp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;

import configs.MotorConfig;
import consts.IntConst;
import consts.OutConst;

@Config
@TeleOp(name = "Outtake Dashboard", group = "NonComp")
public class Outtake extends OpMode {

    private FtcDashboard dashboard;
    private MotorConfig motorConfig;

    private boolean areSlidesDown = true;


    public static double oP = 0, oI = 0, oD = 0, oF = 0;

    private PIDFController pidfController;
    public static double targetPosition = 0;

    public static double iP = 0, iI = 0, iD = 0, iF = 0;
    private PIDFController intPID;
    public static int intTargetPosition = 0;

    public Servo intRot;
    public Servo intY;
    public Servo intClawRot;
    public Servo intClaw;

    public Servo outLeft;
    public Servo outRight;
    public Servo outLink;
    public Servo outY;
    public Servo outClaw;


    private boolean isIntSlideDown;

    public static double intakeRot = IntConst.rot_INIT;
    public static double clawRot = IntConst.clawRot_INIT;
    public static double intakeY = IntConst.y_INIT;
    public static double intakeClaw = IntConst.claw_OPEN;

    public static double outLr = OutConst.lr_INIT;
    public static double outlink;
    public static double outy;
    public static double outclaw;

    @Override
    public void init() {
        motorConfig = new MotorConfig(hardwareMap);

        outLeft = hardwareMap.get(Servo.class, "outLeft");
        outRight = hardwareMap.get(Servo.class, "outRight");
        outY = hardwareMap.get(Servo.class, "outY");
        outLink = hardwareMap.get(Servo.class, "outLink");
        outClaw = hardwareMap.get(Servo.class, "outClaw");

        intRot = hardwareMap.get(Servo.class, "intRot");
        intY = hardwareMap.get(Servo.class, "intY");
        intClawRot = hardwareMap.get(Servo.class, "intClawRot");
        intClaw = hardwareMap.get(Servo.class, "intClaw");

        outRight.setDirection(Servo.Direction.REVERSE);

        updatePIDFController1();

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();



//        intRot.setPosition(IntConst.rot_INIT);
//        intY.setPosition(IntConst.y_INIT);
//        intClawRot.setPosition(IntConst.clawRot_INIT);
//        intClaw.setPosition(IntConst.claw_CLOSED);
    }

    @Override
    public void loop() {
//        setIntPID();
//        updatePIDFController();

        updatePIDFController1();

        setServoPositions();

        pidfController.setTargetPosition(targetPosition);
        pidfController.updatePosition(motorConfig.frontSlideMotor.getCurrentPosition());

        double powerLeft = pidfController.runPIDF();


            motorConfig.frontSlideMotor.setPower(powerLeft);
            motorConfig.backSlideMotor.setPower(powerLeft);

        if (-gamepad1.left_stick_y > 0.1) {
            targetPosition += 40;
        } else if (-gamepad1.left_stick_y < -0.1) {
            targetPosition -= 40;
        }

        telemetry.addData("velocity", MotorConfig.frontSlideMotor.getVelocity());
        telemetry.addData("areSlidesDown", areSlidesDown);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Left Motor Position", motorConfig.frontSlideMotor.getCurrentPosition());

        telemetry.addData("Left Motor Error", motorConfig.frontSlideMotor.getCurrentPosition() - targetPosition);
        telemetry.addData("Right Motor Error", motorConfig.backSlideMotor.getCurrentPosition() - targetPosition);

        telemetry.addData("Right Motor Power", motorConfig.backSlideMotor.getPower());
        telemetry.addData("Left Motor Power", motorConfig.frontSlideMotor.getPower());

        telemetry.update();
    }

    private void updatePIDFController1() {
        CustomPIDFCoefficients coefficients = new CustomPIDFCoefficients(oP, oI, oD, oF);
        pidfController = new PIDFController(coefficients);
    }

    public void setServoPositions() {
        outLeft.setPosition(outLr);
        outRight.setPosition(outLr);
        outY.setPosition(outy);
        outLink.setPosition(outlink);
        outClaw.setPosition(outclaw);

        intClaw.setPosition(intakeClaw);
        intRot.setPosition(intakeRot);
        intY.setPosition(intakeY);
        intClawRot.setPosition(clawRot);

    }

    public void setIntPID() {
        intPID.setTargetPosition(intTargetPosition);
        intPID.updatePosition(motorConfig.intakeMotor.getCurrentPosition());

        double IntakePower = intPID.runPIDF();

        if (Math.abs(motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition) < 10)
            IntakePower = 0;

        if (motorConfig.intakeMotor.getCurrentPosition() > 10) isIntSlideDown = false;

        if (motorConfig.intakeMotor.getCurrentPosition() < 60 && motorConfig.intakeMotor.getVelocity() < 0.05 && intTargetPosition == IntConst.slideRetracted) {
            MotorConfig.intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            MotorConfig.intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            isIntSlideDown = true;
        }

        if (intTargetPosition == IntConst.slideRetracted && isIntSlideDown)
            motorConfig.intakeMotor.setPower(0);

        if (intTargetPosition == IntConst.slideRetracted && motorConfig.intakeMotor.getCurrentPosition() > 10)
            motorConfig.intakeMotor.setPower(-1);

        else motorConfig.intakeMotor.setPower(IntakePower);

        motorConfig.intakeMotor.setPower(IntakePower);

    }

    public void updatePIDFController() {
        CustomPIDFCoefficients intCoefficients = new CustomPIDFCoefficients(iP, iI, iD, iF);
        intPID = new PIDFController(intCoefficients);
    }
}