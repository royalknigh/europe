package noncomp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import configs.MotorConfig;
import configs.ServoConfig;
import consts.IntConst;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Intake Dashboard", group = "NonComp")
public class Intake extends LinearOpMode {
    private FtcDashboard dashboard;

    private ServoConfig servoConfig;
    private MotorConfig motorConfig;

    public Servo intRot;
    public Servo intY;
    public Servo intClawRot;
    public Servo intClaw;
    public Servo ptoRot;
    public Servo ptoLeft;
    public Servo ptoRight;


    public static double iP = 0, iI = 0, iD = 0, iF = 0;
    public PIDFController intPID;
    public static int intTargetPosition = 0;
    private boolean isIntSlideDown;

    public static double intakeRot = IntConst.rot_INIT;
    public static double clawRot = IntConst.clawRot_INIT;
    public static double intakeY = IntConst.y_INIT;
    public static double intakeClaw = IntConst.claw_OPEN;
    public static double ptorot = IntConst.ptoUnlock ;
    public static double legs;


    @Override
    public void runOpMode() {
        motorConfig = new MotorConfig(hardwareMap);

        intRot = hardwareMap.get(Servo.class, "intRot");
        intY = hardwareMap.get(Servo.class, "intY");
        intClawRot = hardwareMap.get(Servo.class, "intClawRot");
        intClaw = hardwareMap.get(Servo.class, "intClaw");

        ptoRot = hardwareMap.get(Servo.class, "ptoRot");
        ptoLeft = hardwareMap.get(Servo.class, "ptoLeft");
        ptoRight = hardwareMap.get(Servo.class, "ptoRight");

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            updatePIDFController();
            setIntPID();

            intClaw.setPosition(intakeClaw);
            intRot.setPosition(intakeRot);
            intY.setPosition(intakeY);
            intClawRot.setPosition(clawRot);

            ptoRot.setPosition(ptorot);
            ptoLeft.setPosition(legs);
            ptoRight.setPosition(legs);

            telemetry.addData("intakeRot", intakeRot);

            telemetry.addData("Target Position", intTargetPosition);
            telemetry.addData(" Motor Position", motorConfig.intakeMotor.getCurrentPosition());

            telemetry.update();
        }
    }

    public void setIntPID() {
        intPID.setTargetPosition(intTargetPosition);
        intPID.updatePosition(motorConfig.intakeMotor.getCurrentPosition());

        double IntakePower = intPID.runPIDF();

        if (Math.abs(motorConfig.intakeMotor.getCurrentPosition() - intTargetPosition) < 10)
            IntakePower = 0;

        if (motorConfig.intakeMotor.getCurrentPosition() > 10) isIntSlideDown = false;

        if (motorConfig.intakeMotor.getCurrentPosition() < 30 && motorConfig.intakeMotor.getVelocity() < 0.05 && intTargetPosition == IntConst.slideRetracted) {
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
