package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "GolfBot" ,group = "Linear Opmode")

//@Disabled
public class GolfBot extends LinearOpMode {
    //Dashboard demo variables
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;
    public FtcDashboard dashboard;

    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRigtDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;

    public void initializeMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        frontRigtDrive = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        frontRigtDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRigtDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRigtDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    public void initializeDashboard()
    {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }

    public void dashboardDemo(){
        double time = getRuntime();

        double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
        double l = SIDE_LENGTH / 2;

        double[] bxPoints = { l, -l, -l, l };
        double[] byPoints = { l, l, -l, -l };
        rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
        for (int i = 0; i < 4; i++) {
            bxPoints[i] += bx;
            byPoints[i] += by;
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setStroke("goldenrod")
                .strokeCircle(0, 0, ORBITAL_RADIUS)
                .setFill("black")
                .fillPolygon(bxPoints, byPoints);
        dashboard.sendTelemetryPacket(packet);

        sleep(20);
    }


    private void joystickTankDrive()
    {
        float Lj_y;
        float Rj_y;

        Lj_y = gamepad1.left_stick_y;
        Rj_y = gamepad1.right_stick_y;

        frontLeftDrive.setPower(Lj_y);
        frontRigtDrive.setPower(Rj_y);
        backLeftDrive.setPower(Lj_y);
        backRightDrive.setPower(Rj_y);
    }

    public void runOpMode()  {
        initializeDashboard();
        initializeMotors();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("FL (0)", frontLeftDrive.getCurrentPosition());
            telemetry.addData("FR (1)", frontRigtDrive.getCurrentPosition());
            telemetry.addData("BL (2)", backLeftDrive.getCurrentPosition());
            telemetry.addData("BR (3)", backRightDrive.getCurrentPosition());
            updateTelemetry(telemetry);

            joystickTankDrive();

//            dashboardDemo();
        }
    }
}

