package org.firstinspires.ftc.teamcode.autos.Main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Disabled
@Autonomous(name="BLUE - Far (ODO)", group="0")
public class BlueAutoOld extends LinearOpMode {

    final double encoderPPR = 537.7 * 2;
    PIDController manipPID = new PIDController(0.008, 0, 0);

    // ODO CONSTANTS
    final double TICKS_PER_REV = 8192.0;
    final double WHEEL_RADIUS_MM = 17.5;
    final double TRACK_WIDTH_MM = 267.97;
    final double PERP_OFFSET_MM = 116.078;

    @Override
    public void runOpMode() throws InterruptedException {

        // === DRIVETRAIN ===
        DcMotor lf = hardwareMap.dcMotor.get("LF");
        DcMotor lb = hardwareMap.dcMotor.get("LB");
        DcMotor rf = hardwareMap.dcMotor.get("RF");
        DcMotor rb = hardwareMap.dcMotor.get("RB");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // === ODOMETRY WHEELS ===
        // NOTE: if these are NOT separate odometry wheels, replace names with your odometry motor names.
        DcMotor odoL = hardwareMap.dcMotor.get("RF");
        DcMotor odoR = hardwareMap.dcMotor.get("intake");
        DcMotor odoS = hardwareMap.dcMotor.get("LF");

        odoL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        ThreeWheelOdometry odo = new ThreeWheelOdometry(
//                TICKS_PER_REV, WHEEL_RADIUS_MM, TRACK_WIDTH_MM, PERP_OFFSET_MM, true
//        );

        // we use an array so the thread can mutate these "last" values
        final int[] last = new int[3];

        // === IMU ===
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // === MANIPULATOR ===
        DcMotor manip = hardwareMap.dcMotor.get("bigWheel");
        manip.setDirection(DcMotorSimple.Direction.REVERSE);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        manip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double manipTarget = manip.getCurrentPosition();
        manipPID.setTarget(manipTarget);

        // === LAUNCHERS ===
        DcMotor outL = hardwareMap.dcMotor.get("outL");
        DcMotor outR = hardwareMap.dcMotor.get("outR");
        outR.setDirection(DcMotorSimple.Direction.REVERSE);

        // === CAMERA ===
//        CamHandler cam = new CamHandler(
//                hardwareMap.get(HuskyLens.class, "camera"),
//                true,
//                telemetry
//        );

        // initialize last readings from odometry BEFORE starting updates
        last[0] = odoL.getCurrentPosition();
        last[1] = odoR.getCurrentPosition();
        last[2] = odoS.getCurrentPosition();

        waitForStart();
        if (isStopRequested()) return;

        // Start odometry update thread AFTER start and after initial readings
        Thread odoThread = new Thread(() -> {
            while (opModeIsActive()) {
                int L = odoL.getCurrentPosition();
                int R = odoR.getCurrentPosition();
                int S = odoS.getCurrentPosition();

                int dL = L - last[0];
                int dR = R - last[1];
                int dS = S - last[2];

                last[0] = L;
                last[1] = R;
                last[2] = S;

                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                //odo.update(dL, dR, dS, heading);

                // small sleep to avoid busy-wait / CPU hog
                try {
                    Thread.sleep(10);
                } catch (InterruptedException ignored) {}
            }
        });
        odoThread.start();

        imu.resetYaw();

        // ------------------------------------------------------------------
        // 1. DRIVE RIGHT 787mm USING ODO
        // ------------------------------------------------------------------
        telemetry.addLine("moving to scan");
        telemetry.update();
        //strafeMM(7.5, 0.35, lf, lb, rf, rb, odo);
        sleep(100); // 200

        // ------------------------------------------------------------------
        // 2. SCAN CAMERA FOR 3 SECONDS
        // ------------------------------------------------------------------
        telemetry.addLine("scaning...");
        telemetry.update();
        double obId = 0;
        long scanStart = System.currentTimeMillis();

        while (opModeIsActive() &&
                System.currentTimeMillis() - scanStart < 3000 &&
                obId == 0) {

            //obId = cam.getObolisk();
            telemetry.addData("Scan", obId);
            telemetry.update();
        }

        // ------------------------------------------------------------------
        // 3. TURN
        // ------------------------------------------------------------------
        telemetry.addLine("turning");
        telemetry.update();
//        turnToAngle(-90, imu, lf, lb, rf, rb);

        // ------------------------------------------------------------------
        // 4. SHOOTING STUFF
        // ------------------------------------------------------------------
        telemetry.addLine("shooting");
        telemetry.update();
        outL.setPower(-0.37);
        outR.setPower(-0.37);
        sleep(700);

        if(obId == 5) {
            //g p p
            doManipAndShoot(manip, 1.0 / 6.0);
            sleep(500);
            doManipAndShoot(manip, 1.0 / 3.0);
            sleep(500);
            doManipAndShoot(manip, 1.0 / 3.0);
            sleep(500);
        } else if (obId == 3) {
            // p g p
            doManipAndShoot(manip, -1.0 / 6.0);
            sleep(500);
            doManipAndShoot(manip, 1.0 / 3.0);
            sleep(500);
            doManipAndShoot(manip, 1.0 / 3.0);
            sleep(500);
        }else{
            //p p g
            doManipAndShoot(manip, -1.0 / 6.0);
            sleep(500);
            doManipAndShoot(manip, -1.0 / 3.0);
            sleep(500);
            doManipAndShoot(manip, -1.0 / 3.0);
            sleep(500);
        }

        // ------------------------------------------------------------------
        // 5. EXIT LINE
        // ------------------------------------------------------------------
        telemetry.addLine("move off line");
        telemetry.update();
        //strafeMM(250, 0.35, lf, lb, rf, rb, odo);

        // STOP
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
        outL.setPower(0);
        outR.setPower(0);

        // ensure odometry thread ends (it will naturally because opModeIsActive() false)
        try {
            odoThread.join(50);
        } catch (InterruptedException ignored) {}
    }

    // ===================== ODOMETRY MOVEMENT =====================

//    void strafeMM(double mm, double power,
//                  DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
//                  ThreeWheelOdometry odo) {
//
//        double startX = odo.getX();
//        double target = startX + mm;
//
//        while (opModeIsActive() && odo.getX() < target) {
//            // simple strafe right
//            lf.setPower(power);
//            lb.setPower(-power);
//            rf.setPower(-power);
//            rb.setPower(power);
//
//            telemetry.addData("odo X", odo.getX());
//            telemetry.addData("target X", target);
//            telemetry.update();
//        }
//
//        lf.setPower(0); lb.setPower(0);
//        rf.setPower(0); rb.setPower(0);
//    }
//
//    void turnToAngle(double targetDeg, IMU imu,
//                     DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb) {
//
//        double kp = 0.01;
//        long timeout = System.currentTimeMillis() + 2500;
//
//        while (opModeIsActive()) {
//
//            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//
//            // normalize IMU wraparound
//            double error = AngleUnit.normalizeDegrees(targetDeg - yaw);
//
//            if (Math.abs(error) < 1) break;
//            if (System.currentTimeMillis() > timeout) break;
//
//            double pwr = error * kp;
//
//            // minimum turning power
//            if (Math.abs(pwr) < 0.08)
//                pwr = Math.signum(pwr) * 0.08;
//
//            //pwr = pwr/2;
//            if (pwr > 0){
//                pwr = 0.25;
//            }
//            else{
//                pwr = -0.25;
//            }
//
//
//            // MOTOR POWER
//            lf.setPower(pwr);
//            lb.setPower(pwr);
//            rf.setPower(-pwr);
//            rb.setPower(-pwr);
//
//            telemetry.addData("Yaw", yaw);
//            telemetry.addData("Error", error);
//            telemetry.addData("Power", pwr);
//            telemetry.update();
//
//            sleep(10); // <- prevents thread starvation
//        }
//
//        lf.setPower(0);
//        lb.setPower(0);
//        rf.setPower(0);
//        rb.setPower(0);
//    }


    // your manip shooting stays unchanged
    void doManipAndShoot(DcMotor manip, double revolutions) {
        double ticks = revolutions * encoderPPR;
        double start = manip.getCurrentPosition();
        double target = start + ticks;

        manipPID.setTarget(target);

        long t0 = System.currentTimeMillis();
        while (opModeIsActive() &&
                Math.abs(manipPID.getError(manip.getCurrentPosition())) > 1 &&
                System.currentTimeMillis() - t0 < 2000) {

            double p = manipPID.calculateOutput(manip.getCurrentPosition());
            manip.setPower(p * 0.5);
        }
        manip.setPower(0);
        sleep(300);
    }
}
