package org.firstinspires.ftc.teamcode.autos.Main;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Autonomous(name="BLUE - FAR - 6 BALL", group="0")
public class BlueAutoBallSixFar extends LinearOpMode {
//kp 0.008
    PIDController manipPID = new PIDController(0.0075, 0, 0);

    private VoltageSensor expansionHubVoltageSensor;

    @Override
    public void runOpMode() {

        // === IMU ===
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

        // == INTAKE ==
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        // === PINPOINT ===
        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // === CAMERA ===
        Limelight camera = new Limelight(
                hardwareMap.get(Limelight3A.class, "limelight"),
                imu.getRobotYawPitchRollAngles()
        );

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

        // === MANIPULATOR ===
        DcMotor manip = hardwareMap.dcMotor.get("bigWheel");
        manip.setDirection(DcMotorSimple.Direction.REVERSE);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        manip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manipPID.setTarget(manip.getCurrentPosition());

        // === SHOOTERS ===
        DcMotor outL = hardwareMap.dcMotor.get("outL");
        DcMotor outR = hardwareMap.dcMotor.get("outR");
        outR.setDirection(DcMotorSimple.Direction.REVERSE);

        expansionHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 1");
        double nominalVoltage = 13;
        // =========================
        // WAIT FOR START
        // =========================
        waitForStart();
        if (isStopRequested()) return;

        // reset sensors AFTER start
        imu.resetYaw();
        pinpoint.resetPosAndIMU();


        camera.setTargetTag(20);

        camera.start();

        // =========================
        // AUTO SEQUENCE
        // =========================


        // Get Obelisk detection
        int obelisk = camera.getObelisk();
        telemetry.addData("Detected Obelisk", obelisk);
        telemetry.update();

        // Wait 5 seconds so you can read it
        //sleep(5000);

        // save heading
        double startHeading =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180;

        // drive forward 4 inches
        driveForwardInches(4, 0.35, lf, lb, rf, rb, pinpoint);


    // turn 20 deg CCW
        turnToAngle(startHeading + 22.5, imu, lf, lb, rf, rb);


        double launchStrength = 0.425;

        double CurrentVoltagePercentage = nominalVoltage / expansionHubVoltageSensor.getVoltage();

        launchStrength *= CurrentVoltagePercentage;

    // spin shooters
        outL.setPower(-launchStrength);
        outR.setPower(-launchStrength);
        sleep(700);

    // fire 3
        doManipAndShoot(manip, 1.0 / 6.0); //so it stops a little short
        sleep(750);
        doManipAndShoot(manip, 1.0 / 3.0);
        sleep(750);
        doManipAndShoot(manip, 1.0 / 3.0);
       sleep(750);
        doManipAndShoot(manip, 1.0 / 6.0);
        sleep(750);
    // turn back to original heading
        turnToAngle(startHeading, imu, lf, lb, rf, rb);

    // drive forward again 24 icnh
        driveForwardInches(20, 0.7, lf, lb, rf, rb, pinpoint);//0.4


        turnToAngle(startHeading + 90, imu, lf, lb, rf, rb);

        intake.setPower(1);

        driveForwardInches(19, 0.5, lf, lb, rf, rb, pinpoint);

        driveForwardInches(6, 0.1, lf, lb, rf, rb, pinpoint);// this is moving  little

        doManipAndShoot(manip, 1.0 / 3.0);

        driveForwardInches(6, 0.1, lf, lb, rf, rb, pinpoint);// this is moving  little

        doManipAndShoot(manip, 1.0 / 3.0);

        driveForwardInches(6, 0.1, lf, lb, rf, rb, pinpoint);// this is moving  little

        turnToAngle(startHeading + 90, imu, lf, lb, rf, rb); //turn around

        intake.setPower(0);

        driveForwardInches(38, 0.4, lf, lb, rf, rb, pinpoint);// drive back to start

        turnToAngle(startHeading + 22.5, imu, lf, lb, rf, rb);//shoot again

        // stop everything
        outL.setPower(0);
        outR.setPower(0);
    }

    // =====================================================
    // DRIVE USING PINPOINT Y
    // =====================================================
    void driveForwardInches(
            double inches,
            double power,
            DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
            GoBildaPinpointDriver pinpoint
    ) {
        pinpoint.update();
        double startX = pinpoint.getPosition().getX(DistanceUnit.INCH);
        double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);

        long startTime = System.currentTimeMillis();
        long timeoutMs = Math.max(2500, (long)(inches / 0.2 * 1000)); // scale timeout with distance

        while (opModeIsActive()) {
            pinpoint.update();
            double currentX = pinpoint.getPosition().getX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);

            double dx = currentX - startX;
            double dy = currentY - startY;
            double traveled = Math.sqrt(dx*dx + dy*dy);

            telemetry.addData("Pinpoint X", currentX);
            telemetry.addData("Pinpoint Y", currentY);
            telemetry.addData("Traveled (in)", traveled);
            telemetry.addData("Target (in)", inches);
            telemetry.update();

            if (traveled >= inches) break;
            if (System.currentTimeMillis() - startTime > timeoutMs) break;

            // Maintain direction based on current heading
            lf.setPower(power);
            lb.setPower(power);
            rf.setPower(power);
            rb.setPower(power);
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        telemetry.addLine("Drive segment complete");
        telemetry.update();
        //sleep(100);
    }



    // =====================================================
    // TURN USING IMU
    // =====================================================
    void turnToAngle(
            double targetDeg,
            IMU imu,
            DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb
    ) {
        double kP = 0.01;
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(targetDeg - yaw);

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetDeg);
            telemetry.addData("Error", error);
            telemetry.update();

            if (Math.abs(error) < 0.25) break; // 1
            if (System.currentTimeMillis() - start > 1500) break;

            double power = error * kP;
            power = Math.max(-0.25, Math.min(0.25, power));

            lf.setPower(power);
            lb.setPower(power);
            rf.setPower(-power);
            rb.setPower(-power);
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }


    // =====================================================
    // SHOOTER MANIP
    // =====================================================
    void doManipAndShoot(DcMotor manip, double revolutions) {
        double ticks = revolutions * 1075.4; // 537.7 * 2
        double target = manip.getCurrentPosition() + ticks;
        manipPID.setTarget(target);

        long t0 = System.currentTimeMillis();

        while (opModeIsActive()
                && Math.abs(manipPID.getError(manip.getCurrentPosition())) > 0.25
                && System.currentTimeMillis() - t0 < 1500) {

            double output = manipPID.calculateOutput(manip.getCurrentPosition()) * 0.4;

            // Enforce a minimum power to overcome static friction for small moves
            if (Math.abs(ticks) < 200) { // roughly 1/6 rev or less
                output = Math.signum(output) * Math.max(Math.abs(output), 0.2);
            }

            manip.setPower(output);
        }

        manip.setPower(0);
        //sleep(250);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
