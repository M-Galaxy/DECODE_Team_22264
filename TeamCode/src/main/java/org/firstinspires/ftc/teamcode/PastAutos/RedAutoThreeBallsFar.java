package org.firstinspires.ftc.teamcode.PastAutos;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Disabled
@Autonomous(name="RED - FAR - 3 BALL", group="1")
public class RedAutoThreeBallsFar extends LinearOpMode {

    PIDController manipPID = new PIDController(0.008, 0, 0);

    private VoltageSensor expansionHubVoltageSensor;

    @Override
    public void runOpMode() {

        // === IMU ===
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));

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
        int tagID = 21;

        camera.update(imu.getRobotYawPitchRollAngles());

        LLResult Result = camera.GetResult();
        int i = 0;
        while (i < 10) {
            Result = camera.GetResult();
            i += 1;
        }

        if (Result != null) {
            telemetry.addData("TAG OUTPUT", Result.getFiducialResults().get(0).getFiducialId());
            tagID = Result.getFiducialResults().get(0).getFiducialId();
        }

        // Wait 5 seconds so you can read it
        //sleep(5000);

        // save heading
        double startHeading =
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180;

        // drive forward 9 inches
        driveForwardInches(9, 0.35, lf, lb, rf, rb, pinpoint);


        // turn 20 deg CCW
        turnToAngle(startHeading - 22.5, imu, lf, lb, rf, rb);


        double launchStrength = 0.45;

        double CurrentVoltagePercentage = nominalVoltage / expansionHubVoltageSensor.getVoltage();

        launchStrength *= CurrentVoltagePercentage;

        // spin shooters
        outL.setPower(-launchStrength);
        outR.setPower(-launchStrength);
        sleep(700);

        // fire 3
        if (tagID == 21) {
            doManipAndShoot(manip, 1.0 / 6.0); //Green
            sleep(750);
            doManipAndShoot(manip, 1.0 / 3.0); //Purple
            sleep(750);
            doManipAndShoot(manip, 1.0 / 3.0); //Purple
            sleep(750);
        }
        else if(tagID == 22){
            doManipAndShoot(manip, -1.0 / 6.0); //Purple
            sleep(750);
            doManipAndShoot(manip, 1.0 / 3.0); //Green
            sleep(750);
            doManipAndShoot(manip, 1.0 / 3.0); //Purple
            sleep(750);
        } else if (tagID == 23) {
            doManipAndShoot(manip, -1.0 / 6.0); //Purple
            sleep(750);
            doManipAndShoot(manip, -1.0 / 3.0); //Green
            sleep(750);
            doManipAndShoot(manip, -1.0 / 3.0); //Purple
            sleep(750);
        }

        // turn back to original heading
        turnToAngle(startHeading, imu, lf, lb, rf, rb);

        // drive forward again 10 icnh
        driveForwardInches(10, 0.4, lf, lb, rf, rb, pinpoint);


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
        double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);

        long startTime = System.currentTimeMillis();
        long timeoutMs = 2500; // safety

        while (opModeIsActive()) {
            pinpoint.update();
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);
            double traveled = Math.abs(currentY - startY);

            telemetry.addData("Pinpoint Y", currentY);
            telemetry.addData("Start Y", startY);
            telemetry.addData("Traveled (in)", traveled);
            telemetry.addData("Target (in)", inches);
            telemetry.update();

            if (traveled >= inches) break;
            if (System.currentTimeMillis() - startTime > timeoutMs) break;

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
        sleep(100);
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

            if (Math.abs(error) < 1.0) break;
            if (System.currentTimeMillis() - start > 2000) break;

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
        sleep(250);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
