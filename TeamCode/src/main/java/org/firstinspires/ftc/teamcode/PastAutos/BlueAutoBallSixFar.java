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


@Autonomous(name="OLD BLUE - FAR - 6 BALL", group="1")
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

        double CurrentVoltagePercentage = nominalVoltage / expansionHubVoltageSensor.getVoltage();

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

        // drive forward 4 inches
        driveForwardInches(4, 0.35, lf, lb, rf, rb, pinpoint);


        double launchStrength = 0.45;

        launchStrength *= CurrentVoltagePercentage;

        // spin shooters
        outL.setPower(-launchStrength);
        outR.setPower(-launchStrength);

        // turn 20 deg CCW
        turnToAngle(startHeading + 22.5, imu, lf, lb, rf, rb);

        sleep(400);

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


        // turn to get line
        turnToAngle(startHeading + 43, imu, lf, lb, rf, rb);

        outL.setPower(0);
        outR.setPower(0);

        doManipAndShoot(manip, 1.0 / 6.0);
        intake.setPower(1);

        //driving to spike line
        // drive forward again 25 icnh
        driveForwardInches(22, 0.65, lf, lb, rf, rb, pinpoint);//0.4


        turnToAngle(startHeading + 90, imu, lf, lb, rf, rb);

        //sleep(750);

        //lets go pick up

        driveForwardInches(7, 0.2, lf, lb, rf, rb, pinpoint);// this is moving  little



        //sleep(300);
        doManipAndShoot(manip, 1.0 / 3.0);

        driveForwardInches(6, 0.3, lf, lb, rf, rb, pinpoint);// this is moving  little

        //sleep(300);
        doManipAndShoot(manip, 1.0 / 3.0);

        //driveForwardInches(6, 0.3, lf, lb, rf, rb, pinpoint);// this is moving  little

        //sleep(200);

        driveInches(-29, 0.5, lf, lb, rf, rb, pinpoint);//0.4

        intake.setPower(0);

        turnToAngle(startHeading, imu, lf, lb, rf, rb);

        driveInches(-10, 0.5, lf, lb, rf, rb, pinpoint);//0.4

        driveInches(1,0.5, lf, lb, rf, rb, pinpoint);

        // turn 20 deg CCW
        turnToAngle(startHeading + 22.5, imu, lf, lb, rf, rb);

        //driveInches(5,0.5, lf, lb, rf, rb, pinpoint);

        CurrentVoltagePercentage = nominalVoltage / expansionHubVoltageSensor.getVoltage();

        launchStrength = 0.445;

        launchStrength *= CurrentVoltagePercentage;

        // spin shooters
        outL.setPower(-launchStrength);
        outR.setPower(-launchStrength);
        sleep(500);

        // fire 3
        if (tagID == 21) {
            doManipAndShoot(manip, 1.0 / 6.0); //Green
            sleep(750);
            doManipAndShoot(manip, 1.0 / 3.0); //Purple
            sleep(500);
            //doManipAndShoot(manip, 1.0 / 3.0); //Purple
            //sleep(750);
        }
        else if(tagID == 22){
            doManipAndShoot(manip, -1.0 / 6.0); //Purple
            sleep(750);
            doManipAndShoot(manip, 1.0 / 3.0); //Green
            sleep(500);
            //doManipAndShoot(manip, 1.0 / 3.0); //Purple
            //sleep(750);
        } else if (tagID == 23) {
            doManipAndShoot(manip, -1.0 / 6.0); //Purple
            sleep(500);
            doManipAndShoot(manip, -1.0 / 3.0); //Green
            sleep(500);
            //doManipAndShoot(manip, -1.0 / 3.0); //Purple
            //sleep(750);
        }

        //shoot allbut one
        //sleep(750);
        //doManipAndShoot(manip, 1.0 / 3.0);


        //GET OUTA THERE
        driveInches(6.7, 1, lf, lb, rf, rb, pinpoint);//0.4
//        turnToAngle(startHeading + 90, imu, lf, lb, rf, rb); //turn around
//
//        intake.setPower(0);
//
//        driveForwardInches(38, 0.4, lf, lb, rf, rb, pinpoint);// drive back to start
//
//        turnToAngle(startHeading - 22.5, imu, lf, lb, rf, rb);//shoot again

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

            if (Math.abs(error) < 0.5) break; // 1
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

    void driveInches(
            double inches,
            double power,
            DcMotor lf, DcMotor lb, DcMotor rf, DcMotor rb,
            GoBildaPinpointDriver pinpoint
    ) {
        double direction = Math.signum(inches);
        double targetDistance = Math.abs(inches);
        power = Math.abs(power) * direction;

        pinpoint.update();
        double startX = pinpoint.getPosition().getX(DistanceUnit.INCH);
        double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);

        long startTime = System.currentTimeMillis();
        long timeoutMs = Math.max(2500, (long)(targetDistance / 0.2 * 1000));

        while (opModeIsActive()) {
            pinpoint.update();
            double currentX = pinpoint.getPosition().getX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);

            double dx = currentX - startX;
            double dy = currentY - startY;
            double traveled = Math.sqrt(dx * dx + dy * dy);

            telemetry.addData("Traveled", traveled);
            telemetry.addData("Target", targetDistance);
            telemetry.update();

            if (traveled >= targetDistance) break;
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
    }


}
