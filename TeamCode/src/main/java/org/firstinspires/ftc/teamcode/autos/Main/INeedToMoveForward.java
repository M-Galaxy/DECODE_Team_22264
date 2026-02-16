package org.firstinspires.ftc.teamcode.autos.Main;

import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.PIDController;

@Autonomous(name="Drive Forward", group="0")
public class INeedToMoveForward extends LinearOpMode {

    PIDController manipPID = new PIDController(0.008, 0, 0);


    private VoltageSensor expansionHubVoltageSensor;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timePerShot = new ElapsedTime();
    double timeLimit = 2;

    ElapsedTime TurnTmer = new ElapsedTime();

    public static final double MAX_LAUNCHER_RPM = 6000.0;

    // PID (RPM-based)
    public static double kP = 0.0032;
    public static double kI = 0.000001;
    public static double kD = 0.0001;
    public static double kF = 1.0 / MAX_LAUNCHER_RPM;


    private double integralSum = 0;
    private double lastError = 0;

    public static final double LAUNCHER_TPR = 28.0;


    @Override
    public void runOpMode() {

        DcMotorEx outL =  hardwareMap.get(DcMotorEx.class, "outL");
        DcMotorEx outR = hardwareMap.get(DcMotorEx.class, "outR");

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
        outL.setDirection(DcMotorSimple.Direction.REVERSE);

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
        timer.reset();
        camera.start();

        // =========================
        // AUTO SEQUENCE
        // =========================


        // Get Obelisk detection
        //Update the camera's data

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

        outL.setPower(0.25);//I wanna rev em
        outR.setPower(0.25);// so much rev

        //drive forward 8 inches
        driveForwardInches(25, 0.2, lf, lb, rf, rb, pinpoint);



//        camera.swichPipe(1);
//        // turn 20 deg CCW
//        //turnToAngle(startHeading + 22.5, imu, lf, lb, rf, rb);
//        TurnTmer.reset();
//        double camer = 80;
//        boolean withinTolerance = false;
//        while((TurnTmer.seconds() < 4)){// && (!withinTolerance)){// 3 seconds of correcting
//            double rx = 0;
//            camer = camera.getTargetXError();
//            if (abs(camer) > 80) {// if far move at 50%
//                if (camer > 0) {
//                    rx = 0.5;
//                } else {
//                    rx = -0.5;
//                }
//            } else if (abs(camer) > 20) {// if close move at 25%
//                if (camer > 0) {
//                    rx = 0.25;
//                } else {
//                    rx = -0.25;
//                }
//            }else if (abs(camer) > 10) {// if close move at 25%
//                if (camer > 0) {
//                    rx = 0.125;
//                } else {
//                    rx = -0.125;
//                }
//            }else if (abs(camer) < 5){
//                withinTolerance = true;
//            }
//            // if close enough move at 0%
//            lf.setPower(rx);
//            lb.setPower(rx);
//            rf.setPower(-rx);
//            rb.setPower(-rx);
//        }
//
//        lf.setPower(0);
//        lb.setPower(0);
//        rf.setPower(0);
//        rb.setPower(0);
//
//
//        double launchStrength = 0.53;
//
//        double CurrentVoltagePercentage = nominalVoltage / expansionHubVoltageSensor.getVoltage();
//
//        launchStrength *= CurrentVoltagePercentage;
//
//        double rmpForFar = 2380;
//
//        // spin shooters
//        waitTillReady(rmpForFar, outL, outR);
//
//        // fire 3
//        if (tagID == 21) {
//            doManipAndShoot(manip, 1.0 / 6.0, outL, outR, rmpForFar); //Green
//            waitTillReady(rmpForFar, outL, outR);sleep(300);
//            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
//            waitTillReady(rmpForFar, outL, outR);
//            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
//            //sleep(500);
//            waitTillReady(rmpForFar, outL, outR);
//        }
//        else if(tagID == 22){
//            doManipAndShoot(manip, -1.0 / 6.0, outL, outR, rmpForFar); //Purple
//            waitTillReady(rmpForFar, outL, outR);sleep(300);
//            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Green
//            waitTillReady(rmpForFar, outL, outR);
//            doManipAndShoot(manip, 1.0 / 3.0, outL, outR, rmpForFar); //Purple
//            //sleep(500);
//            waitTillReady(rmpForFar, outL, outR);
//        } else if (tagID == 23) {
//            doManipAndShoot(manip, -1.0 / 6.0, outL, outR, rmpForFar); //Purple
//            waitTillReady(rmpForFar, outL, outR);sleep(300);
//            doManipAndShoot(manip, -1.0 / 3.0, outL, outR, rmpForFar); //Green
//            waitTillReady(rmpForFar, outL, outR);
//            doManipAndShoot(manip, -1.0 / 3.0, outL, outR, rmpForFar); //Purple
//            //sleep(500);
//            waitTillReady(rmpForFar, outL, outR);
//        }
//
//        // stop everything
//        outL.setPower(0);
//        outR.setPower(0);
//
//        // turn back to original heading
//        turnToAngle(startHeading, imu, lf, lb, rf, rb);
//
//
//
//        // drive forward again 10 inch
//        driveForwardInches(20, 0.25, lf, lb, rf, rb, pinpoint);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    void waitTillReady(double targetRPM, DcMotorEx outL, DcMotorEx outR){
        timePerShot.reset();
        double averageRPM = 0;
        while ((timePerShot.seconds() < 4.20) && !(abs(targetRPM - averageRPM) < 20)){
            double power = returnPID(targetRPM, averageRPM);

            outL.setPower(power);
            outR.setPower(power);

            double velL = outL.getVelocity();
            double velR = outR.getVelocity();
            double avgTicksPerSec = (velL + velR) / 2.0;
            telemetry.update();

            averageRPM = (avgTicksPerSec / LAUNCHER_TPR) * 60.0;
        }
    }

    void spinTheLaunchers(double targetRPM, DcMotorEx outL, DcMotorEx outR){
        double velL = outL.getVelocity();
        double velR = outR.getVelocity();
        double avgTicksPerSec = (velL + velR) / 2.0;
        double averageRPM = (avgTicksPerSec / LAUNCHER_TPR) * 60.0;

        double power = returnPID(targetRPM, averageRPM);


        telemetry.addData("Average RPM", averageRPM);
        telemetry.addData("Target RPM``", targetRPM);

        outL.setPower(power);
        outR.setPower(power);
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

        double P = 0.1;

        long startTime = System.currentTimeMillis();
        long timeoutMs = Math.max(2500, (long)(inches / 0.2 * 1000)); // scale timeout with distance

        while (opModeIsActive()) {
            pinpoint.update();
            double currentX = pinpoint.getPosition().getX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosition().getY(DistanceUnit.INCH);

            double dx = currentX - startX;
            double dy = currentY - startY;
            double traveled = Math.sqrt(dx*dx + dy*dy);

            power = 1*((inches - traveled) * P);

            telemetry.addData("Pinpoint X", currentX);
            telemetry.addData("Pinpoint Y", currentY);
            telemetry.addData("Traveled (in)", traveled);
            telemetry.addData("Target (in)", inches);
            telemetry.addData("Power", power);
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
        double kP = 0.005;// 0.01;
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(targetDeg - yaw);

            telemetry.addData("Yaw", yaw);
            telemetry.addData("Target", targetDeg);
            telemetry.addData("Error", error);
            telemetry.update();

            if (Math.abs(error) < 2.0) break;
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
    void doManipAndShoot(DcMotor manip, double revolutions, DcMotorEx outL, DcMotorEx outR, double targetRPM) {
        double ticks = revolutions * 1075.4; // 537.7 * 2
        double target = manip.getCurrentPosition() + ticks;
        manipPID.setTarget(target);

        long t0 = System.currentTimeMillis();

        while (opModeIsActive()
                && Math.abs(manipPID.getError(manip.getCurrentPosition())) > 0.25
                && System.currentTimeMillis() - t0 < 2000) {//1500

            double output = manipPID.calculateOutput(manip.getCurrentPosition()) * 0.4;

            // Enforce a minimum power to overcome static friction for small moves
            if (Math.abs(ticks) < 300) { // roughly 1/6 rev or less
                output = Math.signum(output) * Math.max(Math.abs(output), 0.2);
            }

            spinTheLaunchers(targetRPM, outL, outR);

            manip.setPower(output);
            telemetry.update();
        }

        manip.setPower(0);
        sleep(250);
        manip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double returnPID(double reference, double state) {
        double error = reference - state;
        double dt = timer.seconds();
        if(dt <= 0) dt = 0.001;
        timer.reset();

        double pOut = kP * error;
        integralSum += error * dt;
        if(Math.abs(error) > 500) integralSum = 0;
        double iOut = kI * integralSum;
        double derivative = (error - lastError) / dt;
        double dOut = kD * derivative;
        double fOut = kF * reference;
        lastError = error;
        //if(Math.abs(error) < 20) {
        //    return reference;
        //}
        return pOut + iOut + dOut + fOut;
    }

}
