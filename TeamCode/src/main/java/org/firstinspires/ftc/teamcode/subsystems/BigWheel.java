package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class BigWheel {

    private final double kp = 1.1; // Proportional gain
    private final double ki = 0; // Integral gain
    private final double kd = 0; // Derivative gain

    private final double gearRatio = 2; // my gears be ratio-ed

    public PIDController PID = new PIDController(kp, ki, kd);
    private final double encoderPPR = 537.7 / gearRatio; // 1 motor to 2 wheel

    // next open slot to insert into
    public int targetSlot = 1;
    public boolean isMoving = false;

    public DcMotor Motor;
    public NormalizedColorSensor colorSensor;


    public double positionOffset;
    public double Target = 0;
    public double posTolerance = 5; // counts

    // target colors (normalized)
    final float[] purpleTarget = {0.0047f, 0.0063f, 0.0068f};
    final float[] greenTarget = {0.0038f, 0.0066f, 0.0042f};

    final float[] orangeTarget = {0.0038f, 0.0066f, 0.0042f, 0.5f};///R,G,B,TOLERANCE

    final float colorTolerance = 0.05f;

    Telemetry telemetry;

    String[] index = {"/", "X", "/", "X", "/", "X"}; // this starts on the orange, i shifted it over 1 a bit ago

    public BigWheel(DcMotor Motor, NormalizedColorSensor colorSensor, Telemetry telemetry) {
        this.Motor = Motor;
        this.colorSensor = colorSensor;
        this.telemetry = telemetry;
        this.positionOffset = (Motor.getCurrentPosition() / encoderPPR);
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PID.setTarget(positionOffset);
        isMoving = false;
    }

    // === Utility ===
    public double getPos() {
        return Motor.getCurrentPosition() - positionOffset;
    }


    public void goToCurTarget() {
        double curPos = getPos() / encoderPPR;
        double motorOut = PID.calculateOutput(curPos);

        double error = abs(PID.getError(curPos));



        isMoving = true;
        if (abs(motorOut) >= 0.005){
            Motor.setPower(motorOut);
        }
        else {
            Motor.setPower(0);
            isMoving = false;
        }

//        if (error > posTolerance / encoderPPR) {
//            Motor.setPower(motorOut);
//            isMoving = true;
//        } else {
//            Motor.setPower(0);
//            if (isMoving) {
//                telemetry.addLine("Arrived at target slot: " + targetSlot);
//                telemetry.update();
//            }
//            isMoving = false;
//        }
    }

    private double normalizeShortestPath(double current, double target) {
        double diff = target - current;
        diff -= Math.floor(diff + 0.5);
        return current + diff;
    }

    private void moveToSlot(int slotIndex) {
        double currentRev = getPos() / encoderPPR;
        double rawTarget = slotIndex * (1.0 / 6.0);
        Target = normalizeShortestPath(currentRev, rawTarget);
        PID.setTarget(Target);// + positionOffset);
        isMoving = true;
    }

    // === Color Checking ===
    public double ballCheck() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float r = colors.red, g = colors.green, b = colors.blue;

        float purpleDiff = abs(r - purpleTarget[0]) + abs(g - purpleTarget[1]) + abs(b - purpleTarget[2]);
        float greenDiff = abs(r - greenTarget[0]) + abs(g - greenTarget[1]) + abs(b - greenTarget[2]);

        if (purpleDiff < colorTolerance && purpleDiff < greenDiff) return 1;
        else if (greenDiff < colorTolerance && greenDiff < purpleDiff) return 2;
        else return 0;
    }

    public void telemetryColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("Raw Red", colors.red);
        telemetry.addData("Raw Green", colors.green);
        telemetry.addData("Raw Blue", colors.blue);

        double detected = ballCheck();
        if (detected == 1) telemetry.addLine("Detected: PURPLE ball");
        else if (detected == 2) telemetry.addLine("Detected: GREEN ball");
        else telemetry.addLine("Detected: None");
    }

    // === SLOT LOGIC ===

    public void launchBall(String colorCode) {
        if (!colorCode.equals("P") && !colorCode.equals("G")) {
            telemetry.addLine("Invalid color input! Use 'P' or 'G'.");
            telemetry.update();
            return;
        }

        int ballPos = -1;
        for (int i = 0; i < index.length; i++) {
            if (index[i].equals(colorCode)) {
                ballPos = i;
                break;
            }
        }

        if (ballPos == -1) {
            telemetry.addLine("No " + colorCode + " ball found.");
            telemetry.update();
            return;
        }

        //When the launcher is launching the ball needs to be in the launching position to get launched by the launcher!
        int launchSlot = (ballPos + 3) % index.length;


        if (ballPos != -1) {
            moveToSlot(launchSlot);
            this.targetSlot = launchSlot;
            telemetry.addData("Moved to launching spot slot", launchSlot);
        } else {
            telemetry.addLine("All slots emptied");
        }
        //index[ballPos] = "X";

        telemetry.addLine(colorCode + " ball moved to launch (pos 4) and cleared.");
        telemetry.update();
    }

    /**
     * Non-blocking intake that only acts if not moving.
     * Returns true if a ball was accepted and moved to next slot.
     */
    public boolean tryIntake() {
        if (isMoving) {
            telemetry.addLine("Currently moving, skipping intake...");
            telemetry.update();
            return false;
        }

        double detected = ballCheck();
        if (detected == 0) {
            telemetry.addLine("No ball detected on tryIntake()");
            telemetry.update();
            return false;
        }

        String inputBall = (detected == 1) ? "P" : "G";

//        if (!index[this.targetSlot].equals("X")) {
//            telemetry.addLine("Slot " + this.targetSlot + " not open!");
//            telemetry.update();
//            return false;
//        }

        // Insert ball
        if (index[this.targetSlot] != "/") {
            index[this.targetSlot] = inputBall;
            telemetry.addData("Added", inputBall + " to slot " + this.targetSlot);
        }

        // Find next open slot
        int nextOpen = -1;
        for (int i = 0; i < index.length; i++) {
            if (index[i].equals("X")) {
                nextOpen = i;
                break;
            }
        }

        nextOpen = (nextOpen + 3) % index.length;

        if (nextOpen != -1) {
            moveToSlot(nextOpen);
            this.targetSlot = nextOpen;
            telemetry.addData("Moved to next open slot", nextOpen);
        } else {
            telemetry.addLine("All slots full!");
        }

        telemetry.update();
        return true;
    }


    // PANIC CODE
    public void moveRight(){
        int rightSlot = (targetSlot + 1) % index.length;
        moveToSlot(rightSlot);
        this.targetSlot = rightSlot;
    }

    public void moveLeft(){
        int leftSlot = (targetSlot - 1) % index.length;
        moveToSlot(leftSlot);
        this.targetSlot = leftSlot;
    }

    //HOMING CODE

    public boolean tryHome(){
        Motor.setPower(0.3);

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float r = colors.red, g = colors.green, b = colors.blue;

        float orangeDiff = abs(r - orangeTarget[0]) + abs(g - orangeTarget[1]) + abs(b - orangeTarget[2]);

        if (orangeDiff > orangeTarget[3]) {
            isMoving = true;
            return false;
        }
        else{// zero out all vars and return true to show homing success.
            positionOffset = Motor.getCurrentPosition() / encoderPPR;
            PID.setTarget(positionOffset);
            targetSlot = 0;
            moveToSlot(0);
            isMoving = false;
            return true;
        }
    }
}
