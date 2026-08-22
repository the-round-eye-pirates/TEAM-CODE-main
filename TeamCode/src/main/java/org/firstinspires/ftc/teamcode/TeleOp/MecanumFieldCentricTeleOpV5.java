package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@TeleOp(name = "Mecanum Field-Centric TeleOp v5", group = "TeleOp")
public class MecanumFieldCentricTeleOpV5 extends LinearOpMode {

    @Configurable
    public static class Tuning {
        public static double DRIVE_SPEED = 1.0;
        public static double SLOW_MODE_MULTIPLIER = 0.35;
        public static double ROTATION_SPEED = 0.8;
        public static double INPUT_DEADZONE = 0.05;
        public static boolean FIELD_CENTRIC = true;
        public static double TEST_MODE_POWER = 0.3;

        public static double ACCEL_SLEW_PER_SEC = 6.0;

        public static boolean HEADING_HOLD_ENABLED = true;
        public static double HEADING_HOLD_KP = 0.8;
        public static double HEADING_HOLD_MAX_POWER = 0.25;

        public static double TRIM_FL = 1.0;
        public static double TRIM_FR = 1.0;
        public static double TRIM_BL = 1.0;
        public static double TRIM_BR = 1.0;

        public static double APPROACH_TRANSLATION_KP = 0.006;
        public static double APPROACH_HEADING_KP = 1.2;

        public static boolean RETURN_TO_SAVE_ENABLED = true;
        public static double RETURN_MAX_POWER = 0.6;
        public static double RETURN_MIN_POWER = 0.12;
        public static double RETURN_POSITION_TOLERANCE_MM = 20.0;
        public static double RETURN_HEADING_TOLERANCE_DEG = 2.0;
        public static double RETURN_REENGAGE_POSITION_MM = 45.0;
        public static double RETURN_REENGAGE_HEADING_DEG = 5.0;
        public static double RETURN_TIMEOUT_SEC = 4.0;

        public static boolean AUTO_START_ENABLED = false;
        public static double AUTO_START_TARGET_X_MM = 0;
        public static double AUTO_START_TARGET_Y_MM = 0;
        public static double AUTO_START_TARGET_HEADING_DEG = 0;
        public static double AUTO_START_TIMEOUT_SEC = 3.0;
        public static boolean AUTO_START_RUMBLE_ON_HANDOFF = true;

        public static boolean PUSH_CORRECTION_ENABLED = true;
        public static double PUSH_MAX_POWER = 0.35;
        public static double PUSH_MIN_POWER = 0.10;
        public static double PUSH_POSITION_TOLERANCE_MM = 15.0;
        public static double PUSH_REENGAGE_POSITION_MM = 35.0;
        public static double PUSH_HEADING_TOLERANCE_DEG = 2.0;
        public static double PUSH_REENGAGE_HEADING_DEG = 5.0;
        public static double PUSH_CORRECTION_TIMEOUT_SEC = 1.5;

        public static double TICKS_PER_MOTOR_REV = 537.7;
        public static double DRIVE_GEAR_REDUCTION = 1.0;
        public static double WHEEL_DIAMETER_MM = 104;
    }

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private TelemetryManager panelsTelemetry;

    private boolean slowMode = false;
    private boolean testMode = false;
    private boolean lastLeftBumper = false;
    private boolean lastOptionsButton = false;
    private boolean lastDpadUp = false;
    private boolean lastStart = false;
    private boolean lastYButton = false;

    private boolean savedPositionValid = false;
    private double savedX_mm = 0, savedY_mm = 0, savedHeadingDeg = 0;

    private double posX_mm = 0, posY_mm = 0;
    private double lastFLTicks = 0, lastFRTicks = 0, lastBLTicks = 0, lastBRTicks = 0;

    private double lastFLPower = 0, lastFRPower = 0, lastBLPower = 0, lastBRPower = 0;
    private final ElapsedTime driveSlewTimer = new ElapsedTime();

    private boolean wasReturning = false;
    private boolean returnArrived = false;
    private final ElapsedTime returnTimer = new ElapsedTime();

    private boolean autoStartActive = false;
    private boolean autoStartArrived = false;
    private final ElapsedTime autoStartTimer = new ElapsedTime();

    private double holdX_mm = 0, holdY_mm = 0, holdHeadingDeg = 0;
    private boolean holdAnchorValid = false;
    private boolean holdArrived = true;
    private final ElapsedTime holdTimer = new ElapsedTime();

    private double headingLockTargetDeg = 0;
    private boolean headingLockValid = false;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        backLeft = hardwareMap.get(DcMotorEx.class, "lr");
        backRight = hardwareMap.get(DcMotorEx.class, "rr");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized. Waiting for start.");
        panelsTelemetry.debug("Auto-Start Enabled", Tuning.AUTO_START_ENABLED);
        panelsTelemetry.debug("Push Correction Enabled", Tuning.PUSH_CORRECTION_ENABLED);
        panelsTelemetry.update(telemetry);

        waitForStart();

        imu.resetYaw();
        lastFLTicks = frontLeft.getCurrentPosition();
        lastFRTicks = frontRight.getCurrentPosition();
        lastBLTicks = backLeft.getCurrentPosition();
        lastBRTicks = backRight.getCurrentPosition();
        driveSlewTimer.reset();

        autoStartActive = Tuning.AUTO_START_ENABLED;
        autoStartArrived = false;
        autoStartTimer.reset();

        holdAnchorValid = false;
        holdArrived = true;
        headingLockTargetDeg = 0;
        headingLockValid = true;

        while (opModeIsActive()) {
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastDpadUp && !testMode) {
                testMode = true;
                lastFLPower = 0;
                lastFRPower = 0;
                lastBLPower = 0;
                lastBRPower = 0;
            }
            lastDpadUp = dpadUp;

            boolean startBtn = gamepad1.start;
            if (startBtn && !lastStart) {
                testMode = false;
                driveSlewTimer.reset();
            }
            lastStart = startBtn;

            if (testMode) {
                runWheelTestMode();
                continue;
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double curHeadingRad = orientation.getYaw(AngleUnit.RADIANS);
            double curHeadingDeg = orientation.getYaw(AngleUnit.DEGREES);

            updateOdometry(curHeadingRad);

            if (autoStartActive) {
                headingLockValid = false;
                driveAutoStart(curHeadingRad, curHeadingDeg);

                boolean rightBumper = gamepad1.right_bumper;
                boolean manualOverride =
                        Math.abs(gamepad1.left_stick_x) > Tuning.INPUT_DEADZONE
                                || Math.abs(gamepad1.left_stick_y) > Tuning.INPUT_DEADZONE
                                || Math.abs(gamepad1.right_stick_x) > Tuning.INPUT_DEADZONE;
                boolean timedOut = autoStartTimer.seconds() > Tuning.AUTO_START_TIMEOUT_SEC;

                if (rightBumper || manualOverride || autoStartArrived || timedOut) {
                    autoStartActive = false;
                    if (Tuning.AUTO_START_RUMBLE_ON_HANDOFF) {
                        gamepad1.rumble(200);
                    }
                }

                panelsTelemetry.debug("Mode", "AUTO START");
                panelsTelemetry.debug("Auto-Start Arrived", autoStartArrived);
                panelsTelemetry.debug("Auto-Start Timed Out", timedOut);
                panelsTelemetry.debug("Heading (deg)", curHeadingDeg);
                panelsTelemetry.debug("Est Pos X (mm)", posX_mm);
                panelsTelemetry.debug("Est Pos Y (mm)", posY_mm);
                panelsTelemetry.update(telemetry);
                continue;
            }

            boolean optionsButton = gamepad1.options;
            if (optionsButton && !lastOptionsButton) {
                imu.resetYaw();
                posX_mm = 0;
                posY_mm = 0;
                savedPositionValid = false;
                holdAnchorValid = false;
                headingLockTargetDeg = 0;
                headingLockValid = true;
            }
            lastOptionsButton = optionsButton;

            boolean yButton = gamepad1.y;
            if (yButton && !lastYButton) {
                savedX_mm = posX_mm;
                savedY_mm = posY_mm;
                savedHeadingDeg = curHeadingDeg;
                savedPositionValid = true;
            }
            lastYButton = yButton;

            boolean returning =
                    Tuning.RETURN_TO_SAVE_ENABLED
                            && gamepad1.a
                            && savedPositionValid;

            if (returning && !wasReturning) {
                returnTimer.reset();
                returnArrived = false;
            }
            wasReturning = returning;

            boolean driverActive =
                    Math.abs(gamepad1.left_stick_x) > Tuning.INPUT_DEADZONE
                            || Math.abs(gamepad1.left_stick_y) > Tuning.INPUT_DEADZONE
                            || Math.abs(gamepad1.right_stick_x) > Tuning.INPUT_DEADZONE;

            String mode;

            if (returning) {
                driveReturnToSave(curHeadingRad, curHeadingDeg);
                holdAnchorValid = false;
                headingLockValid = false;
                mode = "RETURN TO SAVE";
            } else if (driverActive || !Tuning.PUSH_CORRECTION_ENABLED) {
                driveManual(curHeadingRad, curHeadingDeg);
                holdAnchorValid = false;
                mode = "DRIVE";
            } else if (!holdAnchorValid) {
                holdX_mm = posX_mm;
                holdY_mm = posY_mm;
                holdHeadingDeg = curHeadingDeg;
                holdAnchorValid = true;
                holdArrived = true;
                headingLockValid = false;
                applyDrivePowers(0, 0, 0, 1.0);
                mode = "IDLE (anchored)";
            } else {
                headingLockValid = false;
                drivePushCorrection(curHeadingRad, curHeadingDeg);
                mode = holdArrived ? "IDLE (anchored)" : "PUSH CORRECTION";
            }

            panelsTelemetry.debug("Mode", mode);

            if (returning) {
                panelsTelemetry.debug("Return Arrived", returnArrived);
                panelsTelemetry.debug(
                        "Return Timed Out",
                        returnTimer.seconds() > Tuning.RETURN_TIMEOUT_SEC
                );
            }

            panelsTelemetry.debug("Slow Mode", slowMode);
            panelsTelemetry.debug("Field Centric", Tuning.FIELD_CENTRIC);
            panelsTelemetry.debug("Return Enabled", Tuning.RETURN_TO_SAVE_ENABLED);
            panelsTelemetry.debug("Push Correction Enabled", Tuning.PUSH_CORRECTION_ENABLED);
            panelsTelemetry.debug("Heading Hold Enabled", Tuning.HEADING_HOLD_ENABLED);
            panelsTelemetry.debug("Heading Lock Target (deg)", headingLockTargetDeg);
            panelsTelemetry.debug("Heading (deg)", curHeadingDeg);
            panelsTelemetry.debug("Est Pos X (mm)", posX_mm);
            panelsTelemetry.debug("Est Pos Y (mm)", posY_mm);
            panelsTelemetry.debug("Saved Valid", savedPositionValid);

            if (savedPositionValid) {
                panelsTelemetry.debug("Saved X (mm)", savedX_mm);
                panelsTelemetry.debug("Saved Y (mm)", savedY_mm);
                panelsTelemetry.debug("Saved Heading (deg)", savedHeadingDeg);

                double dist = Math.hypot(
                        savedX_mm - posX_mm,
                        savedY_mm - posY_mm
                );

                panelsTelemetry.debug("Dist To Saved (mm)", dist);
            }

            panelsTelemetry.update(telemetry);
        }
    }

    private void updateOdometry(double headingRad) {
        double flTicks = frontLeft.getCurrentPosition();
        double frTicks = frontRight.getCurrentPosition();
        double blTicks = backLeft.getCurrentPosition();
        double brTicks = backRight.getCurrentPosition();

        double dFL = flTicks - lastFLTicks;
        double dFR = frTicks - lastFRTicks;
        double dBL = blTicks - lastBLTicks;
        double dBR = brTicks - lastBRTicks;

        lastFLTicks = flTicks;
        lastFRTicks = frTicks;
        lastBLTicks = blTicks;
        lastBRTicks = brTicks;

        double ticksPerMM =
                (Tuning.TICKS_PER_MOTOR_REV * Tuning.DRIVE_GEAR_REDUCTION)
                        / (Tuning.WHEEL_DIAMETER_MM * Math.PI);

        if (ticksPerMM <= 0) {
            return;
        }

        double dFL_mm = dFL / ticksPerMM;
        double dFR_mm = dFR / ticksPerMM;
        double dBL_mm = dBL / ticksPerMM;
        double dBR_mm = dBR / ticksPerMM;

        double axialDelta_mm =
                (dFL_mm + dFR_mm + dBL_mm + dBR_mm) / 4.0;

        double lateralDelta_mm =
                (dFL_mm - dFR_mm - dBL_mm + dBR_mm) / 4.0;

        double fieldDX =
                lateralDelta_mm * Math.cos(headingRad)
                        - axialDelta_mm * Math.sin(headingRad);

        double fieldDY =
                lateralDelta_mm * Math.sin(headingRad)
                        + axialDelta_mm * Math.cos(headingRad);

        posX_mm += fieldDX;
        posY_mm += fieldDY;
    }

    private void driveManual(double curHeadingRad, double curHeadingDeg) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (Math.abs(y) < Tuning.INPUT_DEADZONE) y = 0;
        if (Math.abs(x) < Tuning.INPUT_DEADZONE) x = 0;
        if (Math.abs(rx) < Tuning.INPUT_DEADZONE) rx = 0;

        boolean leftBumper = gamepad1.left_bumper;

        if (leftBumper && !lastLeftBumper) {
            slowMode = !slowMode;
        }

        lastLeftBumper = leftBumper;

        boolean rotatingManually = rx != 0;

        if (rotatingManually) {
            rx *= Tuning.ROTATION_SPEED;
            headingLockTargetDeg = curHeadingDeg;
            headingLockValid = true;
        } else if (!headingLockValid) {
            headingLockTargetDeg = curHeadingDeg;
            headingLockValid = true;
            rx = 0;
        } else if (Tuning.HEADING_HOLD_ENABLED && (y != 0 || x != 0)) {
            double headingErrorDeg =
                    normalizeAngleDeg(headingLockTargetDeg - curHeadingDeg);

            double headingErrorRad = Math.toRadians(headingErrorDeg);

            rx = clamp(
                    headingErrorRad * Tuning.HEADING_HOLD_KP,
                    -Tuning.HEADING_HOLD_MAX_POWER,
                    Tuning.HEADING_HOLD_MAX_POWER
            );
        } else {
            headingLockTargetDeg = curHeadingDeg;
        }

        if (Tuning.FIELD_CENTRIC) {
            double[] rotated =
                    rotateFieldToRobot(x, y, curHeadingRad);

            x = rotated[0];
            y = rotated[1];
        }

        double speedMultiplier =
                Tuning.DRIVE_SPEED
                        * (slowMode ? Tuning.SLOW_MODE_MULTIPLIER : 1.0);

        applyDrivePowers(y, x, rx, speedMultiplier);
    }

    private void driveReturnToSave(double curHeadingRad, double curHeadingDeg) {
        boolean[] arrivedRef = {returnArrived};

        double[] p = computeApproachPowers(
                posX_mm,
                posY_mm,
                curHeadingDeg,
                curHeadingRad,
                savedX_mm,
                savedY_mm,
                savedHeadingDeg,
                arrivedRef,
                returnTimer,
                Tuning.RETURN_POSITION_TOLERANCE_MM,
                Tuning.RETURN_REENGAGE_POSITION_MM,
                Tuning.RETURN_HEADING_TOLERANCE_DEG,
                Tuning.RETURN_REENGAGE_HEADING_DEG,
                Tuning.RETURN_TIMEOUT_SEC,
                Tuning.RETURN_MAX_POWER,
                Tuning.RETURN_MIN_POWER
        );

        returnArrived = arrivedRef[0];

        applyDrivePowers(p[0], p[1], p[2], 1.0);
    }

    private void driveAutoStart(double curHeadingRad, double curHeadingDeg) {
        boolean[] arrivedRef = {autoStartArrived};

        double[] p = computeApproachPowers(
                posX_mm,
                posY_mm,
                curHeadingDeg,
                curHeadingRad,
                Tuning.AUTO_START_TARGET_X_MM,
                Tuning.AUTO_START_TARGET_Y_MM,
                Tuning.AUTO_START_TARGET_HEADING_DEG,
                arrivedRef,
                autoStartTimer,
                Tuning.RETURN_POSITION_TOLERANCE_MM,
                Tuning.RETURN_REENGAGE_POSITION_MM,
                Tuning.RETURN_HEADING_TOLERANCE_DEG,
                Tuning.RETURN_REENGAGE_HEADING_DEG,
                Tuning.AUTO_START_TIMEOUT_SEC,
                Tuning.RETURN_MAX_POWER,
                Tuning.RETURN_MIN_POWER
        );

        autoStartArrived = arrivedRef[0];

        applyDrivePowers(p[0], p[1], p[2], 1.0);
    }

    private void drivePushCorrection(double curHeadingRad, double curHeadingDeg) {
        if (holdArrived) {
            holdTimer.reset();
        }

        boolean[] arrivedRef = {holdArrived};

        double[] p = computeApproachPowers(
                posX_mm,
                posY_mm,
                curHeadingDeg,
                curHeadingRad,
                holdX_mm,
                holdY_mm,
                holdHeadingDeg,
                arrivedRef,
                holdTimer,
                Tuning.PUSH_POSITION_TOLERANCE_MM,
                Tuning.PUSH_REENGAGE_POSITION_MM,
                Tuning.PUSH_HEADING_TOLERANCE_DEG,
                Tuning.PUSH_REENGAGE_HEADING_DEG,
                Tuning.PUSH_CORRECTION_TIMEOUT_SEC,
                Tuning.PUSH_MAX_POWER,
                Tuning.PUSH_MIN_POWER
        );

        holdArrived = arrivedRef[0];

        applyDrivePowers(p[0], p[1], p[2], 1.0);
    }

    private double[] computeApproachPowers(
            double curX,
            double curY,
            double curHeadingDeg,
            double curHeadingRad,
            double targetX,
            double targetY,
            double targetHeadingDeg,
            boolean[] arrivedState,
            ElapsedTime timer,
            double posTolerance,
            double posReengage,
            double headingToleranceDeg,
            double headingReengageDeg,
            double timeoutSec,
            double maxPower,
            double minPower
    ) {
        double dx = targetX - curX;
        double dy = targetY - curY;
        double distance = Math.hypot(dx, dy);

        double headingErrorDeg =
                normalizeAngleDeg(targetHeadingDeg - curHeadingDeg);

        boolean tightlyWithin =
                distance < posTolerance
                        && Math.abs(headingErrorDeg) < headingToleranceDeg;

        boolean stillLooselyWithin =
                distance < posReengage
                        && Math.abs(headingErrorDeg) < headingReengageDeg;

        if (tightlyWithin) {
            arrivedState[0] = true;
        } else if (!stillLooselyWithin) {
            arrivedState[0] = false;
        }

        boolean timedOut = timer.seconds() > timeoutSec;
        boolean arrived = arrivedState[0] || timedOut;

        double fieldX;
        double fieldY;
        double rx;

        if (arrived) {
            fieldX = 0;
            fieldY = 0;
            rx = 0;
        } else {
            fieldX = applyMinPower(
                    clamp(
                            dx * Tuning.APPROACH_TRANSLATION_KP,
                            -maxPower,
                            maxPower
                    ),
                    minPower
            );

            fieldY = applyMinPower(
                    clamp(
                            dy * Tuning.APPROACH_TRANSLATION_KP,
                            -maxPower,
                            maxPower
                    ),
                    minPower
            );

            double headingErrorRad =
                    Math.toRadians(headingErrorDeg);

            rx = applyMinPower(
                    clamp(
                            headingErrorRad * Tuning.APPROACH_HEADING_KP,
                            -maxPower,
                            maxPower
                    ),
                    minPower
            );
        }

        double[] rotated =
                rotateFieldToRobot(fieldX, fieldY, curHeadingRad);

        return new double[]{rotated[1], rotated[0], rx};
    }

    private double applyMinPower(double power, double minPower) {
        if (power == 0) {
            return 0;
        }

        if (Math.abs(power) < minPower) {
            return Math.copySign(minPower, power);
        }

        return power;
    }

    private double[] rotateFieldToRobot(
            double fx,
            double fy,
            double headingRad
    ) {
        double cosA = Math.cos(-headingRad);
        double sinA = Math.sin(-headingRad);

        double rotX = fx * cosA - fy * sinA;
        double rotY = fx * sinA + fy * cosA;

        return new double[]{rotX, rotY};
    }

    private void applyDrivePowers(
            double y,
            double x,
            double rx,
            double speedMultiplier
    ) {
        double denominator =
                Math.max(
                        Math.abs(y) + Math.abs(x) + Math.abs(rx),
                        1.0
                );

        double frontLeftPower =
                (y + x + rx) / denominator;

        double backLeftPower =
                (y - x + rx) / denominator;

        double frontRightPower =
                (y - x - rx) / denominator;

        double backRightPower =
                (y + x - rx) / denominator;

        double dt = driveSlewTimer.seconds();
        driveSlewTimer.reset();

        if (dt <= 0) {
            dt = 1e-3;
        }

        double maxChange =
                Tuning.ACCEL_SLEW_PER_SEC * dt;

        double flTarget =
                frontLeftPower
                        * speedMultiplier
                        * Tuning.TRIM_FL;

        double frTarget =
                frontRightPower
                        * speedMultiplier
                        * Tuning.TRIM_FR;

        double blTarget =
                backLeftPower
                        * speedMultiplier
                        * Tuning.TRIM_BL;

        double brTarget =
                backRightPower
                        * speedMultiplier
                        * Tuning.TRIM_BR;

        lastFLPower =
                slewPower(lastFLPower, flTarget, maxChange);

        lastFRPower =
                slewPower(lastFRPower, frTarget, maxChange);

        lastBLPower =
                slewPower(lastBLPower, blTarget, maxChange);

        lastBRPower =
                slewPower(lastBRPower, brTarget, maxChange);

        frontLeft.setPower(lastFLPower);
        backLeft.setPower(lastBLPower);
        frontRight.setPower(lastFRPower);
        backRight.setPower(lastBRPower);
    }

    private double slewPower(
            double last,
            double target,
            double maxChange
    ) {
        boolean directionReversed =
                Math.signum(target) != 0
                        && Math.signum(last) != 0
                        && Math.signum(target) != Math.signum(last);

        if (directionReversed) {
            target = 0;
        }

        boolean speedingUp =
                Math.abs(target) > Math.abs(last);

        if (!speedingUp) {
            return target;
        }

        double delta =
                clamp(
                        target - last,
                        -maxChange,
                        maxChange
                );

        return last + delta;
    }

    private double clamp(
            double v,
            double lo,
            double hi
    ) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double normalizeAngleDeg(double deg) {
        while (deg > 180) {
            deg -= 360;
        }

        while (deg < -180) {
            deg += 360;
        }

        return deg;
    }

    private void runWheelTestMode() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        String active = "none";

        if (gamepad1.dpad_up) {
            frontLeft.setPower(Tuning.TEST_MODE_POWER);
            active = "FL";
        } else if (gamepad1.dpad_right) {
            frontRight.setPower(Tuning.TEST_MODE_POWER);
            active = "FR";
        } else if (gamepad1.dpad_down) {
            backRight.setPower(Tuning.TEST_MODE_POWER);
            active = "BR";
        } else if (gamepad1.dpad_left) {
            backLeft.setPower(Tuning.TEST_MODE_POWER);
            active = "BL";
        }

        panelsTelemetry.debug(
                "Mode",
                "WHEEL TEST - press START to exit"
        );
        panelsTelemetry.debug("Active Wheel", active);
        panelsTelemetry.debug(
                "dpad_up=FL  dpad_right=FR  dpad_down=BR  dpad_left=BL",
                ""
        );
        panelsTelemetry.update(telemetry);
    }
}
