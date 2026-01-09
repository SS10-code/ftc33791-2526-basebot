package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.BlueSidePinpoint.BlueSidePinpointConfigurables.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous()
public class BlueSidePinpoint extends OpMode {

    // =====================================================================
    // CONSTANTS
    // =====================================================================
    public static class TeleOpConstants {
        // Shooter
        public static final double CLOSE_ZONE_VELOCITY = 1300;
        public static final double FAR_ZONE_VELOCITY = 1850;
        public static final int SHOOTER_TOLERANCE = 10;

        // Intake
        public static final double INTAKE_POWER = 1.0;
        public static final double PASSIVE_INDEX_VELOCITY = 20;

        public static final double MAG_DUMP_POWER = 0.9;

        // Limelight
        public static final double LIMELIGHT_MOUNT_ANGLE = 12.0;
        public static final double LIMELIGHT_HEIGHT = 13.4;
        public static final double GOAL_HEIGHT = 38.75 - 9.25;

        // Distance sensor
        public static final double ACTIVATION_DISTANCE = 2; // INCHES
    }

    // =====================================================================
    // CONFIGURABLES
    // =====================================================================
    public static class BlueSidePinpointConfigurables {
        // Drivetrain power settings
        public static double intakePathMaxDrivetrainPower = 0.5;
        public static double defaultPathMaxDrivetrainPower = 0.8;

        // Coordinates (mirrored for blue side)
        public static double shootPositionXCoordinate = 50.000;
        public static double intakePathEndXCoordinate = 15.0;

        // Shooter velocities
        public static double shooterVelocityPreload = 1250;
        public static double shooterVelocityGoal = 1250;
        public static double shooterVelocityMid = 1250;
        public static double shooterVelocityLoadingZone = 1250;

        // P2P Drive constants
        public static double driveP = 0.05;
        public static double strafeP = 0.05;
        public static double turnP = 0.02;
        public static double positionTolerance = 1.0; // inches
        public static double headingTolerance = 2.0;  // degrees
    }

    // =====================================================================
    // INSTANCE VARIABLES
    // =====================================================================
    private ElapsedTime actiontime = new ElapsedTime();
    private ElapsedTime pathTimer = new ElapsedTime();
    double shooterTargetVel;
    private int pathState;
    private double maxPower = defaultPathMaxDrivetrainPower;

    // Hardware
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx lShooter, rShooter;
    DcMotorEx intake, index;
    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;
    DistanceSensor distance;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    // Start pose of the robot (in inches) - mirrored for blue side
    private final double startX = 25.733;
    private final double startY = 126.585;
    private final double startHeading = 180; // degrees

    // Target pose for P2P navigation
    private double targetX, targetY, targetHeading;
    private boolean navigationComplete = true;

    /**
     * This method is called once at the init of the OpMode.
     */
    @Override
    public void init() {
        initializeHardware();

        // Set starting pose on pinpoint
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.DEGREES, startHeading));
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     */
    @Override
    public void init_loop() {
        pinpoint.update();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X", getPoseX());
        telemetry.addData("Y", getPoseY());
        telemetry.addData("Heading", getPoseHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     */
    @Override
    public void start() {
        setPathState(0);
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     */
    @Override
    public void loop() {
        // Update pinpoint odometry
        pinpoint.update();

        // Run P2P navigation
        updateNavigation();

        // Run autonomous state machine
        autonomousPathUpdate();

        // --- BLINKIN ---
        if (shooterTargetVel > 500 &&
                (lShooter.getVelocity() < 50 || rShooter.getVelocity() < 50)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else if (!shooterWithinTolerance(shooterTargetVel)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        } else if (shooterWithinTolerance(Teleop_Basebot.Constants.CLOSE_ZONE_VELOCITY)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (shooterWithinTolerance(Teleop_Basebot.Constants.FAR_ZONE_VELOCITY)) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (withinDistance()) {
            setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", getPoseX());
        telemetry.addData("Y", getPoseY());
        telemetry.addData("Heading", getPoseHeading());
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Nav Complete", navigationComplete);
        telemetry.update();
    }

    // =====================================================================
    // PINPOINT POSE HELPERS
    // =====================================================================
    private double getPoseX() {
        Pose2D pose = pinpoint.getPosition();
        return pose.getX(DistanceUnit.INCH);
    }

    private double getPoseY() {
        Pose2D pose = pinpoint.getPosition();
        return pose.getY(DistanceUnit.INCH);
    }

    private double getPoseHeading() {
        Pose2D pose = pinpoint.getPosition();
        return pose.getHeading(AngleUnit.DEGREES);
    }

    // =====================================================================
    // P2P NAVIGATION
    // =====================================================================
    /**
     * Start navigating to a target pose
     */
    public void driveToPose(double x, double y, double heading) {
        targetX = x;
        targetY = y;
        targetHeading = heading;
        navigationComplete = false;
    }

    /**
     * Check if navigation is complete (robot reached target)
     */
    public boolean isNavigationComplete() {
        return navigationComplete;
    }

    /**
     * Update the P2P navigation - call this every loop
     */
    private void updateNavigation() {
        if (navigationComplete) {
            return;
        }

        double currentX = getPoseX();
        double currentY = getPoseY();
        double currentHeading = getPoseHeading();

        // Calculate errors
        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double headingError = normalizeAngle(targetHeading - currentHeading);

        // Check if we've reached the target
        if (Math.abs(xError) < positionTolerance &&
            Math.abs(yError) < positionTolerance &&
            Math.abs(headingError) < headingTolerance) {
            stopDrive();
            navigationComplete = true;
            return;
        }

        // Convert field-relative errors to robot-relative
        double headingRad = Math.toRadians(currentHeading);
        double robotXError = xError * Math.cos(headingRad) + yError * Math.sin(headingRad);
        double robotYError = -xError * Math.sin(headingRad) + yError * Math.cos(headingRad);

        // Calculate motor powers using P control
        double forward = robotXError * driveP;
        double strafe = robotYError * strafeP;
        double turn = headingError * turnP;

        // Clamp to max power
        forward = clamp(forward, -maxPower, maxPower);
        strafe = clamp(strafe, -maxPower, maxPower);
        turn = clamp(turn, -maxPower, maxPower);

        // Mecanum drive kinematics
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Normalize if any power exceeds max
        double maxMagnitude = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (maxMagnitude > maxPower) {
            fl = fl / maxMagnitude * maxPower;
            fr = fr / maxMagnitude * maxPower;
            bl = bl / maxMagnitude * maxPower;
            br = br / maxMagnitude * maxPower;
        }

        // Set motor powers
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setMaxPower(double power) {
        maxPower = power;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // =====================================================================
    // AUTONOMOUS STATE MACHINE (mirrored for blue side)
    // =====================================================================
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start to shoot position (heading -45° for blue side)
                driveToPose(shootPositionXCoordinate, 85.0, -45.0);
                setShooterVel(shooterVelocityPreload);
                setPathState(1);
                break;
            case 1:
                // Wait until robot reaches shoot position, then start intake close line
                if (isNavigationComplete()) {
                    magDump(1.0);
                    setMaxPower(intakePathMaxDrivetrainPower);
                    intakePassiveIndex();
                    driveToPose(intakePathEndXCoordinate, 85.0, 180.0);
                    setPathState(2);
                }
                break;
            case 2:
                // Wait until robot reaches intake close line position, then shoot
                if (isNavigationComplete()) {
                    intake.setPower(0);
                    setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocityGoal);
                    driveToPose(shootPositionXCoordinate, 85.0, -45.0);
                    setPathState(3);
                }
                break;
            case 3:
                // Wait until robot reaches shoot position, then prep for mid line intake
                if (isNavigationComplete()) {
                    magDump(1.0);
                    driveToPose(shootPositionXCoordinate, 60.0, 180.0);
                    setPathState(4);
                }
                break;
            case 4:
                // Wait until robot reaches prep position, then start mid line intake
                if (isNavigationComplete()) {
                    intakePassiveIndex();
                    setMaxPower(intakePathMaxDrivetrainPower);
                    driveToPose(intakePathEndXCoordinate, 60.0, 180.0);
                    setPathState(5);
                }
                break;
            case 5:
                // Wait until robot reaches intake mid line position, then shoot
                if (isNavigationComplete()) {
                    intake.setPower(0);
                    setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocityMid);
                    driveToPose(shootPositionXCoordinate, 85.0, -45.0);
                    setPathState(6);
                }
                break;
            case 6:
                // Wait until robot reaches shoot position, then prep for far line intake
                if (isNavigationComplete()) {
                    magDump(1.0);
                    driveToPose(shootPositionXCoordinate, 35.0, 180.0);
                    setPathState(7);
                }
                break;
            case 7:
                // Wait until robot reaches prep position, then start far line intake
                if (isNavigationComplete()) {
                    intakePassiveIndex();
                    setMaxPower(intakePathMaxDrivetrainPower);
                    driveToPose(intakePathEndXCoordinate, 35.0, 180.0);
                    setPathState(8);
                }
                break;
            case 8:
                // Wait until robot reaches intake far line position, then shoot
                if (isNavigationComplete()) {
                    intake.setPower(0);
                    setMaxPower(defaultPathMaxDrivetrainPower);
                    setShooterVel(shooterVelocityLoadingZone);
                    driveToPose(shootPositionXCoordinate, 85.0, -45.0);
                    setPathState(9);
                }
                break;
            case 9:
                // Wait until robot reaches shoot position, then end
                if (isNavigationComplete()) {
                    magDump(1.0);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    // =====================================================================
    // SHOOTER METHODS
    // =====================================================================
    public void setShooterVel(double vel) {
        shooterTargetVel = vel;
        lShooter.setVelocity(vel);
        rShooter.setVelocity(vel);
    }

    public int getAvgShooterVel() {
        return (int) (lShooter.getVelocity() + rShooter.getVelocity() / 2);
    }

    public boolean shooterWithinTolerance(double target) {
        return withinTolerance(getAvgShooterVel(), target, TeleOpConstants.SHOOTER_TOLERANCE);
    }

    public boolean withinTolerance(double val, double target, double tol) {
        return Math.abs(target - val) <= tol;
    }

    // =====================================================================
    // INDEX METHODS
    // =====================================================================
    public void setIndexPos(int pos) {
        index.setTargetPosition(pos);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        index.setPower(1);
    }

    public boolean withinDistance() {
        return distance.getDistance(DistanceUnit.INCH) < TeleOpConstants.ACTIVATION_DISTANCE;
    }

    public void intakePassiveIndex() {
        intake.setPower(TeleOpConstants.INTAKE_POWER);
        if (!withinDistance()) {
            index.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            index.setVelocity(TeleOpConstants.PASSIVE_INDEX_VELOCITY);
        } else {
            index.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void magDump(double seconds) {
        actiontime.reset();
        while (actiontime.seconds() < seconds) {
            index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            index.setPower(TeleOpConstants.MAG_DUMP_POWER);
            intake.setPower(TeleOpConstants.MAG_DUMP_POWER);
        }
    }

    // =====================================================================
    // LIMELIGHT METHODS
    // =====================================================================
    public double getDistanceToTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angle = Math.toRadians(TeleOpConstants.LIMELIGHT_MOUNT_ANGLE + ty);
            return (TeleOpConstants.GOAL_HEIGHT - TeleOpConstants.LIMELIGHT_HEIGHT) / Math.tan(angle);
        }
        return Double.POSITIVE_INFINITY;
    }

    void setPattern(RevBlinkinLedDriver.BlinkinPattern p) {
        pattern = p;
        blinkinLedDriver.setPattern(pattern);
    }

    // =====================================================================
    // HARDWARE INITIALIZATION
    // =====================================================================
    void initializeHardware() {
        // --- Drivetrain Motors ---
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        // Motor directions (from Constants.java)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Intake ---
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Index ---
        index = hardwareMap.get(DcMotorEx.class, "transfer");
        index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        index.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        index.setTargetPosition(0);
        index.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // --- Shooter ---
        lShooter = hardwareMap.get(DcMotorEx.class, "shooterL");
        rShooter = hardwareMap.get(DcMotorEx.class, "shooterR");
        lShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        lShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Pinpoint Odometry (from Constants.java) ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0.0, 107.31371, DistanceUnit.MM); // strafePodX, forwardPodY in mm
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // --- Limelight ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        // --- Distance Sensor ---
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        // --- Blinkin LED ---
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);
    }
}
