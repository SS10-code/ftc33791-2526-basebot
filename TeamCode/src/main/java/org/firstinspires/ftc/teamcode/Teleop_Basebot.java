package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Teleop_Basebot provides teleoperated control for the robot during driver-controlled periods.
 *
 * @author FTC Team 23070 Royal Turtles
 */
@TeleOp
public class Teleop_Basebot extends LinearOpMode {
    double direction_x, direction_y, pivot, heading;
    Project1Hardware robot;
    Gamepad gamepad;
    Gamepad lastGamepad;
    public boolean closeZone = true, farZone;
    ShooterVelocityRegression shooterVelocityRegression;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Project1Hardware();
        robot.init(hardwareMap);

        MecanumDrive drive = new MecanumDrive(robot);

        shooterVelocityRegression.addDataPoints(BotConstants.AutoShoot.distances, BotConstants.AutoShoot.velocities);

        gamepad = new Gamepad();
        lastGamepad = new Gamepad();

        waitForStart();
        robot.pinpoint.resetPosAndIMU();

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            direction_x = gamepad.left_stick_x;
            direction_y = gamepad.left_stick_y;
            pivot       = gamepad.right_stick_x * 0.8;
            heading     = robot.pinpoint.getHeading(AngleUnit.RADIANS);

            if (gamepad.cross) {
                telemetry.addData("autoAlignReady", drive.autoAlign(0.5));
            } else {
                drive.remote(direction_y, direction_x, pivot, heading);
            }

            if (gamepad.optionsWasPressed()) {
                robot.pinpoint.resetPosAndIMU();
                sleep(250);
            }

            double shootCurVel = robot.getShooterVel();

            if (gamepad.dpadUpWasPressed()) {
                closeZone = true; farZone = false;
            } else if (gamepad.dpadDownWasPressed()) {
                farZone = true; closeZone = false;
            }

            if (gamepad.triangle) {
                robot.setShooterVel(shooterVelocityRegression.predict(robot.getDistanceToTag()));
            }

            if (gamepad.rightBumperWasPressed()) {
                if (closeZone) {
                    robot.setShooterVel(1000);
                } else if (farZone) {
                    robot.setShooterVel(1200);
                }
            } else if (gamepad.leftBumperWasPressed()) {
                robot.shooterOff();
            }

            if (gamepad.squareWasPressed()) {
                robot.setIndexPos(robot.getIndexPos() + 280);
            } else if (gamepad.circle && !lastGamepad.circle) {
                robot.setIndexPos(robot.getIndexPos() - 280);
            }

            if (gamepad.right_trigger > 0.2) {
                robot.setIntake(1.0);
            } else if (gamepad.left_trigger > 0.2) {
                robot.intakeReverse();
            } else {
                robot.intakeOff();
            }

            telemetry.addData("shooterVel", robot.getShooterVel());
            telemetry.addData("distanceToTag", robot.getDistanceToTag());
            telemetry.addData("predictedVelo", shooterVelocityRegression.predict(robot.getDistanceToTag()));
            telemetry.addData("closeZone", closeZone);
            telemetry.addData("farZone", farZone);
            telemetry.addData("heading", heading);
            telemetry.addData("lShooterVelo", robot.lShooter.getVelocity());
            telemetry.addData("rShooterVelo", robot.rShooter.getVelocity());
            telemetry.update();
        }
    }
}