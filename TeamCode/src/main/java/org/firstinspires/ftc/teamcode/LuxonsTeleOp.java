package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@Config
@TeleOp(name = "TeleOp1Player", group = "Main")
public class LuxonsTeleOp extends OpMode {
    LuxonsBaseRobot robot = new LuxonsBaseRobot();
    PinpointDrive drive;
    IMU imu;
    double speed = 1.0;
    public static int rotTarget = 0;
    public static int slidesTarget = 0;

    @Override
    public void init(){
        // Initialize hardware
        robot.init(hardwareMap);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize drivetrain
        drive = new PinpointDrive(hardwareMap, new Pose2d(new Vector2d(0,0),0));

        // Original hardware setup commented out
        // robot.claw.setPosition(0.05);
        // robot.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rotTarget = 0;
        // slidesTarget = 0;
    }

    @Override
    public void loop(){
        // Handle drivetrain movement
        movement();
        // Original arm control commented out
        // armPID();
        // armControl();
        // slidesControl();
        // clawControl();

        // Update previous gamepad state
        robot.previousGamepad1.copy(robot.currentGamepad1);
        robot.previousGamepad2.copy(robot.currentGamepad2);
        robot.currentGamepad1.copy(gamepad1);
        robot.currentGamepad2.copy(gamepad2);

        // Send all telemetry
        telemetry.addLine("=== Controller Inputs ===");
        telemetry.addData("Left Stick X", robot.currentGamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", robot.currentGamepad1.left_stick_y);
        telemetry.addData("Right Stick X", robot.currentGamepad1.right_stick_x);
        telemetry.addData("Left Trigger", robot.currentGamepad1.left_trigger);
        telemetry.addData("Right Trigger", robot.currentGamepad1.right_trigger);
        telemetry.addData("DPad Up", robot.currentGamepad1.dpad_up);
        telemetry.addData("DPad Down", robot.currentGamepad1.dpad_down);
        telemetry.addData("A Button", robot.currentGamepad1.a);
        telemetry.addData("B Button", robot.currentGamepad1.b);
        telemetry.addData("X Button", robot.currentGamepad1.x);
        telemetry.addData("Y Button", robot.currentGamepad1.y);

        telemetry.addLine("=== Drive Outputs ===");
        double x = -robot.currentGamepad1.left_stick_x;
        double y = -robot.currentGamepad1.left_stick_y;
        double rx = -robot.currentGamepad1.right_stick_x * 0.75;

        telemetry.addData("Drive X", y * speed);
        telemetry.addData("Drive Y", x * speed);
        telemetry.addData("Drive Rotation", rx * speed);

        // Current speed mode
        telemetry.addData("Speed Mode", speed);

        telemetry.update();
    }

    private void movement(){
        // Get joystick inputs
        double x = -robot.currentGamepad1.left_stick_x;
        double y = -robot.currentGamepad1.left_stick_y;
        double rx = -robot.currentGamepad1.right_stick_x * 0.75;

        // Reduce speed if left trigger is pressed
        if(robot.currentGamepad1.left_trigger > 0.4) {
            speed = 0.2;
        } else {
            speed = 1.0;
        }

        // Set drivetrain powers based on input
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(y*speed, x*speed),
                        rx*speed
                )
        );
    }

    // Original arm and slides controls are commented out for testing without hardware
    // private void armControl(){ ... }
    // private void armPID(){ ... }
    // private void slidesControl(){ ... }
    // private void clawControl(){ ... }
}


//    private void armControl(){
//        double MAX_ROT = 2600;
//        if(robot.currentGamepad1.dpad_up && rotTarget <= MAX_ROT){
//            rotTarget += 15;
//        }
//
//        if(robot.currentGamepad1.dpad_down && rotTarget >= 0){
//            rotTarget -= 15;
//        }
//    }

//    public void armControl() {
//        double MAX_ROT = 2600;
//
//        if(robot.currentGamepad1.right_trigger > 0.2 && robot.slides.getCurrentPosition() > 1000){
//            rotTarget = 2000;
//        }
//        if(robot.currentGamepad1.right_trigger > 0.2 && robot.slides.getCurrentPosition() < 1000){
//            rotTarget = 0;
//        }
//
//        if(robot.currentGamepad1.dpad_up && robot.arm.getCurrentPosition() < MAX_ROT) {
//            robot.arm.setPower(0.7);
//        } else if (robot.currentGamepad1.dpad_down && robot.arm.getCurrentPosition() > 0){
//            robot.arm.setPower(-0.7);
//        } else if (robot.previousGamepad1.dpad_up && !robot.currentGamepad1.dpad_up){
//            rotTarget = robot.arm.getCurrentPosition();
//        } else if (robot.previousGamepad1.dpad_down && !robot.currentGamepad1.dpad_down){
//            rotTarget = robot.arm.getCurrentPosition();
//        }
//        else {
//            double p = 0.009, i = 0, d = 0.00064, f = 0, ticks_in_degree = 537.7 / 360.0;
//            PIDController controller = new PIDController(p,i,d);
//            int armPos = robot.arm.getCurrentPosition();
//            double pid = controller.calculate(armPos, Math.max(0, Math.min(MAX_ROT,rotTarget)));
//            double ff = Math.cos(Math.toRadians(Math.max(0, Math.min(MAX_ROT,rotTarget)) / ticks_in_degree)) * f;
//            double power = pid + ff;
//            robot.arm.setPower(power);
//        }
//        telemetry.addData("targetArmPos", rotTarget);
//        telemetry.addData("armPower", robot.arm.getPower());
//        telemetry.addData("armPos ", robot.arm.getCurrentPosition());
//
//    }

//    private void armPID(){
//        double p = 0.02, i = 0, d = 0.00064, f = 0, ticks_in_degree = 537.7 / 360.0, MAX_ROT =  2600;
//        PIDController controller = new PIDController(p,i,d);
//        int armPos = robot.arm.getCurrentPosition();
//        double pid = controller.calculate(armPos, Math.max(0, Math.min(MAX_ROT,rotTarget)));
//        double ff = Math.cos(Math.toRadians(Math.max(0, Math.min(MAX_ROT,rotTarget)) / ticks_in_degree)) * f;
//        double power = pid + ff;
//        robot.arm.setPower(power);
//
//        telemetry.addData("armPos ", armPos);
//        telemetry.addData("armTarget ", rotTarget);
//        telemetry.addData("armPower", power);
//    }

//    public void slidesControl() {
//        double MAX_EXTENSION = 3200;
//
//        if(robot.currentGamepad1.right_trigger > 0.2 && robot.arm.getCurrentPosition() > 1700){
//            slidesTarget = 0;
//        }
//        if(robot.currentGamepad1.x) {
//            robot.slides.setPower(0.75);
//        } else if (robot.currentGamepad1.y  && robot.slides.getCurrentPosition() < MAX_EXTENSION){
//            robot.slides.setPower(-0.75);
//        } else if (robot.previousGamepad1.x && !robot.currentGamepad1.x){
//            slidesTarget = robot.slides.getCurrentPosition();
//        } else if (robot.previousGamepad1.y && !robot.currentGamepad1.y){
//            slidesTarget = robot.slides.getCurrentPosition();
//        }
//        else {
////            robot.slides.setPower(0);
//            double p = 0.006, i = 0, d = 0.0001, f = 0, ticks_in_degree = 537.7 / 360.0;
//            PIDController controller = new PIDController(p,i,d);
//            int slidesPos = robot.slides.getCurrentPosition();
//            double pid = controller.calculate(slidesPos, Math.max(0, Math.min(MAX_EXTENSION,slidesTarget)));
//            double ff = Math.cos(Math.toRadians(Math.max(0, Math.min(MAX_EXTENSION,slidesTarget)) / ticks_in_degree)) * f;
//            double power = pid + ff;
//            robot.slides.setPower(-power);
//        }
//        telemetry.addData("targetSlidesPos", slidesTarget);
//        telemetry.addData("slidePower", robot.slides.getPower());
//        telemetry.addData("slidePos ", robot.slides.getCurrentPosition());
//
//    }

//    public void clawControl() {
//        if(robot.currentGamepad1.a) {
//            robot.claw.setPosition(0.28);
//        }
//        else if (robot.currentGamepad1.b){
//            robot.claw.setPosition(0.10);
//        }
//        telemetry.addData("clawPos", robot.claw.getPosition());
//    }
