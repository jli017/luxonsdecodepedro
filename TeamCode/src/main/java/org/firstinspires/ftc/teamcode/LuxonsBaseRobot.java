package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LuxonsBaseRobot {
    //Motors
    public DcMotorEx leftFront, leftBack, rightBack, rightFront, slides, arm, hang;

    //Servos
  //  public ServoImplEx claw, hangServoL, hangServoR, miniArm, wrist;

    //public CRServoImplEx activeLeft, activeRight;

    //Gamepads

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();

    HardwareMap hardwareMap = null;

    public LuxonsBaseRobot() {

    }

    public void init(HardwareMap ahwMap){
        hardwareMap = ahwMap;

        //Wheel Initialization
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

        //Wheel Direction
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Wheel Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Slide Declaration
        //slides = hardwareMap.get(DcMotorEx.class, "slides");

        //Slide Direction
        //slides.setDirection(DcMotorSimple.Direction.FORWARD);

        //Slide Zero Power Behavior
        //slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Arm Declaration
        //arm = hardwareMap.get(DcMotorEx.class, "arm_motor");

        //Arm Direction
        //arm.setDirection(DcMotorEx.Direction.FORWARD);

        //Hang Declaration
        //hang = hardwareMap.get(DcMotorEx.class, "hang");

        //Claw Declaration
        //
    }
}
