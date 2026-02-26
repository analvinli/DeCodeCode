package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
@Disabled
@TeleOp(name = "pidtest")

public class pidtesting extends LinearOpMode {
    //Drivetrain
    DcMotorEx RightFront;
    DcMotorEx RightRear;
    DcMotorEx LeftRear;
    DcMotorEx LeftFront;

    //Spindexer
    Servo KickerServo;
    NormalizedColorSensor IntakeSensor;
    NormalizedColorSensor LeftSensor;
    NormalizedColorSensor BackSensor;
    NormalizedColorSensor RightSensor;
    DcMotorEx SpindexerMotor;

    //Intake or Outtake
    DcMotorEx IntakeMotor;
    DcMotorEx RightFlywheelMotor;
    DcMotorEx LeftFlywheelMotor;

    int[] SpindexPos = {
            0,//shooting0
            1365,//intaking1
            2730,//shooting2
            4095,//intaking3
            5460,//shooting4
            6825,//intaking5
    };
    ElapsedTime KickerTimer = new ElapsedTime();
    int CurrentSpindexerPos;
    static int ticksPerRevolution = 8192;
    boolean FirstShooterLoop = true;
    boolean IntakeActive = false;
    int IntakeState = 0;
    int KickerState = 0;

    public void runOpMode() {
        //--------------------
        //HARDWARE MAP
        //--------------------
        //Drivetrain
        RightFront = hardwareMap.get(DcMotorEx.class, "fr");//
        RightRear = hardwareMap.get(DcMotorEx.class, "br");//
        LeftRear = hardwareMap.get(DcMotorEx.class, "bl");//
        LeftFront = hardwareMap.get(DcMotorEx.class, "fl");//
        double turn;
        double forward;
        double strafe;
        double slowMulti = 1;
        //spindexer
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "spindex");//

        IntakeSensor = hardwareMap.get(NormalizedColorSensor.class, "csi");//
        LeftSensor = hardwareMap.get(NormalizedColorSensor.class, "csl");//
        BackSensor = hardwareMap.get(NormalizedColorSensor.class, "csb");//
        RightSensor = hardwareMap.get(NormalizedColorSensor.class, "csr");//

        KickerServo = hardwareMap.get(Servo.class, "kick");//
        //intake or outtake
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");//
        RightFlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelr");//
        LeftFlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheell");//

        motorConfigs();
        //RightFlywheelMotor.setVelocityPIDFCoefficients(20,0,5,0);//Flywheel Velocity PIDF
        //PIDFCoefficients SpindexPidCoefficients = new PIDFCoefficients(0.05,0,0,0);//Spindex Positional PIDF
        //SpindexerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, SpindexPidCoefficients);

        PIDFController SpindexController = new PIDFController(0.00014,0,0.00001,0);
        //SpindexController.setTolerance(50, 80);
        double raw = 0;
        double power = 0;
        int index = 0;

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.xWasPressed()){
                SpindexController.setSetPoint(SpindexController.getSetPoint()+2730);
//                SpindexController.setSetPoint(SpindexPos[index]);
//                index = (index+2)%6;
            }
            if(gamepad1.bWasPressed()){
                SpindexController.setSetPoint(SpindexController.getSetPoint()-2730);
//                index = (index-2)%6;
//                if(index<0){
//                    index+=6;
//                }
            }

            if(!SpindexController.atSetPoint()){
                raw = SpindexController.calculate(SpindexerMotor.getCurrentPosition());
                SpindexerMotor.setPower(raw+(Math.signum(raw)*0.05));

            }else{
                SpindexerMotor.setPower(0);
            }

            if(gamepad1.dpadUpWasPressed()){
                SpindexController.setP(SpindexController.getP()+0.00001);
            }
            if(gamepad1.dpadDownWasPressed()){
                SpindexController.setP(SpindexController.getP()-0.00001);
            }
            if(gamepad1.aWasPressed()){
                SpindexController.setD(SpindexController.getD()-0.00001);
            }
            if(gamepad1.yWasPressed()){
                SpindexController.setD(SpindexController.getD()+0.00001);
            }
            if(gamepad1.rightBumperWasPressed()){
                SpindexController.setI(SpindexController.getI()+0.00001);
            }
            if(gamepad1.leftBumperWasPressed()){
                SpindexController.setI(SpindexController.getI()-0.00001);
            }


            // --------------------
            // TELEMETRY
            // --------------------
            telemetry.addData("setpoint", SpindexController.getSetPoint());
            telemetry.addData("raw", raw);
            telemetry.addData("kP", SpindexController.getP());
            telemetry.addData("kI", SpindexController.getI());
            telemetry.addData("kD", SpindexController.getD());
            telemetry.addData("kF", SpindexController.getF());
            telemetry.addData("current", SpindexerMotor.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.addData("overshoot", SpindexerMotor.getCurrentPosition()-SpindexController.getSetPoint());
            telemetry.addData("at", SpindexController.atSetPoint());
            telemetry.update();
        }
        //end of teleop
    }
    public double map(double x, double inMin, double inMax, double outMin, double outMax){
        return(x-inMin)*(outMax-outMin)/(inMax-inMin) + outMin;
    }
    public void motorConfigs(){
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        RightFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFlywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        SpindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        RightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        RightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        LeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        LeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SpindexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        SpindexerMotor.setMode((DcMotorEx.RunMode.RUN_WITHOUT_ENCODER));

        RightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //SpindexerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        RightFlywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        LeftFlywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        RightFlywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        RightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public int incrementIndex(int index, int delta, int length){
        index += delta;

        if(index >= length){
            index -= length;
        }else if(index < 0){
            index += length;
        }
        return index;
    }

    public int findIndexSpindex(int target){
        for(int i = 0;i<6;i++){
            if(SpindexPos[i] == target){
                return i;
            }
        }
        return 0;
    }
//    int[] SpindexPos = {
//            0,//shooting0
//            1365,//intaking1
//            2730,//shooting2
//            4095,//intaking3
//            5460,//shooting4
//            6825,//intaking5
//    };

    public int FindClosestOuttake() {
        CurrentSpindexerPos = SpindexerMotor.getCurrentPosition()%ticksPerRevolution;
        if(CurrentSpindexerPos<0) CurrentSpindexerPos+=ticksPerRevolution;

        if(CurrentSpindexerPos >= 0 && CurrentSpindexerPos < 1365){             //after 0, first outtake
            return 0;
        }else if(CurrentSpindexerPos >= 1365 && CurrentSpindexerPos < 2730){    //before 2730, second outtake
            return 2730;
        }else if(CurrentSpindexerPos >= 2730 && CurrentSpindexerPos < 4095){    //after 2730, second outtake
            return 2730;
        }else if(CurrentSpindexerPos >= 4095 && CurrentSpindexerPos < 5460){    //before 5460, third outtake
            return 5460;
        }else if(CurrentSpindexerPos >= 5460 && CurrentSpindexerPos < 6825){    //after 5460, third outtake
            return 5460;
        }else if(CurrentSpindexerPos >= 6825){                                  //before 0, first outtake
            return 0;
        }
        return 0;
    }

    public int FindClosestIntake() {
        CurrentSpindexerPos = SpindexerMotor.getCurrentPosition()%ticksPerRevolution;
        if(CurrentSpindexerPos<0) CurrentSpindexerPos+=ticksPerRevolution;

        if(CurrentSpindexerPos >= 0 && CurrentSpindexerPos < 1365) {           //after 0, first intake
            return 1365;
        }else if(CurrentSpindexerPos >= 1365 && CurrentSpindexerPos < 2730){   //before 2730, first intake
            return 1365;
        }else if(CurrentSpindexerPos >= 2730 && CurrentSpindexerPos < 4095){   //after 2730, second intake
            return 4095;
        }else if(CurrentSpindexerPos >= 4095 && CurrentSpindexerPos < 5460){   //before 5460, second intake
            return 4095;
        }else if(CurrentSpindexerPos >= 5460 && CurrentSpindexerPos < 6825){   //after 5460, third intake
            return 6825;
        }else if(CurrentSpindexerPos >= 6825){                                 //before 0, third intake
            return 6825;
        }
        return 1365;
    }


    public int unwrapTarget(int current, int wrappedTarget) {
        return wrappedTarget;
        // int rev = Math.floorDiv(current, ticksPerRevolution);//# of revolutions made by spindexer
        // int best = rev * ticksPerRevolution + wrappedTarget;//find the closest unwrapped position

        // int up = best + ticksPerRevolution;//check adjacent revolutions, one rotation up and one rotation down
        // int down = best - ticksPerRevolution;

        // if (Math.abs(up - current) < Math.abs(best - current)) best = up;
        // if (Math.abs(down - current) < Math.abs(best - current)) best = down;

        // return best;//returns the closest unwrapped target
    }

    public void SpindexGoToWrappedPosition(int wrappedTarget) {
        CurrentSpindexerPos = SpindexerMotor.getCurrentPosition();
        int absoluteTarget = unwrapTarget(CurrentSpindexerPos, wrappedTarget);
        telemetry.addData("abs target: ", absoluteTarget);

        SpindexerMotor.setTargetPosition(absoluteTarget);
        SpindexerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //IntakeMotor.setPower(0.7);
        SpindexerMotor.setPower(0.05);//need to tune pidf
    }
    public boolean detectColor(NormalizedColorSensor colorsensor){
        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(colors.toColor());
        if(hue > 90 && hue < 170){//green
            return true;
        }else if(hue > 225 && hue < 350){//purple
            return true;
        }
        return false;
    }
}