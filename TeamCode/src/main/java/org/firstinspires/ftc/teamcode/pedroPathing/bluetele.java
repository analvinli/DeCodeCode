package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BlueTele")
public class bluetele extends LinearOpMode {
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
    Servo HoodServo;

    GoBildaPinpointDriver pinpoint;

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
    double IntakePower = 0;
    double IntakeConst = 0.75;
    int IntakeState = 0;
    int KickerState = 0;
    int ScoreState = 0;
    boolean SpindexManual = false;
    boolean flywheelactive = false;
    boolean autoaim = false;

    private Follower follower;
    PIDFController SpindexController = new PIDFController(0.00030,0,0.0000065,0);


    public void runOpMode() {
        //follower = Constants.createFollower(hardwareMap);
        //follower.update();
        //HARDWARE MAPPING
        RightFront = hardwareMap.get(DcMotorEx.class, "fr");
        RightRear = hardwareMap.get(DcMotorEx.class, "br");
        LeftRear = hardwareMap.get(DcMotorEx.class, "bl");
        LeftFront = hardwareMap.get(DcMotorEx.class, "fl");
        double turn;
        double forward;
        double strafe;
        double slowMulti = 1;
        SpindexerMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        IntakeSensor = hardwareMap.get(NormalizedColorSensor.class, "csi");
        LeftSensor = hardwareMap.get(NormalizedColorSensor.class, "csl");
        BackSensor = hardwareMap.get(NormalizedColorSensor.class, "csb");
        RightSensor = hardwareMap.get(NormalizedColorSensor.class, "csr");
        KickerServo = hardwareMap.get(Servo.class, "kick");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        RightFlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelr");
        LeftFlywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheell");
        HoodServo = hardwareMap.get(Servo.class,"hood");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        motorConfigs();
        //pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 96.38 ,8.88, AngleUnit.DEGREES, 0));
        pinpoint.update();



        RightFlywheelMotor.setVelocityPIDFCoefficients(300,0,0,15.5);//Flywheel Velocity PIDF
        SpindexController.setTolerance(80, 100);

        double raw = 0;
        waitForStart();
        while (opModeIsActive()) {
            // --------------------
            // DRIVE
            // --------------------
            forward = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = -gamepad1.right_stick_x*.75;
            //follower.update();

            pinpoint.update();
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            double posX = pinpoint.getPosX(DistanceUnit.INCH);
            double posY = pinpoint.getPosY(DistanceUnit.INCH);
//            telemetry.addData("heading", heading);
//            telemetry.addData("posX", posX);
//            telemetry.addData("posY", posY);

            double targetX = 12.5;
            double targetY = 135;

            //double angle = Math.atan2(targetY - posY, targetX - posX);
            double angle = Math.toRadians(23);
            //telemetry.addData("angle", angle);
            double angleError = AngleUnit.normalizeRadians(angle - heading);
            if(gamepad1.dpadUpWasPressed()){
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 96.38 ,8.88, AngleUnit.DEGREES, 0));
            }


            if(!autoaim){
                RightFront.setPower((forward+strafe-turn)*slowMulti);
                LeftFront.setPower((forward-strafe+turn)*slowMulti);
                LeftRear.setPower((forward+strafe+turn)*slowMulti);
                RightRear.setPower((forward-strafe-turn)*slowMulti);
            }else if(autoaim){
                double turnPower = 1.5 * angleError;
                turnPower = Math.max(-1,Math.min(1,turnPower));
                RightFront.setPower(forward+strafe-turnPower);
                LeftFront.setPower(forward-strafe+turnPower);
                LeftRear.setPower(forward+strafe+turnPower);
                RightRear.setPower(forward-strafe-turnPower);
            }


            //SLOW DRIVE
            if(gamepad1.left_bumper){
                slowMulti = 0.25;
            }else{
                slowMulti = 1;
            }

            if(gamepad1.left_trigger>0.4){
                autoaim = true;
            }else{
                autoaim = false;
            }

            // --------------------
            // FLYWHEEL
            // --------------------
            telemetry.addData("hood", HoodServo.getPosition());
            if(gamepad2.right_trigger>0.4){//close scoring
                HoodServo.setPosition(0);
//                if(1150 - RightFlywheelMotor.getVelocity() > 200){
//                    RightFlywheelMotor.setPower(1);
//                }else{
//                    RightFlywheelMotor.setVelocity(1150);
//                }
                RightFlywheelMotor.setVelocity(1150);
                LeftFlywheelMotor.setPower(RightFlywheelMotor.getPower());
                IntakePower = IntakeConst;
                flywheelactive = true;
            }else if(gamepad2.right_bumper){//reversing flywheel
                RightFlywheelMotor.setVelocity(-800);
                LeftFlywheelMotor.setPower(RightFlywheelMotor.getPower());
                IntakePower = IntakeConst;
                flywheelactive = true;
            }else if(gamepad2.left_trigger>0.4){//far scoring
                HoodServo.setPosition(1);
                //telemetry.addData("flywheel diff: ", 1500 - RightFlywheelMotor.getVelocity());
                if(1570 - RightFlywheelMotor.getVelocity() > 150){
                    RightFlywheelMotor.setPower(1);
                }else{
                    RightFlywheelMotor.setVelocity(1570);
                }
                LeftFlywheelMotor.setPower(RightFlywheelMotor.getPower());
                IntakePower = IntakeConst;
                flywheelactive = true;
            }else{
                RightFlywheelMotor.setPower(0);
                LeftFlywheelMotor.setPower(0);
                IntakePower = 0;
                flywheelactive = false;
            }

            // --------------------
            // SPINDEX + INTAKE
            // --------------------

            //SPINDEX MOVE
            if(!SpindexController.atSetPoint() && !SpindexManual){
                raw = SpindexController.calculate(SpindexerMotor.getCurrentPosition());
                SpindexerMotor.setPower(raw+(Math.signum(raw)*0.03));
            }else if(!SpindexManual){
                SpindexerMotor.setPower(0);
            }

            //SPINDEX MACROs
            if(gamepad2.xWasPressed()){//spin to previous outtake slot
                SpindexManual = false;
                int indexOfClosest = findIndexSpindex(FindClosestOuttake());
                indexOfClosest = incrementIndex(indexOfClosest,+2,SpindexPos.length);
                SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
                IntakePower = IntakeConst;
            }else if(gamepad2.bWasPressed()){//spin to next outtake slot
                SpindexManual = false;
                int indexOfClosest = findIndexSpindex(FindClosestOuttake());
                indexOfClosest = incrementIndex(indexOfClosest,-2,SpindexPos.length);
                SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
                IntakePower = IntakeConst;
            }else if(gamepad2.aWasPressed()){//align to closest outtake [not accurate]
                SpindexManual = false;
                int indexOfClosest = findIndexSpindex(FindClosestOuttake());
                SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
                IntakePower = IntakeConst;
            }else if(gamepad2.yWasPressed()){//kicker
                KickerState = 1;
            }else if(gamepad2.dpad_right){
                SpindexManual = true;
                IntakePower = IntakeConst;
                SpindexerMotor.setPower(-0.45);
            }else if(gamepad2.dpad_left){
                SpindexManual = true;
                IntakePower = IntakeConst;
                SpindexerMotor.setPower(0.45);
            }else if(SpindexManual && !gamepad2.dpad_left && !gamepad2.dpad_right){
                SpindexerMotor.setPower(0);
                IntakePower = 0;
            }

            //INTAKE MACRO
            else if(gamepad1.right_trigger>0.4){
                //go to closest intake position
                //when detect a color in intakesensor, spin to next position
                IntakePower = IntakeConst;
                if(IntakeState == 0){
                    int x = SpindexGetAbsTarget(FindClosestIntake());
                    SpindexController.setSetPoint(x);
                    if(Math.abs(SpindexerMotor.getCurrentPosition()-x)<=500){
                        IntakeState = 1;//ready for ball
                    }
                }else if(IntakeState == 1){
                    if(detectColor(IntakeSensor)){//spin to next intake
                        int indexOfClosest = findIndexSpindex(FindClosestIntake());
                        indexOfClosest = incrementIndex(indexOfClosest,+2,SpindexPos.length);
                        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
                        IntakeState = 2;//wait until finished spinning to next chamber
                    }
                }else if(IntakeState == 2) {
                    if(SpindexWithinTolerance()){
                        IntakeState = 1;//back to ready for ball
                    }
                }
            }else if(gamepad1.right_bumper){//outtaking
                IntakePower = -IntakeConst;
            }else{//turn off intake
                ScoreState = 0;//reset state when button released
                IntakeState = 0;
            }
            IntakeMotor.setPower(IntakePower);


            //SCORING MACRO
            if(!gamepad2.dpad_up){
                ShootMacroState = 99;
            }else{
                ShootUnsorted();
                if(ShootMacroState == 99){
                    ShootMacroState = 0;
                }
            }

            if(gamepad1.aWasPressed()){//force next chamber intake
                if(IntakeState == 1){
                    int indexOfClosest = findIndexSpindex(FindClosestIntake());
                    indexOfClosest = incrementIndex(indexOfClosest,+2,SpindexPos.length);
                    SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
                    IntakeState = 2;//wait until finished spinning to next chamber
                }
            }

            // --------------------
            // KICKER STATE MACHINE
            // --------------------
            if(KickerState == 0){
                KickerServo.setPosition(0.5);//low position
            }else if(KickerState == 1){
                KickerServo.setPosition(0.2);//high position
                KickerTimer.reset();
                KickerState = 2;//ready to wait
            }else if(KickerState == 2){
                if(KickerTimer.milliseconds()>100){//time needed to move up has passed
                    KickerServo.setPosition(0.5);//low position
                    KickerTimer.reset();
                    KickerState = 3;
                }
            }else if(KickerState == 3) {
                if (KickerTimer.milliseconds() > 20) {//cooldown period has passed
                    KickerState = 0;
                }
            }


            // --------------------
            // TELEMETRY
            // --------------------
            //telemetry.addData("-----------------------------","");
            //telemetry.addData("", )

            //telemetry.addData("x", pinpoint.getPosX(DistanceUnit.INCH));
            //telemetry.addData("y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("heading", pinpoint.getHeading(AngleUnit.RADIANS));

//            telemetry.addData("COLORSENSORS", "");
            telemetry.addData("Intake ColorSensor hue: ", JavaUtil.colorToHue(IntakeSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Left ColorSensor hue: ", JavaUtil.colorToHue(LeftSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Right ColorSensor hue: ", JavaUtil.colorToHue(RightSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Back ColorSensor hue: ", JavaUtil.colorToHue(BackSensor.getNormalizedColors().toColor()));
//
//            telemetry.addData("FLYWHEELS", "");
            telemetry.addData("Right Flywheel Speed: ", RightFlywheelMotor.getVelocity());
            //telemetry.addData("Right Flywheel Power: ", RightFlywheelMotor.getPower());

//            telemetry.addData("SPINDEXER", "");
//            CurrentSpindexerPos = SpindexerMotor.getCurrentPosition();
//            telemetry.addData("Current Encoder: ", CurrentSpindexerPos);
//            telemetry.addData("Wrapped Encoder: ", CurrentSpindexerPos%ticksPerRevolution);
//            telemetry.addData("setpoint", SpindexController.getSetPoint());
//            telemetry.addData("Closest Outtake Position: ", FindClosestOuttake());
//            telemetry.addData("Closest Intake Position: ", FindClosestIntake());
            telemetry.addData("overshoot", SpindexerMotor.getCurrentPosition()-SpindexController.getSetPoint());
//            telemetry.addData("current", SpindexerMotor.getCurrentPosition());
//
//            telemetry.addData("intake active: ", IntakePower);
//            telemetry.addData("spindexer active: ", SpindexerMotor.isBusy());
            //telemetry.addData("Unwrapped Position: ", unwrapTarget(CurrentSpindexerPos, FindClosestOuttake()));
//            telemetry.addData("Intake sensor", findColor(IntakeSensor)==2 ? "green" : "purple");
//            telemetry.addData("left sensor", findColor(LeftSensor)==2 ? "green" : "purple");
//            telemetry.addData("right sensor", findColor(RightSensor)==2 ? "green" : "purple");
//            telemetry.addData("back sensor", findColor(BackSensor)==2 ? "green" : "purple");
            telemetry.update();
        }
        //end of teleop
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
        SpindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    int ShootMacroState = 99;
    int numOuttaked = 0;
    ElapsedTime ShootTimer = new ElapsedTime();
    int settleTime = 50;
    public boolean ShootUnsorted(){
        if(ShootMacroState == 0){//set up to outtake position
            SpindexSpinToClosestOuttake();

            numOuttaked = 0;
            ShootMacroState = 1;
        }else if(ShootMacroState == 1){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimer.reset();
                ShootMacroState = 2;
            }
        }else if(ShootMacroState == 2){//have settle time
            if(ShootTimer.milliseconds() > settleTime){
                ShootMacroState = 3;
            }
        }

        else if(ShootMacroState == 3){//start kick
            KickerState = 1;
            ShootMacroState = 4;
        }else if(ShootMacroState == 4){//wait until finished kicking
            if(KickerState == 0){
                numOuttaked++;
                ShootMacroState = 5;
            }
        }else if(ShootMacroState == 5){//spin to next chamber
            if(numOuttaked == 3){//exit after 3 balls shot
                ShootMacroState = 99;
                return true;
            }else{
                SpindexIncrementOuttake(2);
                ShootMacroState = 6;
            }
        }else if(ShootMacroState == 6){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimer.reset();
                ShootMacroState = 7;
            }
        }else if(ShootMacroState == 7){
            if(ShootTimer.milliseconds() > settleTime){//allow to settle
                ShootMacroState = 3;//loop back to kick
            }
        }
        return false;
    }
    public void SpindexIncrementOuttake(int i){
        //clockwise is positive
        int indexOfClosest = findIndexSpindex(FindClosestOuttake());
        indexOfClosest = incrementIndex(indexOfClosest,i,SpindexPos.length);
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }
    public boolean SpindexWithinTolerance(){
        if(Math.abs(SpindexController.getSetPoint()-SpindexerMotor.getCurrentPosition())<800){
            return true;
        }
        return false;
    }
    public void SpindexSpinToClosestOuttake(){
        int indexOfClosest = findIndexSpindex(FindClosestOuttake());
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }

    public double angleWrap(double radians){
        if(radians > Math.PI){
            radians -= 2*Math.PI;
        }
        if(radians < -Math.PI){
            radians += 2*Math.PI;
        }
        return radians;
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
        int rev = Math.floorDiv(current, ticksPerRevolution);//# of revolutions made by spindexer
        int best = rev * ticksPerRevolution + wrappedTarget;//find the closest unwrapped position

        int up = best + ticksPerRevolution;//check adjacent revolutions, one rotation up and one rotation down
        int down = best - ticksPerRevolution;

        if (Math.abs(up - current) < Math.abs(best - current)) best = up;
        if (Math.abs(down - current) < Math.abs(best - current)) best = down;

        return best;//returns the closest unwrapped target
    }

    public int SpindexGetAbsTarget(int wrappedTarget) {
        CurrentSpindexerPos = SpindexerMotor.getCurrentPosition();
        int absoluteTarget = unwrapTarget(CurrentSpindexerPos, wrappedTarget);
        return absoluteTarget;
    }
    public boolean detectColor(NormalizedColorSensor colorsensor){
        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(colors.toColor());
        if(hue > 90 && hue < 170){//green
            return true;
        }else if(hue > 200 && hue < 350){//purple
            return true;
        }
        return false;
    }
    public int findColor(NormalizedColorSensor colorsensor){
        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        double hue = JavaUtil.colorToHue(colors.toColor());
        if(hue > 90 && hue < 170){//green
            return 2;
        }else if(hue > 200 && hue < 350){//purple
            return 1;
        }
        return 0;
    }
}
