package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

import com.pedropathing.follower.Follower;

@Disabled
@Autonomous(name = "RED_FAR_6HP")
public class RED_FAR_6HP extends LinearOpMode {
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

    int[] SpindexPos = {
            0,//shooting0
            1365,//intaking1
            2730,//shooting2
            4095,//intaking3
            5460,//shooting4
            6825,//intaking5
    };
    ElapsedTime KickerTimer = new ElapsedTime();
    int KickerState = 0;
    int CurrentSpindexerPos;
    static int ticksPerRevolution = 8192;
    int IntakeState = 0;

    boolean IntakeActive = false;
    int velocity = 0;
    PIDFController SpindexController = new PIDFController(0.00024,0,0.00001,0);
    //PIDFController SpindexController = new PIDFController(0.00024,0,0.00001,0);


    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;


    private Follower follower;
    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime actionTimer;
    private ElapsedTime opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(96.38, 8.88, Math.toRadians(90));
    private final Pose shootPose = new Pose(88, 12, Math.toRadians(70));
    private final Pose humanPose = new Pose(127.35, 21.75, Math.toRadians(-90));
    private final Pose humanPose2 = new Pose(127.35, 20.75, Math.toRadians(-90));
    private final Pose humanPose3 = new Pose(120, 19.75, Math.toRadians(-120));
    private final Pose humanPose4 = new Pose(120, 26.75, Math.toRadians(-120));
    private final Pose leavePose = new Pose(115,12,Math.toRadians(90));

    private PathChain shootPreloads;
    private PathChain intakeHuman;
    private PathChain intakeHuman2;
    private PathChain intakeHuman3;
    private PathChain intakeHuman4;
    private PathChain shootHuman;
    private PathChain leave;

    public void runOpMode() {
        //HARDWARE MAPPING
        RightFront = hardwareMap.get(DcMotorEx.class, "fr");
        RightRear = hardwareMap.get(DcMotorEx.class, "br");
        LeftRear = hardwareMap.get(DcMotorEx.class, "bl");
        LeftFront = hardwareMap.get(DcMotorEx.class, "fl");
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

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        RightFlywheelMotor.setVelocityPIDFCoefficients(300,0,0,15.5);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        motorConfigs();

        waitForStart();
        KickerState = 0;
        SpindexIncrementOuttake(2);
        int x = 0;
        while (opModeIsActive()) {
//            motif = solveMotif(detectTag());
//            telemetry.addData("detect", detectTag());
//            telemetry.addData("motif0", motif[0]);
//            telemetry.addData("motif1", motif[1]);
//            telemetry.addData("motif2", motif[2]);


            if(pathState == 0){
                velocity = 1570;
                HoodServo.setPosition(1);
                follower.followPath(shootPreloads);
                pathState++;

            }if(pathState == 1){
                if(!follower.isBusy() && FlywheelGood()){
                    pathState++;
                }
            }if(pathState == 2){//shooting
                if(ShootUnsorted()){
                    pathState++;
                    velocity = 0;
                }
            }if(pathState == 3){
                follower.followPath(intakeHuman);
                pathState++;
                IntakeState = 0;
            }if(pathState == 4){
                Intake();
                if(!follower.isBusy()){
                    pathState++;
                }
            }if(pathState == 5){
                follower.followPath(intakeHuman2,1,true);
                pathState++;
                pathTimer.reset();
                IntakeState = 0;
            }if(pathState == 6){//
                Intake();
                if(!follower.isBusy() || pathTimer.milliseconds()>1500){
                    follower.followPath(intakeHuman3,1,true);
                    pathState++;
                    pathTimer.reset();
                    IntakeState = 0;

                }
            }if(pathState == 7){
                Intake();
                if(!follower.isBusy() || pathTimer.milliseconds()>2000){
                    follower.followPath(intakeHuman4);
                    pathState++;
                    IntakeState = 0;

                }
            }if(pathState == 8){
                Intake();
                if(!follower.isBusy()){
                    follower.followPath(shootHuman);
                    pathState++;
                    velocity = 1570;
                    ScoreState = 0;
                }
            }if(pathState == 9){
                if(!follower.isBusy() && FlywheelGood()){
                    pathState++;
                }
            }if(pathState == 10){
                if(ShootUnsorted()){
                    pathState++;
                    velocity = 0;
                }
            }if(pathState == 11){
                follower.followPath(leave);
                pathState++;
            }

            follower.update();
            kickSM();
            SpindexSM();
            FlywheelSM(velocity);

            //telemetry.addData("follower busy", follower.isBusy());
            //telemetry.addData("x", x);

//            telemetry.addData("path state", pathState);
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("overshoot", SpindexerMotor.getCurrentPosition()-SpindexController.getSetPoint());
//            telemetry.addData("Intake ColorSensor: ", findColor(IntakeSensor));
//            telemetry.addData("Left ColorSensor: ", findColor(LeftSensor));
//            telemetry.addData("Right ColorSensor: ", findColor(RightSensor));
//            telemetry.addData("Back ColorSensor: ", findColor(BackSensor));
//
//            telemetry.addData("Intake ColorSensor hue: ", JavaUtil.colorToHue(IntakeSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Left ColorSensor hue: ", JavaUtil.colorToHue(LeftSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Right ColorSensor hue: ", JavaUtil.colorToHue(RightSensor.getNormalizedColors().toColor()));
//            telemetry.addData("Back ColorSensor hue: ", JavaUtil.colorToHue(BackSensor.getNormalizedColors().toColor()));
            telemetry.addData("kicker", KickerState);
            telemetry.addData("scorestate ", ScoreState);
            telemetry.addData("pathstate", pathState);
            telemetry.update();
        }
    }
    public void buildPaths(){
        shootPreloads = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        intakeHuman = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, humanPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), humanPose.getHeading())
                .build();
        intakeHuman2 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, humanPose2))
                .setLinearHeadingInterpolation(humanPose.getHeading(), humanPose2.getHeading())
                .build();
        intakeHuman3 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose2, humanPose3))
                .setLinearHeadingInterpolation(humanPose2.getHeading(), humanPose3.getHeading())
                .build();
        intakeHuman4 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose3, humanPose4))
                .setLinearHeadingInterpolation(humanPose3.getHeading(), humanPose4.getHeading())
                .build();
        shootHuman = follower.pathBuilder()
                .addPath(new BezierLine(humanPose4, shootPose))
                .setLinearHeadingInterpolation(humanPose4.getHeading(), shootPose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();
    }

    int ScoreStateSorted = 0;
    int numOuttakedSorted = 0;
    ElapsedTime ShootTimerSorted = new ElapsedTime();
    int[] motif = new int[] {1,1,2};////////////////////////////////
    public void ShootSorted(){
        if(ScoreStateSorted == 0){//set up to outtake position
            SpindexSpinToClosestOuttake();
            numOuttakedSorted = 0;
            ScoreStateSorted = 1;
        }else if(ScoreStateSorted == 1){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimerSorted.reset();
                ScoreStateSorted = 2;
            }
        }else if(ScoreStateSorted == 2){//have settle time
            if(ShootTimerSorted.milliseconds() > 50){
                ScoreStateSorted = 3;
            }
        }

        else if(ScoreStateSorted == 3){//spin to color
            SpindexSpinToColor(motif[0]);
            ScoreStateSorted = 4;
        }else if(ScoreStateSorted == 4){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimerSorted.reset();
                ScoreStateSorted = 5;
            }
        }else if(ScoreStateSorted == 5){//have settle time
            if(ShootTimerSorted.milliseconds() > 50){
                ScoreStateSorted = 6;
            }
        }

        else if(ScoreStateSorted == 6){//start kick
            KickerState = 1;
            ScoreStateSorted = 7;
        }else if(ScoreStateSorted == 7){//wait until finished kicking
            if(KickerState == 0){
                numOuttakedSorted++;
                ScoreStateSorted = 8;
            }
        }else if(ScoreStateSorted == 8){//spin to next chamber
            if(numOuttakedSorted == 3){//exit after 3 balls shot
                ScoreStateSorted = 99;
            }else{
                SpindexSpinToColor(motif[numOuttakedSorted]);
                ScoreStateSorted = 9;
            }
        }else if(ScoreStateSorted == 9){//wait until at tolerance
            if(SpindexWithinTolerance()){
                ShootTimerSorted.reset();
                ScoreStateSorted = 10;
            }
        }else if(ScoreStateSorted == 10){
            if(ShootTimerSorted.milliseconds() > 50){//allow to settle
                ScoreStateSorted = 6;//loop back to kick
            }
        }
    }

    int ScoreState = 0;
    int numOuttaked = 0;
    ElapsedTime ShootTimer = new ElapsedTime();
    public boolean ShootUnsorted(){
        if(ScoreState == 0){//set up to outtake position
            SpindexSpinToClosestOuttake();
            numOuttaked = 0;
            ScoreState = 1;
        }else if(ScoreState == 1){//wait until at tolerance
            if(SpindexWithinTolerance(1000)){
                ShootTimer.reset();
                ScoreState = 2;
            }
        }else if(ScoreState == 2){//have settle time
            if(ShootTimer.milliseconds() > 50){
                ScoreState = 3;
            }
        }

        else if(ScoreState == 3){//start kick
            KickerState = 1;
            ScoreState = 4;
        }else if(ScoreState == 4){//wait until finished kicking
            if(KickerState == 0){
                numOuttaked++;
                ScoreState = 5;
            }
        }else if(ScoreState == 5){//spin to next chamber
            if(numOuttaked == 3){//exit after 3 balls shot
                ScoreState = 99;
                return true;
            }else{
                SpindexIncrementOuttake(2);
                ScoreState = 6;
            }
        }else if(ScoreState == 6){//wait until at tolerance
            if(SpindexWithinTolerance(1000)){
                ShootTimer.reset();
                ScoreState = 7;
            }
        }else if(ScoreState == 7){
            if(ShootTimer.milliseconds() > 50){//allow to settle
                ScoreState = 3;//loop back to kick
            }
        }
        return false;
    }

    int numIntaked = 0;
    public boolean Intake(){
        if(IntakeState == 0){
            SpindexSpinToClosestIntake();//set up for intaking
            if(SpindexWithinTolerance()){
                IntakeState = 1;
                numIntaked = 0;
            }
        }else if(IntakeState == 1){
            if(detectColor(IntakeSensor) && SpindexWithinTolerance() && numIntaked<3){
                SpindexIncrementIntake(2);//if detect something and we have less than 3 balls
                numIntaked++;//move chamber
            }
            if(numIntaked<3){//turn intake on
                IntakeMotor.setPower(0.7);
                IntakeActive = true;
            }else{
                IntakeActive = false;
                SpindexSpinToClosestOuttake();//ready for outtaking
                IntakeState = 99;
                return true;
            }
        }
        return false;
    }
    public boolean SpindexSpinToColor(int color){
        //1 = purple
        //2 = green
        if(findColor(BackSensor) == color){
            return true;
        }else if(findColor(RightSensor) == color){
            SpindexIncrementOuttake(2);
            return true;
        }else if(findColor(LeftSensor) == color){
            SpindexIncrementOuttake(-2);
            return true;
        }

        else{//spin to one with empty
            if(findColor(RightSensor)==0){
                SpindexIncrementOuttake(2);
            }else if(findColor(LeftSensor)==0){
                SpindexIncrementOuttake(-2);
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
    public void SpindexIncrementIntake(int i){
        //clockwise is positive
        int indexOfClosest = findIndexSpindex(FindClosestIntake());
        indexOfClosest = incrementIndex(indexOfClosest,i,SpindexPos.length);
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }

    public boolean SpindexWithinTolerance(){
        if(Math.abs(SpindexController.getSetPoint()-SpindexerMotor.getCurrentPosition())<320){
            return true;
        }
        return false;
    }
    public boolean SpindexWithinTolerance(int tol){
        if(Math.abs(SpindexController.getSetPoint()-SpindexerMotor.getCurrentPosition())<tol){
            return true;
        }
        return false;
    }

    public void SpindexSpinToClosestOuttake(){
        int indexOfClosest = findIndexSpindex(FindClosestOuttake());
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }
    public void SpindexSpinToClosestIntake(){
        int indexOfClosest = findIndexSpindex(FindClosestIntake());
        SpindexController.setSetPoint(SpindexGetAbsTarget(SpindexPos[indexOfClosest]));
    }
    public void SpindexSM(){
        double error = SpindexerMotor.getCurrentPosition()-SpindexController.getSetPoint();
        if(SpindexWithinTolerance()){
            SpindexerMotor.setPower(0);
            if(!IntakeActive){
                IntakeMotor.setPower(0);
            }
        }else{
            IntakeMotor.setPower(0.7);
            double raw = SpindexController.calculate(SpindexerMotor.getCurrentPosition());
            SpindexerMotor.setPower(raw+(Math.signum(raw)*0.06));
        }
    }
    public int FlywheelSM(int v){
        if(v == 0){
            RightFlywheelMotor.setPower(0);
            LeftFlywheelMotor.setPower(0);
        }
        if(v - RightFlywheelMotor.getVelocity() > 50){
            RightFlywheelMotor.setPower(1);
            LeftFlywheelMotor.setPower(1);
            return 0;
        }else{
            RightFlywheelMotor.setVelocity(v);
            LeftFlywheelMotor.setPower(RightFlywheelMotor.getPower());
            return 1;
        }
    }
    public boolean FlywheelGood(){
        if(Math.abs(velocity-RightFlywheelMotor.getVelocity())<100){
            return true;
        }
        return false;
    }

    public void kickSM(){
        if(KickerState == 0){
            KickerServo.setPosition(1);//low position
        }else if(KickerState == 1){
            KickerServo.setPosition(0.5);//high position
            KickerTimer.reset();
            KickerState = 2;//ready to wait
        }else if(KickerState == 2){
            if(KickerTimer.milliseconds()>250){//time needed to move up has passed
                KickerServo.setPosition(1);//low position
                KickerTimer.reset();
                KickerState = 3;
            }
        }else if(KickerState == 3) {
            if (KickerTimer.milliseconds() > 150) {//cooldown period has passed
                KickerState = 0;
            }
        }
    }

    public int detectTag() {
        int selectedTagId = -1;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for(AprilTagDetection detection : detections) {
            if(detection.id == 21 || detection.id == 22 || detection.id == 23) {
                selectedTagId = detection.id;
                telemetry.addData("Selected ID", selectedTagId);

            }
        }
        return selectedTagId;
    }

    public int[] solveMotif(int selectedTagId){
        //ID# 21 = gpp
        //22 = pgp
        //23 = ppg
        int[] motif = new int[] {0,0,0};
        if(selectedTagId == 21){
            motif = new int[] {2,1,1};//gpp
        }else if(selectedTagId == 22){
            motif = new int[] {1,2,1};//pgp
        }else if(selectedTagId == 23){
            motif = new int[] {1,1,2};//ppg
        }

        return motif;
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