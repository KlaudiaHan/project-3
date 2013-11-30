import SimpleOpenNI.*;
import ddf.minim.*;
import processing.serial.*;

Doorbell doorbell;
Doorbell frogSound;
Doorbell treeSound;
Doorbell monkeySound;
Minim minim;
SimpleOpenNI kinect;

leafGroup leaves;


int userID;
int[] userMap;
float yoff = 0;
int seed = 5;
PImage frogImage;
PImage monkeyImage;
PImage leafImage;


float frogValue = 0;
float treeValue = 0;
float lastTreeValue=0;
float minTreeValue=0;

Serial myPort;

void setup() { 
  size(640, 480,P3D);
  kinect = new SimpleOpenNI(this);
  kinect.enableDepth();
  kinect.enableRGB(); 
  kinect.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
  kinect.alternativeViewPointDepthToImage();
  kinect.setMirror(true);
  smooth();
  // start up Minim
  minim = new Minim(this);
  // Create a new doorbell
  doorbell = new Doorbell(150, 100, 32, "bird.mp3");
  frogSound = new Doorbell(150, 100, 32, "frog.wav");
  monkeySound = new Doorbell(150, 100, 32, "monkey.wav");
  treeSound = new Doorbell(150, 100, 32, "wind.wav");
  frogImage = loadImage("frog.png");
  monkeyImage = loadImage("monkey.png");
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[4], 9600);
  myPort.bufferUntil('\n');
  leafImage=loadImage("leaf.png");

leaves = new leafGroup();
leaves.setUp();
}
void draw() {
  background(200);
  doorbell.display(100, 100);
  doorbell.jiggle();
  
  

  kinect.update();
  //-------------------stead tree---------------------
  pushMatrix();
  stroke(0);
  // Start the tree from the bottom of the screen
  translate(100, height);
  // Move alogn through noise
  yoff += 0.005;
  randomSeed(seed);
  // Start the recursive branching!
  branch(110, 0);
  popMatrix();
  
  
  pushMatrix();
  stroke(0);
  // Start the tree from the bottom of the screen
  translate(600, height);
  // Move alogn through noise
  yoff += 0.005;
  randomSeed(seed);
  // Start the recursive branching!
  branch(50, 0);
  popMatrix();
  
  minTreeValue = treeValue - lastTreeValue;
  //--------------------frog--and---tree---------------------
 if(frogValue>90){
    println("frog frog frog"+frogValue);
     image(frogImage,90,height-250);
     frogSound.ring();
  }
  if(minTreeValue>50){
    leaves.alldead = false;
  }
  if(!leaves.alldead){
     treeSound.ring();
     leaves.move();
  }
  println(minTreeValue);
  lastTreeValue = treeValue;
  //--------------------------------------------------

  PImage rgbImage = kinect.rgbImage();
  //image(depth, 0, 0);
  rgbImage.loadPixels();
  loadPixels();
  IntVector userList = new IntVector();
  userMap = kinect.getUsersPixels(SimpleOpenNI.USERS_ALL);
  kinect.getUsers(userList);
  for (int i =0; i < userMap.length; i++) {
    if (userMap[i] != 0) {
      // set the sketch pixel to the color pixel
      pixels[i] = color(0);
    }
  }
  updatePixels();
  if (userList.size() > 0) {
    for (int i =0; i < userList.size(); i++) {
      int userId = userList.get(i);

      if ( kinect.isTrackingSkeleton(userId)) {
        PVector position = new PVector();
        kinect.getCoM(userId, position);
        kinect.convertRealWorldToProjective(position, position);
        fill(255, 0, 0);
        //ellipse(position.x, position.y, 25, 25);

        PVector head = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_HEAD, 
        head);

        PVector convertedHead = new PVector();
        kinect.convertRealWorldToProjective(head, convertedHead);
       // ellipse(convertedHead.x, convertedHead.y, 10, 10);

        PVector neck = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_NECK, 
        neck);

        PVector convertedNeck = new PVector();
        kinect.convertRealWorldToProjective(neck, convertedNeck);
       // ellipse(convertedNeck.x, convertedNeck.y, 10, 10);



        //-------------------tree-------------

        pushMatrix();
        stroke(0);
        // Start the tree from the bottom of the screen
        translate(convertedHead.x, convertedHead.y);
        // Move alogn through noise
        yoff += 0.005;
        randomSeed(seed);
        // Start the recursive branching!
        branch(50, 0);
        popMatrix();

        //------------------------------------
        // get the positions of the three joints of our arm
        PVector rightHand = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_RIGHT_HAND, 
        rightHand);

        PVector convertedRightHand = new PVector();
        kinect.convertRealWorldToProjective(rightHand, convertedRightHand);


        PVector rightElbow = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_RIGHT_ELBOW, 
        rightElbow);

        PVector convertedRightElbow = new PVector();
        kinect.convertRealWorldToProjective(rightElbow, convertedRightElbow);


        PVector rightShoulder = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_RIGHT_SHOULDER, 
        rightShoulder);

        PVector convertedRightShoulder = new PVector();
        kinect.convertRealWorldToProjective(rightShoulder, convertedRightShoulder);


        // right hip to orient the shoulder angle
        PVector rightHip = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_RIGHT_HIP, 
        rightHip);
        // reduce our joint vectors to two dimensions
        PVector rightHand2D = new PVector(rightHand.x, rightHand.y); 
        PVector rightElbow2D = new PVector(rightElbow.x, rightElbow.y);
        PVector rightShoulder2D = new PVector(rightShoulder.x, 
        rightShoulder.y);
        PVector rightHip2D = new PVector(rightHip.x, rightHip.y);
        // calculate the axes against which we want to measure our angles
        PVector torsoOrientation =
          PVector.sub(rightShoulder2D, rightHip2D); 
        PVector upperArmOrientation =
          PVector.sub(rightElbow2D, rightShoulder2D);
        // calculate the angles between our joints
        float shoulderAngle = angleOf(rightElbow2D, 
        rightShoulder2D, 
        torsoOrientation);
        float elbowAngle = angleOf(rightHand2D, 
        rightElbow2D, 
        upperArmOrientation);
        // show the angles on the screen for debugging

        //---------------------------------------------------right -------------------
        //---------------------------------------left-------------------------------
        PVector leftHand = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_LEFT_HAND, 
        leftHand);

        PVector convertedLeftHand = new PVector();
        kinect.convertRealWorldToProjective(leftHand, convertedLeftHand);


        PVector leftElbow = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_LEFT_ELBOW, 
        leftElbow);

        PVector convertedLeftElbow = new PVector();
        kinect.convertRealWorldToProjective(leftElbow, convertedLeftElbow);


        PVector leftShoulder = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_LEFT_SHOULDER, 
        leftShoulder);

        PVector convertedLeftShoulder = new PVector();
        kinect.convertRealWorldToProjective(leftShoulder, convertedLeftShoulder);


        // left hip to orient the shoulder angle
        PVector leftHip = new PVector();
        kinect.getJointPositionSkeleton(userId, 
        SimpleOpenNI.SKEL_LEFT_HIP, 
        leftHip);
        // reduce our joint vectors to two dimensions
        PVector leftHand2D = new PVector(leftHand.x, leftHand.y); 
        PVector leftElbow2D = new PVector(leftElbow.x, leftElbow.y);
        PVector leftShoulder2D = new PVector(leftShoulder.x, 
        leftShoulder.y);
        PVector leftHip2D = new PVector(leftHip.x, leftHip.y);
        // calculate the axes against which we want to measure our angles
        PVector torsoOrientationLf =
          PVector.sub(leftShoulder2D, leftHip2D); 
        PVector upperArmOrientationLf =
          PVector.sub(leftElbow2D, leftShoulder2D);
        // calculate the angles between our joints
        float shoulderAngleLf = angleOf(leftElbow2D, 
        leftShoulder2D, 
        torsoOrientationLf);
        float elbowAngleLf = angleOf(leftHand2D, 
        leftElbow2D, 
        upperArmOrientationLf);
        //--------------------------------------------------------------------------
        
        float armDirection;
        armDirection = ((convertedRightHand.x-convertedNeck.x)*(convertedLeftHand.x-convertedNeck.x));
        float rightArmDirection = convertedRightHand.x-convertedNeck.x;
        float leftArmDirection = convertedNeck.x-convertedLeftHand.x;
        fill(255, 0, 0);
        //scale(3);
        /*text("shoulder: " + int(shoulderAngle) + "\n" +
          " elbow: " + int(elbowAngle), 20, 20);
        */
        
        //--------------------------------- gestures ----------------------------------------
        if (shoulderAngle>70&&shoulderAngle<140&&elbowAngle>150&&armDirection<0&&rightArmDirection>0&&leftArmDirection>0) {
          wings(convertedNeck.x, convertedNeck.y, convertedRightHand.x, convertedRightHand.y);
          doorbell.ring();
        }
        
        if (shoulderAngleLf>70&&shoulderAngleLf<140&&elbowAngleLf>150&&armDirection<0&&rightArmDirection>0&&leftArmDirection>0) {
          wings(convertedNeck.x, convertedNeck.y, convertedLeftHand.x, convertedLeftHand.y);
        }
        
        if (shoulderAngle>140&&elbowAngle>150&&armDirection<0&&rightArmDirection>0&&leftArmDirection>0){
            pushMatrix();
            stroke(0);
            // Start the tree from the bottom of the screen
            translate(convertedRightHand.x, convertedRightHand.y-15);
            // Move alogn through noise
            yoff += 0.005;
            randomSeed(seed);
            // Start the recursive branching!
            branch(30, 0);
            popMatrix();
            
        }
        
        if (shoulderAngleLf>140&&elbowAngleLf>150&&armDirection<0&&rightArmDirection>0&&leftArmDirection>0){
            pushMatrix();
            stroke(0);
            // Start the tree from the bottom of the screen
            translate(convertedLeftHand.x, convertedLeftHand.y);
            // Move alogn through noise
            yoff += 0.005;
            randomSeed(seed);
            // Start the recursive branching!
            branch(30, 0);
            popMatrix();
        }
        if(shoulderAngle>60&&shoulderAngle<120&&elbowAngle<140&&shoulderAngleLf>60&&shoulderAngleLf<120&&elbowAngleLf<140&&armDirection<0&&rightArmDirection>0&&leftArmDirection>0){
          imageMode(CENTER);
          image(monkeyImage,convertedHead.x,convertedHead.y,100,100);
          monkeySound.ring();
        }
        if(armDirection>0){
            if(convertedRightHand.y>convertedLeftHand.y){
              tooth(convertedRightShoulder.x,convertedRightShoulder.y,convertedRightHand.x,convertedRightHand.y,true);
              tooth(convertedLeftShoulder.x,convertedLeftShoulder.y,convertedLeftHand.x,convertedLeftHand.y,false);
            } else if(convertedRightHand.y<convertedLeftHand.y){
              tooth(convertedRightShoulder.x,convertedRightShoulder.y,convertedRightHand.x,convertedRightHand.y,false);
              tooth(convertedLeftShoulder.x,convertedLeftShoulder.y,convertedLeftHand.x,convertedLeftHand.y,true);
            }
        }
      }
    }
  }
}
float angleOf(PVector one, PVector two, PVector axis) {
  PVector limb = PVector.sub(two, one);
  return degrees(PVector.angleBetween(limb, axis));
}
// user-tracking callbacks!
void onNewUser(int uID) {
  println("start pose detection");
  userID = uID;
  kinect.startPoseDetection("Psi", uID);
}
void onEndCalibration(int userId, boolean successful) {
  if (successful) {
    println(" User calibrated !!!");
    kinect.startTrackingSkeleton(userId);
  }
  else {
    println(" Failed to calibrate user !!!");
    kinect.startPoseDetection("Psi", userId);
  }
}
void onStartPose(String pose, int userId) {
  println("Started pose for user");
  kinect.stopPoseDetection(userId);
  kinect.requestCalibrationSkeleton(userId, true);
}


// Close the sound files
public void stop() {
  // The doorbell object must close its sound.
  doorbell.close(); 
  frogSound.close();
  treeSound.close();
  super.stop();
}


void serialEvent(Serial myPort) { 
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    float[] colors = float(split(inString, ","));
    if (colors.length >=2) {
      // map them to the range 0-255:
      treeValue = colors[0];
      frogValue = colors[1];
    }
    
  }
}
