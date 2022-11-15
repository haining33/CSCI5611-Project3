void setup() {
  size(1280, 720, P2D);
  surface.setTitle(" Kinematics [CSCI 5611 Example]");

}

//arm roots
Vec2 root1 = new Vec2(700, 300);
Vec2 root2 = new Vec2(600, 300);
Vec2 body = new Vec2(610, 270);
Vec2 head = new Vec2(650, 250);
//body position

//upper arm, lower arm, hand, finger
float[] l1 = {45, 45, 45, 45};
//shoulder, elbow, wrist, finger
float[] a1 = {0,0,0,0};

float[] l2 = {45, 45, 45, 45};
float[] a2 = {0,0,0,0}; 

Vec2[] start1 = new Vec2[4];
Vec2[] start2 = new Vec2[4];


float speed = 2;
boolean aPressed = false;
boolean dPressed = false;
boolean wPressed = false;
boolean sPressed = false;

Vec2 goalPos = new Vec2(640, 60);

float arm_width = 20;
float[] widthArm = {arm_width + 10, arm_width, arm_width - 10, arm_width - 15};

float limit = 0.00001;
void solve(float[] angle, Vec2[] start, Vec2 root, float dt) {
  Vec2 goal = new Vec2(goalPos.x, goalPos.y);
  Vec2 startToGoal = goal.minus(start[2]);
  Vec2 startToEndEffector = start[3].minus(start[2]);
  float dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  if(acos(dotProd) > 0.1){
    if (cross(startToGoal, startToEndEffector) < 0){
      angle[3] += (limit+dt);
    }
    else{
      angle[3] -= (limit+dt);
    }
  }

  fk(); //Update link positions with fk (e.g. end effector changed)


  //Update wrist joint
  startToGoal = goal.minus(start[1]);
  startToEndEffector = start[3].minus(start[1]);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  if(acos(dotProd) > 0.1){
    if (cross(startToGoal, startToEndEffector) < 0){
      angle[2] += (limit+dt);
      // print(limit+dt);
    }
    else{
      angle[2] -= (limit+dt);
    }
  }

  fk(); //Update link positions with fk (e.g. end effector changed)

  //Update elbow joint
  startToGoal = goal.minus(start[0]);
  startToEndEffector = start[3].minus(start[0]);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  if(acos(dotProd) > 0.1){  
    if (cross(startToGoal, startToEndEffector) < 0){
      angle[1] += (limit+dt);
    }
    else{
      angle[1] -= (limit+dt);
    }
  }

    // angle[1] = max(angle[1], -HALF_PI);

  fk(); //Update link positions with fk (e.g. end effector changed)

  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = start[3].minus(root);
  dotProd = dot(startToGoal.normalized(), startToEndEffector.normalized());
  dotProd = clamp(dotProd, -1, 1);
  if(acos(dotProd) > 0.1){
    if (cross(startToGoal, startToEndEffector) < 0){
      angle[0] += limit + dt;
    }
    else{
      angle[0] -= limit + dt;
    }
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
}


void fk() {
  //arm one
  start1[0] = new Vec2(cos(a1[0])*l1[0], sin(a1[0])*l1[0]).plus(root1);
  start1[1] = new Vec2(cos(a1[0]+a1[1])*l1[1], sin(a1[0]+a1[1])*l1[1]).plus(start1[0]);
  start1[2] = new Vec2(cos(a1[0]+a1[1]+a1[2])*l1[2], sin(a1[0]+a1[1]+a1[2])*l1[2]).plus(start1[1]);
  start1[3] = new Vec2(cos(a1[0]+a1[1]+a1[2]+a1[3])*l1[3], sin(a1[0]+a1[1]+a1[2]+a1[3])*l1[3]).plus(start1[2]);

  //arm two
  start2[0] = new Vec2(cos(a2[0])*l2[0], sin(a2[0])*l2[0]).plus(root2);
  start2[1] = new Vec2(cos(a2[0]+a2[1])*l2[1], sin(a2[0]+a2[1])*l2[1]).plus(start2[0]);
  start2[2] = new Vec2(cos(a2[0]+a2[1]+a2[2])*l2[2], sin(a2[0]+a2[1]+a2[2])*l2[2]).plus(start2[1]);
  start2[3] = new Vec2(cos(a2[0]+a2[1]+a2[2]+a2[3])*l2[3], sin(a2[0]+a2[1]+a2[2]+a2[3])*l2[3]).plus(start2[2]);
}





void draw() {
  background(250,250,250);
  //abs(mouseX - goalPos.x) < goalRad + 2 && abs(mouseY - goalPos.y) < goalRad  + 2 &&
  if (mousePressed) {
    goalPos = new Vec2(mouseX, mouseY);
    println(goalPos);
  }
  if(wPressed){
    root1.y -= 5;
    root2.y -= 5;
    body.y -= 5;
    head.y -= 5;
  }
  if(sPressed){
    root1.y += 5;
    root2.y += 5;
    body.y += 5;
    head.y += 5;
  }
  if(aPressed){
    root1.x -= 5;
    root2.x -= 5;
    body.x -= 5;
    head.x -= 5;
  }
  if(dPressed){
    root1.x += 5;
    root2.x += 5;
    body.x += 5;
    head.x += 5;
  }
  fk();
  
  Vec2 ellipsePos = new Vec2(0,0);
  solve(a1, start1, root1, 1/frameRate);
  solve(a2, start2, root2, 1/frameRate);

  //color arms
  fill(255, 0, 0);
  pushMatrix();
  circle(head.x, head.y, 50);
  popMatrix();
  
  pushMatrix();
  translate(body.x, body.y);
  rect(0, 0, 80 ,100);
  popMatrix();




  fill(255, 255, 255);
  circle(goalPos.x, goalPos.y, 10);

  fill(255, 224, 189);
  float arm_width = 20;
  //draw arm one


  pushMatrix();
  translate(root1.x , root1.y);
  rotate(a1[0]);
  rect(0, -arm_width/2, l1[0], arm_width);
  popMatrix();

  pushMatrix();
  translate(start1[0].x, start1[0].y);
  rotate(a1[0] + a1[1]);
  rect(0, -arm_width/2, l1[1], arm_width);
  popMatrix();

  pushMatrix();
  translate(start1[1].x, start1[1].y);
  rotate(a1[0] + a1[1] + a1[2]);

  rect(0, -arm_width/2, l1[2], arm_width);
  popMatrix();

  pushMatrix();
  translate(start1[2].x , start1[2].y);
  rotate(a1[0] + a1[1] + a1[2] + a1[3]);

  rect(0, -arm_width/2, l1[3], arm_width);
  popMatrix();

  //draw arm two 
  pushMatrix();
  translate(root2.x, root2.y);
  rotate(a2[0]);
  rect(0, -arm_width/2, l2[0], arm_width);

  popMatrix();

  pushMatrix();
  translate(start2[0].x, start2[0].y);
  rotate(a2[0] + a2[1]);
  rect(0, -arm_width/2, l2[1], arm_width);
  popMatrix();

  pushMatrix();
  translate(start2[1].x, start2[1].y);
  rotate(a2[0] + a2[1] + a2[2]);
  rect(0, -arm_width/2, l2[2], arm_width);
  popMatrix();

  pushMatrix();
  translate(start2[2].x, start2[2].y);
  rotate(a2[0] + a2[1] + a2[2] + a2[3]);
  rect(0, -arm_width/2, l2[3], arm_width);
  popMatrix();
}


void keyPressed() {
  if (key == 'r') {
    goalPos = new Vec2(640, 60);
  }
  if(key == 'w')
    wPressed = true;
  if(key == 'a')
    aPressed = true;
  if(key == 's')
    sPressed = true;
  if(key == 'd')
    dPressed = true;
}

void keyReleased() {
  if(key == 'w')
    wPressed = false;
  if(key == 'a')
    aPressed = false;
  if(key == 's')
    sPressed = false;
  if(key == 'd')
    dPressed = false;
}



//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;

  public Vec2(float x, float y) {
    this.x = x;
    this.y = y;
  }

  public String toString() {
    return "(" + x+ "," + y +")";
  }

  public float length() {
    return sqrt(x*x+y*y);
  }

  public Vec2 plus(Vec2 rhs) {
    return new Vec2(x+rhs.x, y+rhs.y);
  }

  public void add(Vec2 rhs) {
    x += rhs.x;
    y += rhs.y;
  }

  public Vec2 minus(Vec2 rhs) {
    return new Vec2(x-rhs.x, y-rhs.y);
  }

  public void subtract(Vec2 rhs) {
    x -= rhs.x;
    y -= rhs.y;
  }

  public Vec2 times(float rhs) {
    return new Vec2(x*rhs, y*rhs);
  }

  public void mul(float rhs) {
    x *= rhs;
    y *= rhs;
  }

  public void clampToLength(float maxL) {
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL) {
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }

  public void setToLength(float newL) {
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }

  public void normalize() {
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }

  public Vec2 normalized() {
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }

  public float distanceTo(Vec2 rhs) {
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t) {
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t) {
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b) {
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b) {
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b) {
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max) {
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
