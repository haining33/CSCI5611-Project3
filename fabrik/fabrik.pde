int numJoints = 4;
float tolerance = 0.1;
Vec2 target = new Vec2(0, 0);
Vec2 joints[] = new Vec2[numJoints];
float lengths[] = new float[numJoints - 1];
float angles[] = new float[numJoints - 1];
Vec2 origin = new Vec2(0, 0);
float totalLength = 0;  
float armWidth = 20;

void setup(){
  size(1280,720);
  surface.setTitle("FABRIK");
  
  joints[0] = new Vec2(700, 300);
  for (int i = 1; i < numJoints; i++) {
    joints[i] = new Vec2(700 + i * 150, 300);
  }
  for (int i = 0; i < numJoints-1; i++) {
    lengths[i] = (joints[i].minus(joints[i+1])).length();
  }
  origin = new Vec2(700, 300);
  for (int i = 0; i < numJoints-1; i++) {
    totalLength += lengths[i];  
  }
}



void backward() {
  joints[numJoints-1] = target;
  for (int i = numJoints-2; i >= 0; i--) {
    Vec2 r = joints[i+1].minus(joints[i]);
    float l = lengths[i] / r.length();
    Vec2 pos = joints[i+1].times(1 - l).plus(joints[i].times(l));
    joints[i] = pos;
  }
}

void forward() {
  joints[0] = origin;
  for (int i = 0; i < numJoints-1; i++) {
    Vec2 r = joints[i+1].minus(joints[i]);
    float l = lengths[i] / r.length();
    Vec2 pos = joints[i].times(1 - l).plus(joints[i+1].times(l));
    joints[i+1] = pos;
  }
}

void solve() {

  float distance = joints[0].minus(target).length();
  if (distance > totalLength) {
    for (int i = 0; i < numJoints-1; i++) {
      Vec2 r = target.minus(joints[i]);
      float l = lengths[i] / r.length();
      joints[i+1] = joints[i].times(1 - l).plus(target.times(l));
    }
  }
  else {
    int count = 0;
    float dif = joints[numJoints-1].minus(target).length();
    while (dif > tolerance) {
      backward();
      forward();
      dif = joints[numJoints-1].minus(target).length();
      count += 1;
      if (count > 10) {
        break;
      }
    }
  }
  for(int i = 0; i < numJoints - 1; i++){
    float x = joints[i + 1].x - joints[i].x;
    float y = joints[i + 1].y - joints[i].y;
    if(x >= 0 && y >= 0 || x >= 0 && y < 0) {
      angles[i] = atan(y/x);
    }else{
      angles[i] = PI + atan(y/x);
    }
  }
}


void draw() {
  if (mousePressed) {
    target = new Vec2(mouseX, mouseY);
  }
  
  solve();
  background(255, 255, 255);
  stroke(0, 0, 255);
  strokeWeight(2);
  for (int i = 0; i < numJoints-1; i++) {
    fill(255,0,0);
    pushMatrix();
    translate(joints[i].x, joints[i].y);
    rotate(angles[i]);
    rect(0,-armWidth/2, lengths[i], armWidth);
    popMatrix();
  }
  noStroke();
}

//-----------------
// Vector Library
//-----------------

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
