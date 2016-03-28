// Based on example by Casey Reas and Ben Fry

Flock flock;

boolean mouseActive = false;
float desiredseparation = 25.0f;
float neighbordist = 40.0f;

// http://stackoverflow.com/questions/4169350/getting-mouse-position-in-a-canvas-element-when-z-index-1
//$(document).mousemove(function (e){
//  mouseX = e.pageX;
//  mouseY = e.pageY;
//});

float myMouseX;
float myMouseY;

void setup() {

  int sx = window.innerWidth;
  int sy = window.innerHeight;

  size(sx, sy);

  frameRate(25);

  int nBoids = (int)ceil(max(sx, sy) / 15f);
  
  mouseActive = false;
  myMouseX = width / 2;
  myMouseY = height / 2;

  colorMode(RGB,255,255,255,255);

  flock = new Flock();

  // Add an initial set of boids into the system

  for (int i = 0; i < nBoids; i++) {
    //flock.addBoid(new Boid(new Vector3D(width/2,height/2),2.0f,0.05f));
    flock.addBoid(new Boid(new Vector3D(random(width), random(height)), 2.5f, 0.06f));
  }

  background(255, 255, 255);

  smooth();

}



void draw() {
  background(255, 255, 255);
  flock.run();
}


void updateMouse(boolean active) {
  mouseActive = active;
  myMouseX = mouseX;
  myMouseY = mouseY;
}

void mousePressed() {
  //console.log("PRESSED: " + mouseX + ", " + mouseY);
  //flock.addBoid(new Boid(new Vector3D(mouseX,mouseY),2.0f,0.05f));
  updateMouse(true);
}

void mouseReleased() {
  //console.log("RELEASED: " + mouseX + ", " + mouseY);
  mouseActive = false;
  updateMouse(false);
}

void mouseOut() {
  //console.log("OUT: " + mouseX + ", " + mouseY);
  mouseActive = false; 
  updateMouse(false);
}

void mouseIn() {
  //console.log("IN: " + mouseX + ", " + mouseY);
  mouseActive = false; 
  updateMouse(false);
}

void mouseMoved() {
  //console.log("MOVED: " + mouseX + ", " + mouseY);
  updateMouse(false);
}

void mouseDragged() {
  updateMouse(true);
}



class Flock {

  ArrayList boids; // An arraylist for all the boids



  Flock() {

    boids = new ArrayList(); // Initialize the arraylist

  }



  void run() {

    for (int i = 0; i < boids.size(); i++) {

      Boid b = (Boid) boids.get(i);  

      b.run(boids);  // Passing the entire list of boids to each boid individually

    }

  }



  void addBoid(Boid b) {

    boids.add(b);

  }



}





class Boid {



  Vector3D loc;

  Vector3D vel;

  Vector3D acc;

  float r;

  float maxforce;    // Maximum steering force

  float maxspeed;    // Maximum speed



  Boid(Vector3D l, float ms, float mf) {

    acc = new Vector3D(0,0);

    vel = new Vector3D(random(-2,2),random(-2,2));

    loc = l.copy();

    r = 2.0f;

    maxspeed = ms;

    maxforce = mf;

  }

  

  void run(ArrayList boids) {

    flock(boids);

    update();

    borders();

    render();

  }



  // We accumulate a new acceleration each time based on three rules

  void flock(ArrayList boids) {

    Vector3D sep = new Vector3D(0,0,0);
    int sepCount = 0;

    Vector3D ali = new Vector3D(0,0,0);
    int aliCount = 0;

    Vector3D coh = new Vector3D(0,0,0);
    int cohCount = 0;

    for (int i = 0; i < boids.size(); i++) {

      Boid other = (Boid) boids.get(i);

      if (!loc.isNear(loc, other.loc, neighbordist)) {
        continue;
      }

      float d = loc.distance(loc, other.loc);

      if (d > neighbordist || d == 0) {
        continue;
      }

      // Separation
      if (d < desiredseparation) {
        // Calculate vector pointing away from neighbor
        Vector3D diff = loc.sub(loc,other.loc);
        diff.normalize();
        diff.div(d);        // Weight by distance
        sep.add(diff);
        sepCount++;            // Keep track of how many
      }

      ali.add(other.vel);
      aliCount++;

      coh.add(other.loc);
      cohCount++;

    }

    // Separation
    if (sepCount > 0) {
      sep.div((float)sepCount);
    }
    sep.mult(3.0f);

    // Alignment
    if (aliCount > 0) {
      ali.div((float)aliCount);
      ali.limit(maxforce);
    }
    ali.mult(1.5f);

    // Cohesion
    if (cohCount > 0) {
      coh.div((float)cohCount);
      coh = steer(coh, false);
    }

    // Add the force vectors to acceleration

    acc.add(sep);

    acc.add(ali);

    acc.add(coh);


    // Add force towards or against mouse
    Vector3D mousePos = new Vector3D(myMouseX, myMouseY, 0);
    if (!mouseActive) {
      mousePos.sub(loc);
      mousePos.normalize();
      mousePos.mult(0.04f);
      acc.add(mousePos);
    } else {
      if (loc.isNear(loc, mousePos, (float)(max(width, height) / 3)) && loc.distance(loc, mousePos) < (float)(max(width, height) / 3)) {
        mousePos.sub(loc);
        mousePos.mult(-1f);
        mousePos.normalize();
        acc.add(mousePos);
      }
    }

    // Add randomness
    //Vector3D randomness = new Vector3D(random(-1,1),random(-1,1));
    //randomness.normalize();
    //randomness.mult(0.2f);
    //acc.add(randomness);

  }

  

  // Method to update location

  void update() {

    // Update velocity

    vel.add(acc);

    // Limit speed

    vel.limit(maxspeed);

    loc.add(vel);

    // Reset accelertion to 0 each cycle

    acc.setXYZ(0,0,0);

  }



  void seek(Vector3D target) {

    acc.add(steer(target,false));

  }

 

  void arrive(Vector3D target) {

    acc.add(steer(target,true));

  }



  // A method that calculates a steering vector towards a target

  // Takes a second argument, if true, it slows down as it approaches the target

  Vector3D steer(Vector3D target, boolean slowdown) {

    Vector3D steer;  // The steering vector

    Vector3D desired = target.sub(target,loc);  // A vector pointing from the location to the target

    float d = desired.magnitude(); // Distance from the target is the magnitude of the vector

    // If the distance is greater than 0, calc steering (otherwise return zero vector)

    if (d > 0) {

      // Normalize desired

      desired.normalize();

      // Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)

      if ((slowdown) && (d < 100.0f)) desired.mult(maxspeed*(d/100.0f)); // This damping is somewhat arbitrary

      else desired.mult(maxspeed);

      // Steering = Desired minus Velocity

      steer = target.sub(desired,vel);

      steer.limit(maxforce);  // Limit to maximum steering force

    } else {

      steer = new Vector3D(0,0);

    }

    return steer;

  }

  

  void render() {

    // Draw a triangle rotated in the direction of velocity

    float theta = vel.heading2D() + radians(90);

    fill(255, 255, 255);

    stroke(110, 37, 133);

    pushMatrix();

    translate(loc.x,loc.y);

    rotate(theta);

    beginShape(TRIANGLES);

    vertex(0, -r*2);

    vertex(-r, r*2);

    vertex(r, r*2);

    endShape();

    popMatrix();

  }

  

  // Wraparound

  void borders() {

    if (loc.x < -r) loc.x = width+r;

    if (loc.y < -r) loc.y = height+r;

    if (loc.x > width+r) loc.x = -r;

    if (loc.y > height+r) loc.y = -r;

  }

}

// Simple Vector3D Class 

static class Vector3D {

  float x;

  float y;

  float z;



  Vector3D(float x_, float y_, float z_) {

    x = x_; y = y_; z = z_;

  }



  Vector3D(float x_, float y_) {

    x = x_; y = y_; z = 0f;

  }

  

  Vector3D() {

    x = 0f; y = 0f; z = 0f;

  }



  void setX(float x_) {

    x = x_;

  }



  void setY(float y_) {

    y = y_;

  }



  void setZ(float z_) {

    z = z_;

  }

  

  void setXY(float x_, float y_) {

    x = x_;

    y = y_;

  }

  

  void setXYZ(float x_, float y_, float z_) {

    x = x_;

    y = y_;

    z = z_;

  }



  void setXYZ(Vector3D v) {

    x = v.x;

    y = v.y;

    z = v.z;

  }

  

  float magnitude() {

    return (float) Math.sqrt(x*x + y*y + z*z);

  }



  Vector3D copy() {

    return new Vector3D(x,y,z);

  }



  Vector3D copy(Vector3D v) {

    return new Vector3D(v.x, v.y,v.z);

  }

  

  void add(Vector3D v) {

    x += v.x;

    y += v.y;

    z += v.z;

  }



  void sub(Vector3D v) {

    x -= v.x;

    y -= v.y;

    z -= v.z;

  }



  void mult(float n) {

    x *= n;

    y *= n;

    z *= n;

  }



  void div(float n) {

    x /= n;

    y /= n;

    z /= n;

  }



  void normalize() {

    float m = magnitude();

    if (m > 0) {

       div(m);

    }

  }



  void limit(float max) {

    if (magnitude() > max) {

      normalize();

      mult(max);

    }

  }



  float heading2D() {

    float angle = (float) Math.atan2(-y, x);

    return -1*angle;

  }



  Vector3D add(Vector3D v1, Vector3D v2) {

    Vector3D v = new Vector3D(v1.x + v2.x,v1.y + v2.y, v1.z + v2.z);

    return v;

  }



  Vector3D sub(Vector3D v1, Vector3D v2) {

    Vector3D v = new Vector3D(v1.x - v2.x,v1.y - v2.y,v1.z - v2.z);

    return v;

  }



  Vector3D div(Vector3D v1, float n) {

    Vector3D v = new Vector3D(v1.x/n,v1.y/n,v1.z/n);

    return v;

  }



  Vector3D mult(Vector3D v1, float n) {

    Vector3D v = new Vector3D(v1.x*n,v1.y*n,v1.z*n);

    return v;

  }

  boolean isNear (Vector3D v1, Vector3D v2, float minDist) {
    float dx = v1.x - v2.x;
    float dy = v1.y - v2.y;
    float dz = v1.z - v2.z;
    if (dx > minDist || dy > minDist || dz > minDist) {
      return false;
    }
    return true;
  }

  float distance (Vector3D v1, Vector3D v2) {

    float dx = v1.x - v2.x;

    float dy = v1.y - v2.y;

    float dz = v1.z - v2.z;

    return (float) Math.sqrt(dx*dx + dy*dy + dz*dz);

  }



}