Flock flock;
PImage fish;
float fishSizeNum;
int maxFish = 40;
int max = 0;
ArrayList<Year> years;
Table data;
int selYear = 1945;
int curr_num;
float oneFish;

void setup() {
  fishSizeNum= -10;
  size(1040, 1060);

  // load in data from csv, add it to Year array
  years = new ArrayList<Year>();
  data = loadTable("total_tons_by_year.csv", "header");
  for (TableRow row : data.rows()) {
    int total = row.getInt("Total");
    String year = row.getString("Year");
    Year nYear = new Year(year, total);
    years.add(nYear);
    // record maximum total weight
    if (total > max) {
      max = total;
    }
  }
  flock = new Flock();
  fish = loadImage("SmallBlueTopFish.png");
  
  Year selected_year = years.get(selYear - 1945);
  float num_fish = maxFish * selected_year.total_weight / max;
  oneFish = max / maxFish;
  int num = (int) num_fish;
  curr_num = num;
  // Add an initial set of boids into the system
  for (int i = 0; i < num; i++) {
    flock.addBoid(new Boid(width/2,height/2));
  }
  fill(0);
}

void draw() {
  background(255,255,255);
  textSize(25);
  text(selYear + "", width - 80, 35);
  textSize(15);
  text(" = " + oneFish + " tons of fish", width - 80, 55);
  flock.run();
}

void keyPressed() {
  if (key == CODED) {
    if (keyCode == LEFT && selYear > 1945) {
      selYear--;
    }
    if (keyCode == RIGHT && selYear < 2012) {
      selYear++;
    }
    Year selected_year = years.get(selYear - 1945);
  float num_fish = maxFish * selected_year.total_weight / max;
  int num = (int) num_fish;
  if (curr_num < num) {
    for (int i = 0; i < num - curr_num; i++) {
      flock.addBoid(new Boid(width/2,height/2));
    }
  }
  else if (curr_num > num) {
    for (int i = 0; i < curr_num - num; i++) {
      flock.removeLast();
    }
  }
  curr_num = num;
  }
  
}

// Add a new boid into the System
void mousePressed() {
  flock.addBoid(new Boid(mouseX,mouseY));
}



// The Boid class

class Boid {

  PVector location;
  PVector velocity;
  PVector acceleration;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  float SimSpeed;

    Boid(float x, float y) {
    acceleration = new PVector(0, 0);

    // This is a new PVector method not yet implemented in JS
    // velocity = PVector.random2D();

    // Leaving the code temporarily this way so that this example runs in JS
    float angle = random(TWO_PI);
    velocity = new PVector(cos(angle), sin(angle));


    SimSpeed = 2.0;
    
    location = new PVector(x, y);
    r = 35.0;
    maxspeed = (SimSpeed*2);
    maxforce = (SimSpeed/30.3);
  }

  void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();
    render();
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acceleration.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    PVector sep = separate(boids);   // Separation
    PVector ali = align(boids);      // Alignment
    PVector coh = cohesion(boids);   // Cohesion
    // Arbitrarily weight these forces
    sep.mult(1.5);
    ali.mult(1.0);
    coh.mult(1.0);
    // Add the force vectors to acceleration
    applyForce(sep);
    applyForce(ali);
    applyForce(coh);
  }

  // Method to update location
  void update() {
    // Update velocity
    velocity.add(acceleration);
    // Limit speed
    velocity.limit(maxspeed);
    location.add(velocity);
    // Reset accelertion to 0 each cycle
    acceleration.mult(0);
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, location);  // A vector pointing from the location to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed);

    // Above two lines of code below could be condensed with new PVector setMag() method
    // Not using this method until Processing.js catches up
    // desired.setMag(maxspeed);

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);  // Limit to maximum steering force
    return steer;
  }

  void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading2D() + radians(90);
    // heading2D() above is now heading() but leaving old syntax until Processing.js catches up
     
    pushMatrix();
    translate(location.x, location.y);
    rotate(theta);
    
    //shape( fish, fishSizeNum, fishSizeNum, 30, 30);
    image (fish, 0 , -7);
    popMatrix();
    
  }

  // Wraparound
  void borders() {
    if (location.x < -r) location.x = width+r;
    if (location.y < -r) location.y = height+r;
    if (location.x > width+r) location.x = -r;
    if (location.y > height+r) location.y = -r;
  }

  // Separation
  // Method checks for nearby boids and steers away
  PVector separate (ArrayList<Boid> boids) {
    float desiredseparation = 25.5f;
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(location, other.location);
        diff.normalize();
        diff.div(d);        // Weight by distance
        steer.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div((float)count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // First two lines of code below could be condensed with new PVector setMag() method
      // Not using this method until Processing.js catches up
      // steer.setMag(maxspeed);

      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(maxspeed);
      steer.sub(velocity);
      steer.limit(maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  PVector align (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div((float)count);
      // First two lines of code below could be condensed with new PVector setMag() method
      // Not using this method until Processing.js catches up
      // sum.setMag(maxspeed);

      // Implement Reynolds: Steering = Desired - Velocity
      sum.normalize();
      sum.mult(maxspeed);
      PVector steer = PVector.sub(sum, velocity);
      steer.limit(maxforce);
      return steer;
    } 
    else {
      return new PVector(0, 0);
    }
  }

  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  PVector cohesion (ArrayList<Boid> boids) {
    float neighbordist = 50;
    PVector sum = new PVector(0, 0);   // Start with empty vector to accumulate all locations
    int count = 0;
    for (Boid other : boids) {
      float d = PVector.dist(location, other.location);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(other.location); // Add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return seek(sum);  // Steer towards the location
    } 
    else {
      return new PVector(0, 0);
    }
  }
}




// The Flock (a list of Boid objects)

class Flock {
  ArrayList<Boid> boids; // An ArrayList for all the boids

  Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }

  void run() {
    for (Boid b : boids) {
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }

  void addBoid(Boid b) {
    boids.add(b);
  }
  
  void removeLast() {
    boids.remove(boids.get(boids.size() - 1));
  }

}

class Year {
  String year;
  int total_weight;
  
  Year(String the_year, int weight) {
    year = the_year;
    total_weight = weight;
  }
}
