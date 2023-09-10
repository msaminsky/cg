// Adapted from http://paperjs.org/examples/path-simplification and
// http://paperjs.org/examples/tadpoles/:
// Flocking Processing example by Daniel Schiffman:
// http://processing.org/learning/topics/flocking.html
// Thank you Jurge Lehni & Jonathan Puckey, and all contributors to Paper.js!

// set up the flocking
var Boid = Base.extend({
  initialize: function (position, maxSpeed, maxForce) {
    var strength = Math.random() * 0.5;
    this.acceleration = new Point();
    this.vector = Point.random() * 2 - 1;
    this.position = position.clone();
    this.radius = 30;
    this.maxSpeed = maxSpeed + strength;
    this.maxForce = maxForce + strength;
    this.amount = strength * 10 + 10;
    this.count = 0;
    this.createItems();
  },

  run: function (boids) {
    this.lastLoc = this.position.clone();
    if (!groupTogether) {
      this.flock(boids);
    } else {
      this.align(boids);
    }
    this.borders();
    this.update();
    this.calculateTail();
  },
  calculateTail: function () {
    var segments = this.path.segments,
      shortSegments = this.shortPath.segments;
    var speed = this.vector.length;
    var pieceLength = 5 + speed / 3;
    var point = this.position;
    segments[0].point = shortSegments[0].point = point;
    // Chain goes the other way than the movement
    var lastVector = -this.vector;
    for (var i = 1; i < this.amount; i++) {
      var vector = segments[i].point - point;
      this.count += speed * 10;
      var wave = Math.sin((this.count + i * 3) / 10);
      var sway = lastVector.rotate(90).normalize(wave);
      point += lastVector.normalize(pieceLength) + sway;
      segments[i].point = point;
      if (i < 3) shortSegments[i].point = point;
      lastVector = vector;
    }
    this.path.smooth();
  },

  createItems: function () {
    this.path = new Path({
      strokeColor: "pink",
      strokeWidth: 5,
      strokeCap: "round",
    });
    for (var i = 0; i < this.amount; i++) this.path.add(new Point());

    this.shortPath = new Path({
      strokeColor: "pink",
      strokeWidth: 4,
      strokeCap: "round",
    });
    for (var i = 0; i < Math.min(3, this.amount); i++)
      this.shortPath.add(new Point());
  },

  // We accumulate a new acceleration each time based on three rules
  flock: function (boids) {
    var separation = this.separate(boids) * 3;
    var alignment = this.align(boids);
    var cohesion = this.cohesion(boids);
    this.acceleration += separation + alignment + cohesion;
  },

  update: function () {
    // Update velocity
    this.vector += this.acceleration;
    // Limit speed (vector#limit?)
    this.vector.length = Math.min(this.maxSpeed, this.vector.length);
    this.position += this.vector;
    // Reset acceleration to 0 each cycle
    this.acceleration = new Point();
  },

  seek: function (target) {
    this.acceleration += this.steer(target, false);
  },

  arrive: function (target) {
    this.acceleration += this.steer(target, true);
  },

  borders: function () {
    var vector = new Point();
    var position = this.position;
    var radius = this.radius;
    var size = view.size;
    if (position.x < -radius) vector.x = size.width + radius;
    if (position.y < -radius) vector.y = size.height + radius;
    if (position.x > size.width + radius) vector.x = -size.width - radius;
    if (position.y > size.height + radius) vector.y = -size.height - radius;
    if (!vector.isZero()) {
      this.position += vector;
      var segments = this.path.segments;
      for (var i = 0; i < this.amount; i++) {
        segments[i].point += vector;
      }
    }
  },
  // A method that calculates a steering vector towards a target
  // Takes a second argument, if true, it slows down as it approaches
  // the target
  steer: function (target, slowdown) {
    var steer,
      desired = target - this.position;
    var distance = desired.length;
    // Two options for desired vector magnitude
    // (1 -- based on distance, 2 -- maxSpeed)
    if (slowdown && distance < 100) {
      // This damping is somewhat arbitrary:
      desired.length = this.maxSpeed * (distance / 100);
    } else {
      desired.length = this.maxSpeed;
    }
    steer = desired - this.vector;
    steer.length = Math.min(this.maxForce, steer.length);
    return steer;
  },

  separate: function (boids) {
    var desiredSeperation = 60;
    var steer = new Point();
    var count = 0;
    // For every boid in the system, check if it's too close
    for (var i = 0, l = boids.length; i < l; i++) {
      var other = boids[i];
      var vector = this.position - other.position;
      var distance = vector.length;
      if (distance > 0 && distance < desiredSeperation) {
        // Calculate vector pointing away from neighbor
        steer += vector.normalize(1 / distance);
        count++;
      }
    }
    // Average -- divide by how many
    if (count > 0) steer /= count;
    if (!steer.isZero()) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.length = this.maxSpeed;
      steer -= this.vector;
      steer.length = Math.min(steer.length, this.maxForce);
    }
    return steer;
  },

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  align: function (boids) {
    var neighborDist = 25;
    var steer = new Point();
    var count = 0;
    for (var i = 0, l = boids.length; i < l; i++) {
      var other = boids[i];
      var distance = this.position.getDistance(other.position);
      if (distance > 0 && distance < neighborDist) {
        steer += other.vector;
        count++;
      }
    }

    if (count > 0) steer /= count;
    if (!steer.isZero()) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.length = this.maxSpeed;
      steer -= this.vector;
      steer.length = Math.min(steer.length, this.maxForce);
    }
    return steer;
  },

  // Cohesion
  // For the average location (i.e. center) of all nearby boids,
  // calculate steering vector towards that location
  cohesion: function (boids) {
    var neighborDist = 100;
    var sum = new Point();
    var count = 0;
    for (var i = 0, l = boids.length; i < l; i++) {
      var other = boids[i];
      var distance = this.position.getDistance(other.position);
      if (distance > 0 && distance < neighborDist) {
        sum += other.position; // Add location
        count++;
      }
    }
    if (count > 0) {
      sum /= count;
      // Steer towards the location
      return this.steer(sum, false);
    }
    return sum;
  },
});

var boids = [];
var groupTogether = false;

// Add the boids:
for (var i = 0; i < 200; i++) {
  var position = Point.random() * view.size;
  boids.push(new Boid(position, 10, 0.05));
}

function onFrame(event) {
  for (var i = 0, l = boids.length; i < l; i++) {
    if (groupTogether) {
      var length = (((i + event.count / 30) % l) / l) * path.length;
      var point = path.getPointAt(length);
      if (point) boids[i].arrive(point);
    }
    boids[i].run(boids);
  }
}

// Reposition the heart path whenever the window is resized:
function onResize(event) {
  path.fitBounds(view.bounds);
  path.scale(0.8);
}

var path = new Path();

function onMouseDown(event) {
  // If we produced a path before, deselect it:
  if (path) {
    path.selected = false;
  }

  // Create a new path and set its stroke color to pink:
  path = new Path({
    segments: [event.point],
    strokeColor: "pink",
    // Select the path, so we can see its segment points:
    fullySelected: true,
  });
}

// While the user drags the mouse, points are added to the path
// at the position of the mouse:
function onMouseDrag(event) {
  path.add(event.point);
}

// When the mouse is released, we simplify the path:
function onMouseUp(event) {
  var segmentCount = path.segments.length;

  // When the mouse is released, simplify it:
  path.simplify(10);
}

function onMouseDown(event) {
  groupTogether = !groupTogether;
}

function onKeyDown(event) {
  if (event.key == "space") {
    var layer = project.activeLayer;
    layer.selected = !layer.selected;
    return false;
  }
}
