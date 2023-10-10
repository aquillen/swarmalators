
// contains boids and flocks, here boids are actually swarmalators 

// globals used in Boid constructor: boid_mass, omega0 which is intrinsic frequency 
function Boid(dxx) { // argument dxx used in display
  // constructor
  this.acceleration = createVector(0,0);
  let vw = 0.1;
  this.velocity = createVector(random(-vw,vw),random(-vw,vw));
  let xw = 130;
  this.position = createVector(random(-xw,xw),random(-xw,xw));
  
  // boid contains a phase phi, because it's a swarmalator
  this.phi = random(0,2*PI); 
  this.phidot = random(0,1);
  this.omega0 = omega0; // intrinsic velocity 
  
  this.r = 5.0;  // for display and in pixels
  this.m = boid_mass;
  this.dx = dxx; // scale multiply by this to get pixels, used in display
  

}

//  Boid stuff originally from p5 examples
Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  var theta = this.velocity.heading() + radians(90);
  push();
  colorMode(HSB, 360);// HSB = Hue Saturation Brightness
  let phi_deg = int(this.phi*180/PI); 
  fill(phi_deg,360,360); // choose a color based on value of phi in degrees
  noStroke();
  translate(0,0);
  // w.r. to center of screen
  translate(width/2+this.position.x*this.dx,height/2+this.position.y*this.dx);
  
  // draw a triangle in direction that boid is moving
  rotate(theta); 
  beginShape();  // draw triangle
  vertex(0, -this.r*2);
  vertex(-this.r, this.r*2);
  vertex(this.r, this.r*2);
  endShape(CLOSE);
  rotate(-theta);
  pop();
      
}


// wrap boids in display, but note that forces are not yet wrapped.
Boid.prototype.borders = function(){
  //print(this.dx);
  let wreal = (width + this.r)/this.dx ;
  let hreal = (height+ this.r)/this.dx ;
  if (this.position.x > 0.5*wreal){
    this.position.x -= wreal;
  }
  if (this.position.x < -0.5*hreal){
    this.position.x += wreal;
  }
  if (this.position.y > 0.5*wreal){
    this.position.y -= hreal;
  }
  if (this.position.y < -0.5*hreal){
    this.position.y += hreal;
  }
}

// a flock of boids
// globals used in constructor:  boidspeed, dx,
//    d_attract, d_repel, d_align, d_rotate, d_anglepush, d_vforce, d_cohesion
//    attract_force, repel_force, align_force, rotate_force, anglepush_force, vforce_force
//    cohesion_force, propel_force
function Flock(nboids) {
   // An array for all the boids
   this.boid_set = []; // Initialize the array of boids
   this.dx = dx;  // used in display
    
   this.boidspeed = boidspeed; // relevant for propulsion
   
   this.rsoft = 2.0;  // softening length
  
     // distances
   this.d_attract   = d_attract; // distance for attraction/repulsion forces between Boids
   this.d_repel   = d_repel;  // distance for attraction/repulsion forces between Boid
   this.d_align   = d_align;  // distance for alignment
   this.d_rotate =   d_rotate;  // for rotation force (vary phases based on particles)
   this.d_anglepush = d_anglepush;  // for anglepush (forces from phases)
   this.d_vforce = d_vforce; // in vforce (damping)
   this.d_cohesion = d_cohesion; // in cohesion force
  
   // strengths, units acceleration
   this.attract_force = attract_force;  // for attact force size
   this.repel_force   = repel_force;    // for repel force size
   this.align_force   = align_force;   // allign/cohesion 
   this.rotate_force = rotate_force;  // for rotation
   this.anglepush_force = anglepush_force; // for anglepush
   this.vforce_force = vforce_force;  // for vforce
   this.cohesion_force = cohesion_force;  // for cohesion force
   
   this.propel_force  = propel_force;  // for propel
    
  // create nboids of Boids!
  for (let i=0;i<nboids;i++){
     let b = new Boid(this.dx);
     this.boid_set.push(b);
  }
  
  this.display_flock = function(){  // display boids
    let n = this.boid_set.length;
    for (let i=0;i<n;i++){
      let b = this.boid_set[i];
      b.render();
    }
  }
  // call border wrap function
  this.borders = function(){
    let n = this.boid_set.length;
    for (let i=0;i<n;i++){
      let b = this.boid_set[i];
      b.borders();
    }
  }
  // zero all the accelerations and reset phidot to intrinsic velocity 
  this.zeroaccel = function(){
    let n = this.boid_set.length;
    for(let k = 0; k < n; k++){ // loop over all boids
      let bk = this.boid_set[k];
      bk.acceleration.mult(0); // set acclerations to zero
      bk.phidot = bk.omega0;   // set phidot to intrinsic frequency
    }
  }
  
  // friction/dissipation force that depends on velocity differences for nearby boids
  this.vforce = function() {
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) {  // loop over all pairs of boids
      let bi = this.boid_set[i];
      for (let j = i+1; j <n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.sub(bi.position,bj.position);
        let r_len = dr.mag(); // distance between 
        if (r_len < this.d_vforce){ // distance limit, not weighting by distance
          let dv = p5.Vector.sub(bi.velocity,bj.velocity); // velocity difference
          let Force = dv.copy(); // in direction of velocity difference 
          // Force.normalize();
          Force.mult(this.vforce_force);
          let ai = p5.Vector.mult(Force,-1/bi.m); // acceleration dependson velocity diff
          let aj = p5.Vector.mult(Force, 1/bj.m);
          bi.acceleration.add(ai);
          bj.acceleration.add(aj);
        }
      }
    }
  }
  
  // Repel force depends on inverse distance -- all boid pairs
  // if you change the sign of repel_force then can be attractive
  // there is a distance cutoff d_repel
  // if repel_force >0 then it is is a repel force
  this.repel = function() {
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) {
      let bi = this.boid_set[i];
      for (let j = i+1; j <n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.sub(bi.position,bj.position);
        let r_len = dr.mag();  // length of interboid distance
        if (r_len < this.d_repel){ // distance cutoff
          let drhat = dr.copy();
          drhat = drhat.normalize();  // unit vector for direction
          let Force = drhat.copy();
          let fac = -1.0*this.repel_force*this.d_repel/(r_len + this.rsoft); // normalized here 
          // note use of softening 
          // if this is negative then it is repulsive, otherwise attractive
          // fac *= cos(2*(bi.phi - bj.phi)); 
          Force.mult(fac); 
          let ai = p5.Vector.mult(Force,-1/bi.m);
          let aj = p5.Vector.mult(Force, 1/bj.m);
          bi.acceleration.add(ai);
          bj.acceleration.add(aj);
        }
      // down side of this method is if we have many nearby particles, then
      // acceleration gets high
      }
    }
  }
  
  
  // Attract force depends on inverse distance -- all boid pairs
  // there is a distance cutoff d_attract
  // if attract_force >0 then it is is an attraction force
  this.attract = function() {
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) {
      let bi = this.boid_set[i];
      for (let j = i+1; j <n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.sub(bi.position,bj.position);
        let r_len = dr.mag();  // length of interboid distance
        if (r_len < this.d_attract){ // distance cutoff
          let drhat = dr.copy();
          drhat = drhat.normalize();  // unit vector for direction
          let Force = drhat.copy();
          let fac = 1.0*this.attract_force*this.d_attract/(r_len + this.rsoft); // normalized here 
          // note use of softening 
          // if this is positive then it is attractive
          Force.mult(fac); 
          let ai = p5.Vector.mult(Force,-1/bi.m);
          let aj = p5.Vector.mult(Force, 1/bj.m);
          bi.acceleration.add(ai);
          bj.acceleration.add(aj);
        }
      // down side of this method is if we have many nearby particles, then
      // acceleration gets high
      }
    }
  }
  
  // trying out a new force! appears to attract depending upon phis 
  // cut off via d_anglepush, strength anglepush_force
  this.anglepush = function() {
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) { //loop over boid pairs
      let bi = this.boid_set[i];
      for (let j = i+1; j <n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.sub(bi.position,bj.position); // position difference
        let dv = p5.Vector.sub(bi.velocity,bj.velocity); // velocity difference
        let r_len = dr.mag();  // length of interboid distance
        if (r_len < this.d_anglepush){
          let drhat = dr.copy();
          drhat = drhat.normalize();  // unit vector for direction between particles
          let Force = drhat.copy();
          Force.mult(this.anglepush_force); 
          let fac = cos(bi.phi- bj.phi);  // attracts when angles similar 
          Force.mult(fac); 
          
          let ai = p5.Vector.mult(Force,-1/bi.m);
          let aj = p5.Vector.mult(Force, 1/bj.m);
          bi.acceleration.add(ai);
          bj.acceleration.add(aj);
        }
      }
    }
  }
    
  // Cohesion force, doing all pairs, attraction force 
  this.cohesion = function() {
     if (this.cohesion_force >0){
        let n = this.boid_set.length;
        for (let i = 0; i < n; i++) { // loop over boid pairs
           let bi = this.boid_set[i];
           let pos_sum = createVector(0,0);
           let count = 0;
           for (let j = 0; j < n; j++) {
              let bj = this.boid_set[j];
              let dr = p5.Vector.sub(bi.position,bj.position); 
              let r_len = dr.mag();  // length of interboid distance
              if ((r_len > 0) && (r_len < this.d_cohesion)){
                  pos_sum.add(bj.position); // sum of positions
                  count++;
              }
            }
            if (count >0){
              pos_sum.div(count);   // is average of nearby boid positions!
              let Force = p5.Vector.sub(pos_sum, bi.position); // target direction
              Force.normalize();
              Force.mult(this.boidspeed); // nudge velocity so that going toward 
              // average of nearby boids position
              Force.sub(bi.velocity);
              let ai = p5.Vector.mult(Force,this.cohesion_force/bi.m);  
              bi.acceleration.add(ai);
            }
         }
     }
  }
  
  // alignment try to stear toward mean velocity of nearby Boids
  // velocity dependent forces here!
  // d_align for range and align_force for strength
  this.align = function() {  
    let n = this.boid_set.length;
    // For every boid in the system, check if it's close to another
    for (let i = 0; i < n; i++) {
      let count = 0;
      let bi = this.boid_set[i];
      // let steer = createVector(0,0);  // from average of nearest neighbor velocities
      let v_ave = createVector(0,0);
      for (let j = 0; j < n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.dist(bi.position,bj.position);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((dr > 0) && (dr < this.d_align)) {
          v_ave.add(bj.velocity);  // sum of velocities of nearby
          count++; // Keep track of how many nearby
        }
      }
      if (count > 0){
        v_ave.normalize();
        v_ave.mult(this.boidspeed);
        let steer = p5.Vector.sub(v_ave,bi.velocity);
        // desired is now the stear 
        // As long as the vector is greater than 0
        if (steer.mag() > 0) {
            steer.mult(this.align_force/bi.m);
            // steer.limit(this.maxforce);
            bi.acceleration.add(steer);  //  only on bi
                }
        }
     }
  }
  
  // try to reach same velocity, for is propto propel_force times 
  // difference in velocity from boidspeed
  this.propel = function(){
    let n = this.boid_set.length;
    for (let i = 0; i < n; i++) { // loop over boids
      let bi = this.boid_set[i];
      let vi = bi.velocity.copy();
      let vmag = vi.mag();  // length
      let vhat = vi.copy(); // unit vector
      vhat.normalize();
      let Force = vhat.copy(); // direction same as velocity
      Force.mult((vmag-this.boidspeed)*this.propel_force);
      let ai = p5.Vector.div(Force,-bi.m);
      bi.acceleration.add(ai);
    }
  }
    
  // seems to rotate according to dphi, giving phi interactions
  // this seems to cause synchronization 
  // distance cutoff d_rotate, strength rotate_force
  this.protate = function(){
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) { // loop over boid pairs
       //let count = 0;
       let bi = this.boid_set[i];
       for (let j = i+1; j < n; j++) {
         let bj = this.boid_set[j];
         let dr = p5.Vector.dist(bi.position,bj.position);
         if (dr<this.d_rotate){ // apply interaction if distance is within this range
             let rforce = sin(1*(bj.phi - bi.phi))*this.rotate_force;
             // this seems to be a standard Kuramoto type of interaction 
             bi.phidot += rforce; // assuming all particles have the same moment of inertia
             bj.phidot -= rforce; // equal and opposite 
         }
       }
     }
   }
   
  // integrate dt
  this.single_timestep = function(dt){
    // this.zeroaccel(); // called in main part of code
    // this.propel(); // propel boids
    // this.repel(); // repel repulsion
    // this.cohesion(); // attraction repulsion
    // this.align();
    // boid_node_interact(boid_set,node_set,
    //        force_amp,force_k,vforce_amp); 
    // apply interactions between boids and nodes
    let n = this.boid_set.length;
    for (let i = 0; i < n; i++) {
      bi = this.boid_set[i];
      let dv = p5.Vector.mult(bi.acceleration,dt); //  acc *dt 
      bi.velocity.add(dv); // 
      let dr = p5.Vector.mult(bi.velocity,dt);  // vel *dt 
      bi.position.add(dr); //
      bi.phi += bi.phidot*dt; // update phase 
      bi.phi = bi.phi%(2*PI);  // modulo 2 pi for phase
    }
  }
   
  // shift all boid positions by some position called centroid
  this.shift = function(centroid){
        let n = this.boid_set.length;
        for(let i = 0;i< n;i++){
            boidi = this.boid_set[i];
            boidi.position.sub(centroid);
        }
    }
  
}
