
// this is main
var mass_spring_system;
var flock;

const canvas_size = 600;
const rad_fac = 0.70;  // sets radius of bubble/rink 
const dx = 1;          // grid spacing
const dt = 0.5;        // timestep

const nnodes  = 100;   // numbers of mass nodes in boundary
const node_mass = 6;  // mass of nodes
const nboids = 200;   // number of boids
const boid_mass = 0.4;  // mass of boids

var ks = 3/dx; // spring constant
const gammas = 0.0;   // spring damping parm

const gamma_node = 0.002;  // damping on nodes (force depends on velocity)

// units force -- boids/nodes interactions
const force_amp = 10.0; // for interactions between boids and nodes
const force_k = 0.2;    // 1/scale for interactions between boids and nodes
const vforce_amp = 0.00;  // damping for boid/node interaction

// boid forces, distances 
const d_repel = 20.;
const d_attract = 20.;  // scale over which attractive force applies
const d_align = 20.; // scale for alignment/steer
const d_rotate = 20.;  // scale for rotational alignment of phi
const d_anglepush = 20.0;  // scale for pushing when close together
const d_vforce = 10; // scale for velocity damping
const d_cohesion = 10; // for cohesion force

// boid forces, amplitudes, units acceleration
const attract_force = 0.1*boid_mass; // amplitude of attract force, not used
const repel_force = 0.5*boid_mass;  // 
const align_force = 0.5*boid_mass; // 
const anglepush_force = 0.02*boid_mass;  // push away or attract according to phases
const rotate_force = 0.003*boid_mass; // align orientations by rotating angles
const vforce_force = 0.1*boid_mass; // velocity damping
const cohesion_force = 0.0*boid_mass; // cohesion

const boidspeed = 1;  // speed of self propulsion for boids
const omega0 = 0.05;  // intrinsic frequency of phases

// for display/draw updates 
const ndt=3;

// sliders 
var anglepush_force_Tslider; 
var d_anglepush_Tslider;
var vforce_force_Tslider; 
var d_vforce_Tslider;
var align_force_Tslider;
var d_align_Tslider;
var repel_force_Tslider;
var d_repel_Tslider;
var attract_force_Tslider;
var d_attract_Tslider;

var gamma_node_Tslider;
var ks_Tslider;

var d_rotate_Tslider;
var rotate_force_Tslider;

const propel_force = 1; //

function setup() {
  createCanvas(canvas_size, canvas_size); 
  let xwidth = width*dx;     // grid size set here!
  let yheight = height*dx;
  let big_radius = rad_fac*xwidth/2;
  
  // create a mass spring system
  mass_spring_system = new Mass_spring_system(nnodes,big_radius,node_mass);
  // mass_spring_system.display_springs(); // display springs
  // mass_spring_system.display_nodes(); // display nodes
  
  // create a flock of boids
  flock = new Flock(nboids);
  flock.display_flock(); // display boids
  
  // interaction between boids and boundary
  let boid_set = flock.boid_set;
  let node_set = mass_spring_system.node_set;
  // boid_node_interact(boid_set,node_set,force_amp,force_k,vforce_amp);
  
  // set up some sliders
  anglepush_force_Tslider = new Tslider(0,'anglep',0,anglepush_force); 
  flock.anglepush_force = anglepush_force;
  d_anglepush_Tslider = new Tslider(1,'d_anglep',0,d_anglepush); 
  flock.d_anglepush = d_anglepush;
  
  rotate_force_Tslider = new Tslider(2,'rotate',0,rotate_force); 
  flock.rotate_force = rotate_force;
  d_rotate_Tslider = new Tslider(3,'d_rotate',0,d_rotate); 
  flock.d_rotate = d_rotate;
  
  vforce_force_Tslider = new Tslider(4,'vforce',0,vforce_force); 
  flock.vforce_force = rotate_force;
  d_vforce_Tslider = new Tslider(5,'d_vforce',0,d_vforce); 
  flock.d_vforce = d_vforce;
  
  align_force_Tslider = new Tslider(6,'align',0,align_force); 
  flock.align_force = align_force;
  d_align_Tslider = new Tslider(7,'d_align',0,d_align); 
  flock.d_align = d_align;
  
  repel_force_Tslider = new Tslider(8,'repel',0,repel_force); 
  flock.repel_force = repel_force;
  d_repel_Tslider = new Tslider(9,'d_repel',0,d_repel); 
  flock.d_repel = d_repel;
}

var dcount=0;  // for centroiding

function draw() {
  let boid_set = flock.boid_set;
  let node_set = mass_spring_system.node_set;
  background(240);
  
  // let n_nod = node_set.length;
  // let n_bd = boid_set.length;
    
  if (nnodes > 2){
     mass_spring_system.display_springs();
     mass_spring_system.display_nodes();
  }
  flock.display_flock();
    
  for (let k=0;k<ndt;k++){ // numbers of timesteps per dispay update
    if (nnodes > 2){
      mass_spring_system.zeroaccel(); // zero node accels
      mass_spring_system.compute_accel();   // compute accelerations on nodes
    }
       
    flock.zeroaccel();   // zero boid accels
    // compute accelerations on boids
    flock.propel();
    flock.align();
      // flock.cohesion();
    flock.attract();
    flock.anglepush(); // adjust forces depending upon relative phases
    // flock.vforce(); // force that depends on velocity differences of nearby boids
    // is a damping force 
    flock.repel();
    flock.protate(); // try to align nearby boids, dphi force, affects phis

    if (nnodes > 2){
      boid_node_interact(boid_set,node_set,force_amp,force_k,vforce_amp); // interactions
    }
    // mass_spring_system.single_timestep(dt); // update nodes
    flock.single_timestep(dt); // update boids
  
    //if (nnodes<2){ //if we have only a few nodes
      flock.borders(); // user periodic boundaries 
    //}
    // else{
    //   dcount++;
    //  if ((dcount%10)==0){  // shift centroid
    //    let centroid = mass_spring_system.centroid();
    //    mass_spring_system.shift(centroid);
    //    flock.shift(centroid);
    //  }
    // }
  }
  flock.anglepush_force = anglepush_force_Tslider.slider.value();
  flock.rotate_force    = rotate_force_Tslider.slider.value();
  flock.vforce_force    = vforce_force_Tslider.slider.value();
  flock.align_force     = align_force_Tslider.slider.value();
  flock.repel_force     = repel_force_Tslider.slider.value();
  flock.d_anglepush     = d_anglepush_Tslider.slider.value();
  flock.d_rotate        = d_rotate_Tslider.slider.value();
  flock.d_vforce        = d_vforce_Tslider.slider.value();
  flock.d_align         = d_align_Tslider.slider.value();
  flock.d_repel         = d_repel_Tslider.slider.value();
  anglepush_force_Tslider.text();
  rotate_force_Tslider.text();
  vforce_force_Tslider.text();
  align_force_Tslider.text();
  repel_force_Tslider.text();
  d_anglepush_Tslider.text();
  d_rotate_Tslider.text();
  d_vforce_Tslider.text();  
  d_align_Tslider.text();
  d_repel_Tslider.text();
}


// exponential short range forces between boids and nodes
// U = force_amp*exp(-force_k*d) where d = distance between
function boid_node_interact(boid_set,node_set,
                             force_amp,force_k,vforce_amp){
  let n_nod = node_set.length;
  let n_bd = boid_set.length;
  
  if (n_nod <2) return;
  if (n_bd <2) return;
  
  for (let i=0;i<n_nod;i++){// loop over nodes
    let nodei = node_set[i];
    for(let j=0;j<n_bd;j++){ // loop over boids
      boidj = boid_set[j];
      let dr = p5.Vector.sub(boidj.position,nodei.position);  // vector between
      let d = dr.mag(); // distance between 
      let dv = p5.Vector.sub(boidj.velocity,nodei.velocity);
      //let v = dv.mag;
      if (d < 3/force_k){
        let drhat = dr.copy();
        drhat.normalize(); //  is a unit vector
        let Force = drhat.copy(); // force in direction between
        Force.mult(-force_amp*exp(-force_k*d));
        let vForce = dv.copy();
        vForce.mult(vforce_amp);// damping force depends on velocity diff
        Force.add(vForce);
        let a_boid = Force.copy();
        let a_node = Force.copy();
        a_boid.div(-boidj.m); // acceleration
        a_node.div( nodei.m); // acceleration
        boidj.acceleration.add(a_boid);
        nodei.acceleration.add(a_node);
        // let theta = atan2(drhat.y,drhat.x)  + PI/2;  
        // tan is zero if vector between is || to orientation of rod
        // boidj.phidot += 0.5*sin(2*(theta-boidj.phi));        
      }
    }
  }
  
}
