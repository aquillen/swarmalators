

// create a slider and store a label and position for it so we
// can update text labeling during draw
// each slider should have a different index so they are 
// all in a line
// this routines needs modifying so that there can be more than one row of slides
function Tslider(index,label,min,value){
  this.label = label;
  const slider_len = 80;
  const slider_len_s = '80px';
  const x0 = 10;  // poisition from top left of first one
  const y0 = 10;  // position from top left of first one
  const xsep = 10; // separation between sliders in x direction 
  const ysep = 40; // separation between sliders in y direction 
  let max_xi = int(width/(xsep+slider_len)); // how many sliders will fit horizontally
  let xi = index%max_xi;
  let yi = int(index/max_xi);
  print(yi);
  this.slider_x = xi*(slider_len + xsep) + x0;
  this.slider_y = yi*ysep + y0;
  // this.slider_w = slider_len;
  let max = value*2; // for slider range
  let step = (max-min)/20; // for the numbers of possible values in the slider
  
  // constructor
  this.slider = createSlider(min,max,value,step);
  this.slider.position(this.slider_x, this.slider_y);
  this.slider.style('width',  slider_len_s);
  
}

// label the slider
Tslider.prototype.text = function(){
   fill(0); // black
   text(this.label,this.slider_x + 10,this.slider_y+30); // offset the label
}
