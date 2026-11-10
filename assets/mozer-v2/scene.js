
// WebGL2 guarantees filtering of half-float textures. Float32 linear filtering
// requires OES_texture_float_linear, which is absent on some Safari GPUs.
const mozerHalfSources=new WeakMap();
function mozerUploadTexture(texture){
 if((texture.type===THREE.FloatType||texture.type===THREE.HalfFloatType)&&texture.magFilter!==THREE.NearestFilter){
  let state=mozerHalfSources.get(texture),input=texture.image.data;
  if(input instanceof Float32Array){
   if(!state||state.source!==input)state={source:input,packed:new Uint16Array(input.length)};
  }
  if(state){for(let i=0;i<state.source.length;i++)state.packed[i]=THREE.DataUtils.toHalfFloat(Math.max(-65504,Math.min(65504,state.source[i])));texture.type=THREE.HalfFloatType;texture.image.data=state.packed;mozerHalfSources.set(texture,state);}
 }
 texture.needsUpdate=true;
}

// Position Based Fluids: one population with pressure constraints, viscosity,
// gravity and solid-wall collisions. No droplet emitter or prescribed surface.
class VolumeFluid {
 constructor(seed=0,fillAmount=.26){
  this.h=.25;this.rest=2.6;this.radius=.855;this.spacing=.155;this.accumulator=0;this.clock=0;
  this.force=1;this.amplitude=1;this.viscosity=3.2;this.splash=1;
  const points=[];
  let low=-this.radius,high=this.radius;
  for(let j=0;j<20;j++){const level=(low+high)*.5,h=level+this.radius,fraction=h*h*(3*this.radius-h)/(4*this.radius**3);if(fraction<fillAmount)low=level;else high=level;}
  const fillLevel=(low+high)*.5;
  for(let y=-.81;y<fillLevel;y+=this.spacing)for(let z=-.81;z<.82;z+=this.spacing)for(let x=-.81;x<.82;x+=this.spacing){
   if(x*x+y*y+z*z<this.radius*this.radius)points.push(x+.002*Math.sin(y*37+seed),y,z+.002*Math.cos(x*29+seed));
  }
  this.p=new Float32Array(points);this.old=this.p.slice();this.v=new Float32Array(points.length);this.pred=this.p.slice();this.delta=new Float32Array(points.length);
  this.count=points.length/3;this.lambda=new Float32Array(this.count);this.rho=new Float32Array(this.count);
  this.gridX=14;this.gridY=14;
  this.head=new Int32Array(14**3);this.next=new Int32Array(this.count);this.neighbors=new Int32Array(this.count*80);this.counts=new Uint8Array(this.count);
  this.field=new Float32Array(32**3);this.rebuildNeighbors();this.reconstruct();
 }
 setAmplitude(a){this.amplitude=a;}
 key(x,y,z){return Math.max(0,Math.min(13,Math.floor((x+1.25)/this.h)))+14*Math.max(0,Math.min(13,Math.floor((y+1.25)/this.h)))+196*Math.max(0,Math.min(13,Math.floor((z+1.25)/this.h)));}
 rebuildNeighbors(){
  const p=this.pred,n=this.count,h2=this.h*this.h;this.head.fill(-1);this.counts.fill(0);
  for(let i=0;i<n;i++){const key=this.key(p[i*3],p[i*3+1],p[i*3+2]);this.next[i]=this.head[key];this.head[key]=i;}
  for(let i=0;i<n;i++){
   const k=i*3,key=this.key(p[k],p[k+1],p[k+2]);let count=0;
   for(let z=-1;z<=1;z++)for(let y=-1;y<=1;y++)for(let x=-1;x<=1;x++){
    const bin=key+x+this.gridX*y+this.gridX*this.gridY*z;if(bin<0||bin>=this.head.length)continue;
    for(let j=this.head[bin];j>=0;j=this.next[j])if(i!==j){
     const l=j*3,dx=p[k]-p[l],dy=p[k+1]-p[l+1],dz=p[k+2]-p[l+2];
     if(dx*dx+dy*dy+dz*dz<h2&&count<80)this.neighbors[i*80+count++]=j;
    }
   }
   this.counts[i]=count;
  }
 }
 constrain(){
  const p=this.pred,n=this.count,h2=this.h*this.h,rest=this.rest;
  for(let i=0;i<n;i++){
   const k=i*3;let density=1,gx=0,gy=0,gz=0,grad2=0;
   for(let q=0;q<this.counts[i];q++){
    const j=this.neighbors[i*80+q],l=j*3,dx=p[k]-p[l],dy=p[k+1]-p[l+1],dz=p[k+2]-p[l+2],r2=dx*dx+dy*dy+dz*dz;
    if(r2>=h2)continue;const w=1-r2/h2;density+=w*w*w;
    const g=-6*w*w/(h2*rest),x=dx*g,y=dy*g,z=dz*g;gx+=x;gy+=y;gz+=z;grad2+=x*x+y*y+z*z;
   }
   this.rho[i]=density;this.lambda[i]=-Math.max(0,density/rest-1)/(grad2+gx*gx+gy*gy+gz*gz+.8);
  }
  this.delta.fill(0);
  for(let i=0;i<n;i++){
   const k=i*3;let sx=0,sy=0,sz=0;
   for(let q=0;q<this.counts[i];q++){
    const j=this.neighbors[i*80+q],l=j*3,dx=p[k]-p[l],dy=p[k+1]-p[l+1],dz=p[k+2]-p[l+2],r2=dx*dx+dy*dy+dz*dz;
    if(r2>=h2)continue;const w=1-r2/h2,weight=w*w*w;
    const correction=this.lambda[i]+this.lambda[j]-.0007/(.5+this.splash)*Math.pow(weight/.754,4);
    const g=correction*-6*w*w/(h2*rest);sx+=dx*g;sy+=dy*g;sz+=dz*g;
   }
   const scale=Math.min(1,.035/Math.max(.00001,Math.hypot(sx,sy,sz)));this.delta[k]=sx*scale;this.delta[k+1]=sy*scale;this.delta[k+2]=sz*scale;
  }
  for(let k=0;k<p.length;k+=3){
   p[k]+=this.delta[k];p[k+1]+=this.delta[k+1];p[k+2]+=this.delta[k+2];
   this.collide(p,k);
  }
 }
 collide(p,k){const r=Math.hypot(p[k],p[k+1],p[k+2]);if(r>this.radius){const f=this.radius/r;p[k]*=f;p[k+1]*=f;p[k+2]*=f;}}
 accelerate(p,v,k,dt,ax,az,omega,drive){
  v[k]+=(-omega*p[k+1]-v[k])*drive*dt-ax*dt;
  v[k+1]+=((omega*p[k]-v[k+1])*drive-3)*dt;
  v[k+2]+=-az*dt;
 }
 step(elapsed,ax=0,az=0,spin=0){
  this.accumulator+=Math.min(.05,elapsed);const dt=1/60,p=this.p,v=this.v;
  while(this.accumulator>=dt){
   this.accumulator-=dt;this.clock+=dt;this.old.set(p);
   const omega=spin*1.75*Math.min(1,this.clock/2.5),drive=1.5*this.force*(1+.32*this.amplitude*Math.sin(this.clock*4));
   for(let k=0;k<p.length;k+=3){
    const viscousDrag=Math.exp(-.055*this.viscosity*this.viscosity*dt);
    v[k]*=viscousDrag;v[k+1]*=viscousDrag;v[k+2]*=viscousDrag;
    // Torque about the view axis; particles climb walls through momentum and
    // the spherical collision constraint, rather than a drawn circular path.
    this.accelerate(p,v,k,dt,ax,az,omega,drive);
    this.pred[k]=p[k]+v[k]*dt;this.pred[k+1]=p[k+1]+v[k+1]*dt;this.pred[k+2]=p[k+2]+v[k+2]*dt;
   }
   this.rebuildNeighbors();for(let iteration=0;iteration<3;iteration++)this.constrain();
   for(let k=0;k<p.length;k++){v[k]=(this.pred[k]-p[k])/dt;p[k]=this.pred[k];}
   // Symmetric XSPH viscosity smooths local velocity while retaining circulation.
   this.delta.fill(0);
   const strength=Math.min(.85,1-Math.exp(-this.viscosity*this.viscosity*dt*2)),h2=this.h*this.h;
   for(let i=0;i<this.count;i++){
    const k=i*3;let sx=0,sy=0,sz=0,weights=0;
    for(let q=0;q<this.counts[i];q++){
     const j=this.neighbors[i*80+q],l=j*3,r2=(p[k]-p[l])**2+(p[k+1]-p[l+1])**2+(p[k+2]-p[l+2])**2;
     if(r2>=h2)continue;const w=(1-r2/h2)**3;weights+=w;sx+=(v[l]-v[k])*w;sy+=(v[l+1]-v[k+1])*w;sz+=(v[l+2]-v[k+2])*w;
     if(r2>this.spacing*this.spacing){const cohesion=10*dt/(.08+this.splash*this.splash)*w;sx+=(p[l]-p[k])*cohesion/strength;sy+=(p[l+1]-p[k+1])*cohesion/strength;sz+=(p[l+2]-p[k+2])*cohesion/strength;}
    }
    const f=strength/Math.max(1,weights);this.delta[k]=sx*f;this.delta[k+1]=sy*f;this.delta[k+2]=sz*f;
   }
   for(let k=0;k<v.length;k++)v[k]+=this.delta[k];
  }
 }
 impulse(x,z,strength=.32){
  for(let k=0;k<this.p.length;k+=3){const dx=this.p[k]-x,dz=this.p[k+2]-z,falloff=Math.exp(-(dx*dx+dz*dz)/.3);this.v[k+1]+=strength*falloff;this.v[k]+=strength*.2*falloff;}
 }
 reconstruct(){
  const N=32,scale=(N-1)/2,h=.27,h2=h*h,support=h*scale;this.field.fill(0);
  const alpha=this.clock>0?Math.min(1,this.accumulator/(1/60)):1;
  for(let k=0;k<this.p.length;k+=3){
   const px=this.old[k]+(this.p[k]-this.old[k])*alpha,py=this.old[k+1]+(this.p[k+1]-this.old[k+1])*alpha,pz=this.old[k+2]+(this.p[k+2]-this.old[k+2])*alpha,cx=(px+1)*scale,cy=(py+1)*scale,cz=(pz+1)*scale;
   for(let z=Math.max(0,Math.ceil(cz-support));z<=Math.min(N-1,Math.floor(cz+support));z++){
    const dz=z/scale-1-pz;
    for(let y=Math.max(0,Math.ceil(cy-support));y<=Math.min(N-1,Math.floor(cy+support));y++){
     const dy=y/scale-1-py,remaining=h2-dz*dz-dy*dy;if(remaining<=0)continue;
     const reach=Math.sqrt(remaining)*scale,base=N*y+N*N*z;
     for(let x=Math.max(0,Math.ceil(cx-reach));x<=Math.min(N-1,Math.floor(cx+reach));x++){
      const dx=x/scale-1-px,w=(remaining-dx*dx)/h2;this.field[x+base]+=w*w*w/2.5;
     }
    }
   }
  }
  return this.field;
 }
 mass(){return this.count*this.spacing**3;}
}

// Same PBF pressure, XSPH viscosity and cohesion as VolumeFluid. Only the
// container and external forces change: an open channel with a driven inlet.
class StreamFluid extends VolumeFluid {
 constructor(){
  super(0);
  const points=[];
  for(let x=-4.05;x<4.05;x+=this.spacing)for(let y=-.16;y<.17;y+=this.spacing)for(let z=-.20;z<.21;z+=this.spacing)points.push(x,y,z);
  this.p=new Float32Array(points);this.old=this.p.slice();this.pred=this.p.slice();this.v=new Float32Array(points.length);this.delta=new Float32Array(points.length);
  this.count=points.length/3;this.lambda=new Float32Array(this.count);this.rho=new Float32Array(this.count);
  this.gridX=38;this.gridY=12;this.head=new Int32Array(38*12*8);this.next=new Int32Array(this.count);this.neighbors=new Int32Array(this.count*80);this.counts=new Uint8Array(this.count);
  this.field=new Float32Array(128*32*16);this.flowSpeed=1;this.turbulence=1;this.accumulator=0;this.clock=0;
  for(let k=0;k<this.v.length;k+=3)this.v[k]=.9;
  for(let k=0;k<this.p.length;k+=3){const x=this.p[k];this.p[k+1]+=.075*Math.sin(x*3.1);this.p[k+2]+=.08*Math.sin(x*2.4);}
  this.old.set(this.p);this.pred.set(this.p);this.rebuildNeighbors();
 }
 key(x,y,z){
  if(!this.gridX||this.gridX===14)return super.key(x,y,z);
  return Math.max(0,Math.min(37,Math.floor((x+4.5)/this.h)))+38*Math.max(0,Math.min(11,Math.floor((y+.75)/this.h)))+456*Math.max(0,Math.min(7,Math.floor((z+1)/this.h)));
 }
 collide(p,k){
  // No ceiling: crests can rise and detach freely under pressure and inertia.
  if(this.gridX!==38)return super.collide(p,k);
  const floor=-.24+.08*Math.exp(-(((p[k]+1.2)/.35)**2))+.10*Math.exp(-(((p[k]-2.2)/.40)**2));
  if(p[k+1]<floor)p[k+1]=floor;
  p[k+2]=Math.max(-.48,Math.min(.48,p[k+2]));
 }
 accelerate(p,v,k,dt){
  const x=p[k],y=p[k+1],speed=this.flowSpeed;
  v[k]+=(speed*1.45-v[k])*this.force*1.8*dt;
  v[k+1]-=2.2*dt;
   const swirl=this.swirl||0;v[k+1]+=-p[k+2]*swirl*3*dt;v[k+2]+=(p[k+1]+.12)*swirl*3*dt;
  // The pump adds momentum at the submerged bank. Free droplets then have
  // pressure/cohesion, viscosity and gravity, not a scripted trajectory.
  const submerged=Math.max(0,Math.min(1,(.24-y)/.24));
  const pump=Math.exp(-(((x+1.1)/.36)**2))+Math.exp(-(((x-2.0)/.42)**2));
  const pulse=Math.max(0,Math.sin(this.clock*3.2+x*2.));
  v[k+1]+=pump*submerged*pulse*this.amplitude*this.force*speed*10*dt*this.turbulence;
  v[k+2]+=Math.sin(this.clock*2.+x*3.)*pump*submerged*this.amplitude*speed*2.4*dt*this.turbulence;
  // Momentum differences stretch the same fluid mass into ligaments and droplets.
  v[k]+=.28*Math.sin(this.clock*2.1+x*2.7)*this.turbulence*dt;
  v[k+1]+=Math.sin(this.clock*3.6+x*4.1+p[k+2]*8.)*submerged*this.splash*.22*dt;
 }
 step(dt){
  super.step(dt);
  for(let k=0;k<this.p.length;k+=3){
   const offset=this.p[k]>4.15?-8.3:this.p[k]<-4.15?8.3:0;
   if(offset){this.p[k]+=offset;this.pred[k]+=offset;this.old[k]+=offset;}
  }
 }
 reconstruct(){
  if(this.gridX!==38)return super.reconstruct();
  const nx=128,ny=32,nz=16,h=.22,h2=h*h,sx=127/8.8,sy=31/2.4,sz=15/1.2;
  this.field.fill(0);const alpha=this.clock?Math.min(1,this.accumulator/(1/60)):1;
  for(let k=0;k<this.p.length;k+=3){
   const px=this.old[k]+(this.p[k]-this.old[k])*alpha,py=this.old[k+1]+(this.p[k+1]-this.old[k+1])*alpha,pz=this.old[k+2]+(this.p[k+2]-this.old[k+2])*alpha;
   const cx=(px+4.4)*sx,cy=(py+.6)*sy,cz=(pz+.6)*sz;
   for(let z=Math.max(0,Math.ceil(cz-h*sz));z<=Math.min(nz-1,Math.floor(cz+h*sz));z++){
    const dz=z/sz-.6-pz;
    for(let y=Math.max(0,Math.ceil(cy-h*sy));y<=Math.min(ny-1,Math.floor(cy+h*sy));y++){
     const dy=y/sy-.6-py,remaining=h2-dz*dz-dy*dy;if(remaining<=0)continue;
     const reach=Math.sqrt(remaining)*sx,base=nx*y+nx*ny*z;
     for(let x=Math.max(0,Math.ceil(cx-reach));x<=Math.min(nx-1,Math.floor(cx+reach));x++){
      const dx=x/sx-4.4-px,w=(remaining-dx*dx)/h2;this.field[base+x]+=w*w*w/2.5;
     }
    }
   }
  }
  return this.field;
 }
}

// Shared optical palette for the oil under glass and the background stream.
const oilOptics=`
// Convert display colors back to the radiance expected by finishGold.
vec3 displayRadiance(vec3 rgb){
 vec3 y=pow(clamp(rgb,vec3(0.),vec3(.99)),vec3(2.2));
 vec3 a=2.51-2.43*y,b=.03-.59*y,c=-.14*y;
 return (-b+sqrt(max(vec3(0.),b*b-4.*a*c)))/(2.*a*.9);
}
vec3 canvasRadiance(){return displayRadiance(sampleBackdrop(vUv));}
vec3 oilThroughBackground(float length,vec3 shell,vec3 background){
 float path=length*uDensity*.65;
 float clarity=clamp(uOilTransparency,0.,1.);
 // Midpoint exactly preserves the approved material. Upper half reduces
 // absorption; lower half reduces transmission, without changing canvas alpha.
 float absorptionScale=min(1.,2.*(1.-clarity));
 vec3 depthAbsorption=-log(max(uShadeColor,vec3(.002)));
 float deep=smoothstep(.6,2.6,path)*.18;
 vec3 transmitted=background*exp(-(uAbsorption*path+depthAbsorption*deep)*absorptionScale);
 vec3 body=displayRadiance(pow(clamp(uOilColor,vec3(.001),vec3(.99)),vec3(1./2.2)));
 float opaqueFraction=max(0.,1.-2.*clarity)*smoothstep(0.,.08,length);
 return mix(transmitted,body,opaqueFraction)*shell;
}
vec3 oilTransmission(vec3 ray,float length,vec3 shell){
 vec2 refractedUv=clamp(vUv+ray.xy*.028,vec2(0.),vec2(1.));
 vec3 background=displayRadiance(sampleBackdrop(refractedUv));
 return oilThroughBackground(length,shell,background);
}
vec3 oilInterfaceRadiance(vec3 background,float thickness,vec3 reflected,float fresnel,vec3 shell){
 vec3 transmitted=oilThroughBackground(thickness,shell,background);
 // Sum the first reflection from the two interfaces of a clear oil sheet.
 float interfaceReflection=1.-(1.-fresnel)*(1.-fresnel);
 interfaceReflection=.10+.90*interfaceReflection;
 vec3 reflectedLight=studio(reflected);
 if(uReflectionCeiling>0.){
  float peak=max(reflectedLight.r,max(reflectedLight.g,reflectedLight.b));
  float excess=max(0.,peak-uReflectionCeiling);
  float limited=min(peak,uReflectionCeiling)+excess/(1.+excess/.8);
  reflectedLight*=limited/max(peak,.00001);
 }
 return transmitted*(1.-interfaceReflection)+reflectedLight*interfaceReflection;
}
vec3 goldShellTransmission(float thickness,float facing){
 vec3 transmission=exp(log(max(uGlassColor,vec3(.03)))*thickness*.12);
 return transmission; // Clear center: shell color follows actual optical thickness only.
}
vec3 goldLight(){return mix(uHighlightColor,vec3(1.,.58,.105),.78);}
vec3 goldReflection(vec3 reflected){
 vec3 a=normalize(reflected+vec3(uGlassRoughness,0.,0.)),b=normalize(reflected-vec3(uGlassRoughness,0.,0.));
 return studio(reflected)*.5+studio(a)*.25+studio(b)*.25;
}
vec3 roughGold(vec3 col,vec3 normal){
 float frost=clamp(uGlassRoughness/.12,0.,1.);
 vec3 diffuse=uGlassColor*(.35+.55*max(0.,dot(normal,normalize(vec3(-.45,.7,1.)))))+uHighlightColor*.20;
 col=mix(col,diffuse,.10*frost);
 return col+uGlassColor*studio(normalize(normal+vec3(-.3,.4,0.)))*frost*.045;
}
vec3 finishGold(vec3 col,vec3 reflected){
 // Highlights come only from the reflected studio environment.
 vec3 radiance=max(vec3(0.),col*.9*uExposure-vec3(.025));
 float luminance=max(.00001,dot(radiance,vec3(.2126,.7152,.0722)));
 vec3 chromatic=radiance*(film(vec3(luminance)).x/luminance);
 vec3 mapped=film(radiance); // One filmic curve; no second chroma boost that clipped gold to lemon yellow.
 return pow(clamp(mapped,0.,1.),vec3(1./2.2));
}
`;

const liquidVertex = `
out vec2 vUv;
void main(){vUv=uv;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.);}
`;

const liquidFragment = `
precision highp float;
in vec2 vUv;
out vec4 outColor;
uniform vec2 uView;
uniform vec3 uCenter;
uniform mat3 uRigidInv;
uniform float uCoreRadius;
uniform vec3 uArmBends[4];
uniform vec4 uNodes[4];uniform float uArmScales[4];
uniform vec4 uRoots[5];
uniform vec4 uTips[5];
uniform vec4 uTwigRoots[10];
uniform vec4 uTwigTips[10];
uniform vec4 uDrops[8];
uniform vec3 uDropShapes[8];
uniform float uTime;
uniform float uFocus;
uniform float uScale;
uniform float uOnlyDrops;
uniform float uDropIndex;
uniform float uDrawDrops;
uniform vec3 uHoverColor,uVesselTint[5];uniform float uHoverIntensity,uHoverWeights[5],uVesselTintStrength;
uniform vec3 uShadeColor;
uniform vec3 uHighlightColor;
uniform float uGlassRoughness;

float join(float a,float b,float k){float h=max(k-abs(a-b),0.)/k;return min(a,b)-h*h*k*.25;}
float tube(vec3 p,vec3 a,vec3 b,float ra,float rb){
 vec3 ba=b-a;float t=clamp(dot(p-a,ba)/max(dot(ba,ba),.0001),0.,1.);
 return length(p-a-ba*t)-mix(ra,rb,t);
}
float armParameter(vec3 p,int i){vec3 b=uNodes[i].xyz,v=uArmBends[i];float t=clamp(dot(p,b)/dot(b,b),0.,1.);for(int j=0;j<3;j++){vec3 q=b*t+v*sin(3.14159265*t),d=b+v*(3.14159265*cos(3.14159265*t));t=clamp(t-dot(q-p,d)/max(.001,dot(d,d)),0.,1.);}return t;}
vec3 armPoint(int i,float t){return uNodes[i].xyz*t+uArmBends[i]*sin(3.14159265*t);}
float armTube(vec3 p,int i){float t=armParameter(p,i);return length(p-armPoint(i,t))-mix(.12,.075,t)*uArmScales[i];}
vec3 local(vec3 p){return uRigidInv*(p-uCenter)/uScale;}

// Glass stays rigid; branch anchors respond slightly to the pointer on the CPU.
float field(vec3 world){
 vec3 p=local(world);float d=length(p)-uCoreRadius;
 for(int i=0;i<4;i++){
  vec4 b=uNodes[i];
  float stem=armTube(p,i);
  d=join(d,stem,.24);
  d=join(d,length(p-b.xyz)-b.w,.18);
 }
 for(int i=0;i<5;i++){
  vec4 a=uRoots[i],b=uTips[i];
  float bound=length(p-(a.xyz+b.xyz)*.5)-length(b.xyz-a.xyz)*.5-b.w;
  if(bound<d+.16){
   float stem=tube(p,a.xyz,b.xyz,a.w,.036);
   d=join(d,join(stem,length(p-b.xyz)-b.w,.09),.14);
  }
 }
 for(int i=0;i<10;i++){
  vec4 a=uTwigRoots[i],b=uTwigTips[i];
  float bound=length(p-(a.xyz+b.xyz)*.5)-length(b.xyz-a.xyz)*.5-b.w;
  if(bound<d+.065)d=join(d,join(tube(p,a.xyz,b.xyz,a.w,.015),length(p-b.xyz)-b.w,.045),.055);
 }
 if(uDrawDrops>.5)for(int i=0;i<8;i++)d=min(d,(length((p-uDrops[i].xyz)/uDropShapes[i])-uDrops[i].w)*min(uDropShapes[i].x,uDropShapes[i].y));
 return d*uScale;
}
vec3 normalAt(vec3 p){
 vec2 e=vec2(.001,-.001);
 return normalize(e.xyy*field(p+e.xyy)+e.yyx*field(p+e.yyx)+e.yxy*field(p+e.yxy)+e.xxx*field(p+e.xxx));
}
float spot(vec3 r,vec3 dir,float power){return pow(max(dot(r,normalize(dir)),0.),power);}
uniform vec3 uGlassReferenceColor,uReflectionColor,uReflectionReferenceColor;
uniform vec2 uBackdropUvScale,uBackdropUvOffset;
uniform sampler2D uStudioMap,uBackdropImage;uniform float uStudioStrength,uStudioSoftness,uReflectionCeiling,uOilTransparency;
uniform float uRoundGloss,uRoundGlossSize,uWallTint;
uniform vec3 uEdgeColor;uniform float uEdgeCenter,uEdgeSoftness;
uniform vec4 uGlassSpots[3];uniform vec3 uSpotPowers,uSpotSoftness;
uniform float uAmbient,uKey,uRim,uExposure;uniform vec3 uAmbientColor,uKeyColor,uRimColor;uniform mat3 uLightInv;
vec3 sampleBackdrop(vec2 uv){return texture(uBackdropImage,clamp(uv*uBackdropUvScale+uBackdropUvOffset,vec2(0.),vec2(1.))).rgb;}
vec3 studio(vec3 r){
 r=uLightInv*r;
 vec2 uv=vec2(atan(r.x,r.z)/6.2831853+.5,asin(clamp(r.y,-1.,1.))/3.14159265+.5);
 vec3 lighting=textureLod(uStudioMap,uv,uStudioSoftness+uGlassRoughness*12.).rgb;
 float peak=max(lighting.r,max(lighting.g,lighting.b));
 float low=min(lighting.r,min(lighting.g,lighting.b));
 float colored=smoothstep(.06,.30,(peak-low)/max(peak,.00001));
 vec3 colorRatio=clamp(uReflectionColor/max(uReflectionReferenceColor,vec3(.002)),vec3(.002),vec3(8.));
 // Recolor the hue captured in the reference map, preserving neutral white lights.
 lighting*=pow(colorRatio,vec3(colored));
 vec3 tint=mix(uKeyColor,uRimColor,smoothstep(-.25,.75,r.x));
 return lighting*tint*uStudioStrength*(.45+.45*uKey+.10*uRim)+lighting*uAmbientColor*uAmbient*.08;
}
vec3 film(vec3 c){return clamp((c*(2.51*c+.03))/(c*(2.43*c+.59)+.14),0.,1.);}

// One reconstructed 3D density field for the entire fluid, including spray.
uniform highp sampler3D uWaves;
uniform sampler2D uAirBubbles;
uniform float uAirCount,uAirSize;
uniform sampler2D uLogos;
uniform sampler2D uBackground;

uniform float uAmplitude;
uniform vec3 uAbsorption;
uniform vec3 uOilColor;
uniform float uDensity;
uniform vec3 uGlassColor;
${oilOptics}
float fluidDensity(vec3 point,float sectionId){
 vec3 grid=clamp((point+1.)*.5*31.,vec3(0.),vec3(31.));
 vec3 uv=vec3((grid.x+.5)/32.,(grid.y+.5)/32.,(sectionId*32.+grid.z+.5)/160.);
 return texture(uWaves,uv).r;
}
vec3 toWorld(vec3 v){return vec3(dot(uRigidInv[0],v),dot(uRigidInv[1],v),dot(uRigidInv[2],v));}
float surfaceField(vec3 point,float id,float radius){
 // Fluid occupies the INNER cavity, with a small separation from the glass.
 float cavityRadius=radius*.912;
 float fluidSurface=(.25-fluidDensity(point/cavityRadius,id))*cavityRadius*.24;
 return max(fluidSurface,length(point)-cavityRadius);
}
float cavityField(vec3 point,float id,float radius){return id<5.?surfaceField(point,id,radius):point.y+radius*.28;}
// Analytic glass intersections: no iterative surface search or epsilon speckles.
void sphereHit(vec3 ro,vec3 rd,vec4 ball,float id,inout float best,inout vec4 selected,inout float selectedId,inout vec3 hitNormal,inout float coverage){
 vec3 oc=ro-ball.xyz;float b=dot(oc,rd),disc=b*b-dot(oc,oc)+ball.w*ball.w;
 float aa=max(.000001,fwidth(disc));
 if(disc>=0.){
  float t=-b-sqrt(disc);
  if(t>0.&&t<best){best=t;selected=ball;selectedId=id;hitNormal=normalize(oc+rd*t);coverage=smoothstep(0.,aa,disc);}
 }
}
void stemHit(vec3 ro,vec3 rd,vec3 a,vec3 b,float ra,float rb,inout float best,inout vec4 selected,inout float selectedId,inout vec3 hitNormal,inout float coverage){
 vec3 axis=b-a;float len=length(axis);axis/=len;vec3 oc=ro-a;
 float z=dot(oc,axis),dz=dot(rd,axis),slope=(rb-ra)/len;
 vec3 op=oc-axis*z,dp=rd-axis*dz;float r=ra+slope*z;
 float qa=dot(dp,dp)-slope*slope*dz*dz,qb=dot(op,dp)-r*slope*dz,qc=dot(op,op)-r*r;
 float disc=qb*qb-qa*qc,aa=max(.000001,fwidth(disc));
 if(disc>=0.&&abs(qa)>.00001){
  float t=(-qb-sqrt(disc))/qa,y=z+dz*t;
  if(t>0.&&t<best&&y>0.&&y<len){
   best=t;selected=vec4(a+axis*y,ra+slope*y);selectedId=20.;
   hitNormal=normalize(op+dp*t-axis*(ra+slope*y)*slope);coverage=smoothstep(0.,aa,disc);
  }
 }
}

void junctionBounds(vec3 ro,vec3 rd,vec3 point,float radius,inout float begin,inout float end){
 vec3 offset=ro-point;float along=-dot(offset,rd),disc=along*along-dot(offset,offset)+radius*radius;
 if(disc>0.){float lo=along-sqrt(disc),hi=along+sqrt(disc);if(lo<begin){begin=lo;end=hi;}}
}

void fluidDropHit(vec3 ro,vec3 rd,vec4 ball,vec3 shape,float id,inout float best,inout vec4 selected,inout float selectedId,inout vec3 hitNormal,inout float coverage){
 vec3 oc=(ro-ball.xyz)/shape,ray=rd/shape;
 float a=dot(ray,ray),b=dot(oc,ray),disc=b*b-a*(dot(oc,oc)-ball.w*ball.w);
 if(disc>=0.){
  float t=(-b-sqrt(disc))/a;
  if(t>0.&&t<best){best=t;selected=ball;selectedId=id;hitNormal=normalize((oc+ray*t)/shape);coverage=smoothstep(0.,max(.000001,fwidth(disc)),disc);}
 }
}

void main(){
 vec2 xy=(vUv-.5)*uView;
 vec3 ro=vec3(xy.x,xy.y+.63,3.5),eyeRay=normalize(vec3(0.,-.18,-1.));
 vec3 origin=local(ro),direction=uRigidInv*eyeRay;
 float distance=100.,id=0.,coverage=1.;vec4 section=vec4(0.,0.,0.,uCoreRadius);vec3 localNormal=vec3(0.,0.,1.);
 if(uDrawDrops>.5)for(int i=0;i<8;i++)fluidDropHit(origin,direction,uDrops[i],uDropShapes[i],float(i+12),distance,section,id,localNormal,coverage);
 if(uOnlyDrops<.5){
 // Reject rays outside the rigid vessel envelope before testing junctions.
 vec3 envelope=vec3(5.5,5.5,2.2),reciprocal=1./(direction+vec3(.000001));
 vec3 a=(-envelope-origin)*reciprocal,b=(envelope-origin)*reciprocal;
 vec3 nearBox=min(a,b),farBox=max(a,b);
 if(min(farBox.x,min(farBox.y,farBox.z))<max(0.,max(nearBox.x,max(nearBox.y,nearBox.z))))discard;
 sphereHit(origin,direction,section,0.,distance,section,id,localNormal,coverage);
 for(int i=0;i<4;i++)sphereHit(origin,direction,uNodes[i],float(i+1),distance,section,id,localNormal,coverage);
 for(int i=0;i<5;i++)sphereHit(origin,direction,uTips[i],float(i+5),distance,section,id,localNormal,coverage);
 for(int i=0;i<10;i++){
  sphereHit(origin,direction,uTwigTips[i],10.,distance,section,id,localNormal,coverage);
  stemHit(origin,direction,uTwigRoots[i].xyz,uTwigTips[i].xyz,uTwigRoots[i].w,.015,distance,section,id,localNormal,coverage);
 }
 for(int i=0;i<4;i++){
 if(length(uArmBends[i])<.001)stemHit(origin,direction,vec3(0.),uNodes[i].xyz,.12*uArmScales[i],.075*uArmScales[i],distance,section,id,localNormal,coverage);
 else{
  vec3 padding=abs(uArmBends[i])+vec3(.14),lo=min(vec3(0.),uNodes[i].xyz)-padding,hi=max(vec3(0.),uNodes[i].xyz)+padding;
  vec3 inv=1./(direction+vec3(.000001)),a=(lo-origin)*inv,b=(hi-origin)*inv,mn=min(a,b),mx=max(a,b);
  float t=max(0.,max(mn.x,max(mn.y,mn.z))),end=min(distance,min(mx.x,min(mx.y,mx.z)));
  if(t<end)for(int j=0;j<${MOZER_PROFILE.armSteps};j++){
   vec3 q=origin+direction*t;float sd=armTube(q,i);
   if(sd<.00035){float at=armParameter(q,i),r=mix(.12,.075,at)*uArmScales[i];distance=t;section=vec4(armPoint(i,at),r);id=20.;float e=.0008;localNormal=normalize(vec3(armTube(q+vec3(e,0.,0.),i)-armTube(q-vec3(e,0.,0.),i),armTube(q+vec3(0.,e,0.),i)-armTube(q-vec3(0.,e,0.),i),armTube(q+vec3(0.,0.,e),i)-armTube(q-vec3(0.,0.,e),i)));coverage=1.;break;}
   t+=max(.00025,sd*.8);if(t>end)break;
  }
 }
}
 for(int i=0;i<5;i++)stemHit(origin,direction,uRoots[i].xyz,uTips[i].xyz,uRoots[i].w,.036,distance,section,id,localNormal,coverage);
 // Refine only the small neck regions with a smooth implicit union. The rest
 // of the scene retains fast exact sphere/cone intersections.
 float neckBegin=100.,neckEnd=0.;
 for(int i=0;i<4;i++){
  vec3 axis=normalize(uNodes[i].xyz);
  junctionBounds(origin,direction,axis*(uCoreRadius-.06),.35,neckBegin,neckEnd);
  junctionBounds(origin,direction,uNodes[i].xyz-axis*(uNodes[i].w*.90),.30,neckBegin,neckEnd);
 }
 for(int i=0;i<5;i++){
  junctionBounds(origin,direction,normalize(uRoots[i].xyz)*(uCoreRadius-.02),.24,neckBegin,neckEnd);
  junctionBounds(origin,direction,uTips[i].xyz-normalize(uTips[i].xyz-uRoots[i].xyz)*uTips[i].w,.18,neckBegin,neckEnd);

 }
 for(int i=0;i<10;i++)junctionBounds(origin,direction,uTwigRoots[i].xyz,.095,neckBegin,neckEnd);
 if(neckBegin<distance&&neckEnd>0.){
  float at=max(0.,neckBegin);bool found=false;
  for(int j=0;j<${MOZER_PROFILE.junctionSteps};j++){
   vec3 world=ro+eyeRay*(at*uScale);float sd=field(world)/uScale;
   if(sd<.00035){found=true;break;}
   at+=max(.00025,sd*.85);if(at>min(neckEnd,distance))break;
  }
  if(found&&at<distance){
   distance=at;vec3 point=origin+direction*at;
   localNormal=uRigidInv*normalAt(ro+eyeRay*(at*uScale));coverage=1.;
   float nearest=length(point)-uCoreRadius;section=vec4(0.,0.,0.,uCoreRadius);id=0.;
   for(int i=0;i<4;i++){float d=length(point-uNodes[i].xyz)-uNodes[i].w;if(d<nearest){nearest=d;section=uNodes[i];id=float(i+1);}}
   for(int i=0;i<5;i++){float d=length(point-uTips[i].xyz)-uTips[i].w;if(d<nearest){nearest=d;section=uTips[i];id=float(i+5);}}
   for(int i=0;i<9;i++){
    vec3 a,b;float ra,rb;
    if(i<4){a=vec3(0.);b=uNodes[i].xyz;ra=.12*uArmScales[i];rb=.075*uArmScales[i];}else{a=uRoots[i-4].xyz;b=uTips[i-4].xyz;ra=uRoots[i-4].w;rb=.036;}
    vec3 axis=b-a;float t=clamp(dot(point-a,axis)/dot(axis,axis),0.,1.),r=mix(ra,rb,t),d=i<4?armTube(point,i):length(point-a-axis*t)-r;
    if(d<nearest){nearest=d;section=vec4(i<4?armPoint(i,armParameter(point,i)):a+axis*t,r);id=20.;}
   }
  }
 }
 }
 if(distance>90.)discard;
 if(uOnlyDrops>.5 && abs(id-(uDropIndex+12.))>.1)discard;
 vec3 lp=origin+direction*distance,p=ro+eyeRay*(distance*uScale),n=toWorld(localNormal),view=-eyeRay;
 float facing=max(dot(n,view),0.),glassF=.042+.958*pow(1.-facing,5.);
 vec3 micro=vec3(sin(lp.y*37.+lp.z*29.),sin(lp.z*41.+lp.x*23.),sin(lp.x*43.+lp.y*31.));
 vec3 opticalNormal=normalize(n+toWorld(micro)*uGlassRoughness*.15);
 vec3 glassReflection=reflect(-view,opticalNormal),ray=refract(-view,opticalNormal,1./1.50),lr=uRigidInv*ray;
 vec3 oc=lp-section.xyz;
 float travel=max(.001,-2.*dot(oc,lr));
 vec3 exitPoint=p+ray*travel*uScale;
 float radius=section.w*.925,along=-dot(oc,lr),disc=along*along-dot(oc,oc)+radius*radius;
 float enter=disc>0.?max(0.,along-sqrt(max(disc,0.))):0.;
 float leave=disc>0.?min(travel,along+sqrt(max(disc,0.))):0.;
 float innerLength=max(0.,leave-enter),waterLength=innerLength;
 vec3 waveNormal=vec3(0.,1.,0.);float surfaceHit=-1.;
 float conduit=id>19.?1.:0.,tubeRadius=section.w;
 float shellTravel=max(.005,travel-innerLength),totalInternal=0.;
 if(id<12.&&innerLength>.001&&conduit<.5&&totalInternal<.5){
  // Second dielectric boundary: glass -> air or glass -> liquid.
  // Without this boundary a hollow vessel would render like one solid sphere.
  vec3 innerPoint=oc+lr*enter,innerNormal=normalize(innerPoint);
  float submerged=1.-step(0.,cavityField(innerPoint,id,section.w));
  vec3 cavityRay=refract(lr,innerNormal,mix(1.50,1.50/1.47,submerged));
  if(length(cavityRay)<.01){totalInternal=1.;}
  else{
   lr=normalize(cavityRay);oc=innerPoint+lr*.001;
   enter=0.;leave=max(0.,-2.*dot(oc,lr));innerLength=leave;
  }
 }
 if(id<5.&&innerLength>.001&&conduit<.5&&totalInternal<.5){
  // Intersect the refracted eye ray with the simulated, non-planar free surface.
  // The submerged part is a real bounded 3D volume, not a color overlay.
  waterLength=0.;
  float previousT=enter,previousF=surfaceField(oc+lr*enter,id,section.w);
  for(int s=0;s<${MOZER_PROFILE.fluidSamples};s++){
   float nextT=enter+innerLength*(float(s)+1.)/${MOZER_PROFILE.fluidSamples}.;
   float nextF=surfaceField(oc+lr*nextT,id,section.w);
   if(previousF*nextF<0.){
    float lo=previousT,hi=nextT,loF=previousF;
    for(int b=0;b<5;b++){
     float mid=(lo+hi)*.5,mf=surfaceField(oc+lr*mid,id,section.w);
     if(mf*loF>0.){lo=mid;loF=mf;}else hi=mid;
    }
    float hit=(lo+hi)*.5;
    if(surfaceHit<0.)surfaceHit=hit;
    waterLength+=previousF<0.?hit-previousT:nextT-hit;
   }else if(nextF<0.)waterLength+=nextT-previousT;
   previousT=nextT;previousF=nextF;
  }
  if(surfaceHit>=0.){
   vec3 hitPoint=oc+lr*surfaceHit;
   float e=.085*section.w;
   waveNormal=normalize(vec3(
    surfaceField(hitPoint+vec3(e,0.,0.),id,section.w)-surfaceField(hitPoint-vec3(e,0.,0.),id,section.w),
    surfaceField(hitPoint+vec3(0.,e,0.),id,section.w)-surfaceField(hitPoint-vec3(0.,e,0.),id,section.w),
    surfaceField(hitPoint+vec3(0.,0.,e),id,section.w)-surfaceField(hitPoint-vec3(0.,0.,e),id,section.w))+vec3(.000001));
  }
 }
 if(id>=5.&&id<12.&&conduit<.5){
  float level=-section.w*.28;
  float crossing=(level-oc.y)/(abs(lr.y)>.00001?lr.y:.00001);
  if(abs(lr.y)<.00001)waterLength=oc.y<level?innerLength:0.;
  else if(lr.y<0.)waterLength=max(0.,leave-max(enter,crossing));
  else waterLength=max(0.,min(leave,crossing)-enter);
 }
 if(conduit>.5)waterLength=max(0.,travel-tubeRadius*.15);

 vec3 behindNormal=toWorld(normalize(lp-section.xyz+(uRigidInv*ray)*travel));
 vec3 backgroundRay=refract(ray,-behindNormal,1.50);
 if(id<12.&&innerLength>.001&&conduit<.5&&totalInternal<.5){
  vec3 innerExit=oc+lr*leave,innerExitN=normalize(innerExit);
  float submerged=1.-step(0.,cavityField(innerExit,id,section.w));
  vec3 throughGlass=refract(lr,-innerExitN,mix(1.,1.47,submerged)/1.50);
  float b=dot(innerExit,throughGlass);
  float endDistance=-b+sqrt(max(0.,b*b-dot(innerExit,innerExit)+section.w*section.w));
  vec3 outerExit=innerExit+throughGlass*endDistance;
  vec3 outerExitN=normalize(outerExit);
  exitPoint=toWorld(section.xyz+outerExit)*uScale+uCenter;
  vec3 outsideRay=refract(throughGlass,-outerExitN,1.50);
  if(length(outsideRay)>.01)backgroundRay=toWorld(outsideRay);
  else backgroundRay=toWorld(reflect(throughGlass,-outerExitN));
 }
 if(length(backgroundRay)<.1)backgroundRay=reflect(ray,-behindNormal);
 // Clear glass and a pale golden liquid: light is absorbed along the actual water path.
 // Follow the exiting refracted ray to the rear scene plane. Sampling
 // the glass exit itself omits the remaining air path and loses the stream.
 float rearDistance=max(0.,(-1.65-exitPoint.z)/min(-.05,backgroundRay.z));
 vec3 rearPoint=exitPoint+backgroundRay*rearDistance;
 vec2 backdropUv=vec2(rearPoint.x,rearPoint.y-.18*rearPoint.z)/uView+.5;
 vec4 backdrop=texture(uBackground,backdropUv);
 vec2 blur=vec2(uGlassRoughness*.018);
 vec4 softBackdrop=(texture(uBackground,backdropUv+vec2(blur.x,0.))+texture(uBackground,backdropUv-vec2(blur.x,0.))+texture(uBackground,backdropUv+vec2(0.,blur.y))+texture(uBackground,backdropUv-vec2(0.,blur.y)))*.25;
 backdrop=mix(backdrop,softBackdrop,clamp(uGlassRoughness/.12,0.,1.));
 vec3 sceneBackdrop=sampleBackdrop(backdropUv);
 vec3 rearDisplay=mix(sceneBackdrop,backdrop.rgb/max(backdrop.a,.001),backdrop.a);
 vec3 background=displayRadiance(rearDisplay);
 float path=(waterLength/max(section.w,.05))*mix(1.,1.3,conduit)*uDensity*.65*(id>=12.&&id<20.?1.6:1.);
 vec3 transmitted=oilThroughBackground((waterLength/max(section.w,.05))*mix(1.,1.3,conduit)*(id>=12.&&id<20.?1.6:1.),vec3(1.),background);
 // Clear oil is colored by wavelength-dependent absorption, not diffuse body paint.
 // Optical thickness of the actual shell increases strongly near grazing rays.
 float shellPath=shellTravel;
 vec3 wallTransmission=goldShellTransmission(shellPath,facing);
 if(id<4.5)wallTransmission*=mix(vec3(1.),uVesselTint[int(id+.1)],uVesselTintStrength);
 transmitted*=wallTransmission;
 float centerReflection=1.;
 float coatF=.08+.18*uVesselTintStrength;
 float wallReflection=coatF+(1.-coatF)*pow(1.-facing,4.);
 vec3 broad=normalize(glassReflection+vec3(uGlassRoughness,0.,0.));
 vec3 broad2=normalize(glassReflection-vec3(uGlassRoughness,0.,0.));
 vec3 bronzeReflection=goldReflection(glassReflection);
 vec3 col=transmitted*(1.-wallReflection)+bronzeReflection*wallReflection;
 // Back-wall refraction gives the edge its thickness, without painted circular highlights.
 float backF=.042+.958*pow(1.-abs(dot(behindNormal,ray)),5.);
 // Fresnel reflection from the real inner/back wall, attenuated by the oil path.
 col+=studio(reflect(ray,-behindNormal))*uGlassColor*backF*.24*exp(-uAbsorption*path);
 if(totalInternal>.5){
  vec3 cavityReflection=reflect(uRigidInv*ray,normalize(oc));
  col=studio(toWorld(cavityReflection))*exp(-uAbsorption*shellPath*.35);
  col=col*(1.-glassF)+studio(glassReflection)*glassF;
 }
 if(surfaceHit>=0.&&totalInternal<.5){
  vec3 wn=toWorld(waveNormal),fluidRay=toWorld(lr);
  if(dot(wn,-fluidRay)<0.)wn=-wn;
  float incidence=abs(dot(wn,-fluidRay));
  float waterF=.036+.964*pow(1.-incidence,5.);
  float oilToAir=surfaceField(oc,id,section.w)<0.?1.47:1./1.47;
  vec3 reflected=reflect(fluidRay,wn),refracted=refract(fluidRay,wn,oilToAir);
  if(length(refracted)<.01)waterF=1.;
  // The reflected light crosses the oil bank twice before reaching the eye.
  float reflectionPath=surfaceField(oc,id,section.w)<0.?surfaceHit*2.:0.;
  vec3 waterReflection=studio(reflected)*exp(-uAbsorption*reflectionPath);
  // Refract the rear scene through the actual free surface before applying front glass.
  vec2 liquidUv=clamp(backdropUv+(refracted.xy-fluidRay.xy)*waterLength*.32/uView,vec2(0.),vec2(1.));
  vec4 liquidRear=texture(uBackground,liquidUv);
  vec3 liquidBackdrop=sampleBackdrop(liquidUv);
  vec3 liquidBackground=displayRadiance(mix(liquidBackdrop,liquidRear.rgb/max(liquidRear.a,.001),liquidRear.a));
  vec3 surfaceOil=oilInterfaceRadiance(liquidBackground,waterLength/max(section.w,.05),reflected,waterF,vec3(1.));
  // Interior first, then both glass interfaces. Never replace an already shaded shell.
  col=surfaceOil*wallTransmission*(1.-wallReflection)+bronzeReflection*wallReflection;
  col+=studio(reflect(ray,-behindNormal))*uGlassColor*backF*.24*exp(-uAbsorption*path);
  // Concentration of refracted rays follows the computed surface slope.
  float slope=length(waveNormal.xz);
  float caustic=pow(max(0.,dot(reflect(fluidRay,wn),normalize(vec3(-.55,.8,1.)))),55.);
  
 }
 // Roughness scatters the transmitted view as well as widening reflections.
 // Even on a uniform white backdrop the tinted diffuse shell stays visible.
 // Surface roughness only broadens reflections; no diffuse fog is added.
 // A thin reflective gold shoulder gives the rim real studio contrast, not a flat color band.
 float coating=0.;
 col=mix(col,goldReflection(glassReflection),coating);

 if(id<4.5){
  // Surface gradient: zero coverage in the center, smoothly rising toward the rim.
  float radial=sqrt(max(0.,1.-facing*facing));
  float edge=smoothstep(uEdgeCenter,1.,radial);
  edge=pow(edge,mix(3.8,.45,clamp(uEdgeSoftness,0.,1.)));
  float coverage=clamp(edge*uWallTint,0.,.92);
  // Map the selected gradient color to radiance directly: never multiply it by a gray backdrop.
  vec3 tintRadiance=displayRadiance(pow(clamp(uEdgeColor,vec3(.001),vec3(.99)),vec3(1./2.2)));
  vec3 tinted=tintRadiance*(.86+.14*max(0.,n.y));
  col=mix(col,tinted,coverage*.72);
  // Three soft elliptical studio highlights, positioned on the visible spherical surface.
  vec2 surface=vec2(dot(n,vec3(1.,0.,0.)),dot(n,normalize(vec3(0.,1.,-.18))));
  float highlights=0.;
  for(int i=0;i<3;i++){
   vec4 spotShape=uGlassSpots[i];
   vec2 q=(surface-spotShape.xy)/max(vec2(.025),spotShape.zw*uRoundGlossSize);
   float d=dot(q,q);
   float feather=mix(2.8,.55,clamp(uSpotSoftness[i],0.,1.));
   highlights+=exp(-pow(d,feather))*uSpotPowers[i];
  }
  // Removed view-space elliptical overlays: reflected environment supplies the highlights.

 }

 // The reference environment already carries its gold material color.
 // Apply the selected glass color as a relative optical filter, preserving
 // the approved default while making color edits visible across the shell.
 if(id<12.||id>19.){
  vec3 glassFilter=clamp(uGlassColor/max(uGlassReferenceColor,vec3(.002)),vec3(.002),vec3(4.));
  float wallWeight=.32+.88*pow(1.-facing,1.4);
  col*=pow(glassFilter,vec3(wallWeight));
 }
 // Resolve submerged air interfaces after the oil surface lighting, before the ink.
 if(id<4.5&&waterLength>.015&&conduit<.5&&totalInternal<.5){
  for(int bi=0;bi<32;bi++){
   if(float(bi)>=uAirCount)break;
   vec4 bubble=texture(uAirBubbles,vec2((float(bi)+.5)/32.,(id+.5)/5.));
   if(bubble.w<=0.)continue;
   vec3 bc=bubble.xyz*section.w*.912;float br=bubble.w*section.w*.912*uAirSize;
   vec3 bo=oc-bc;float bt=-dot(bo,lr),disc=bt*bt-dot(bo,bo)+br*br;
   if(disc>0.&&bt>enter&&bt<leave){
    float hit=bt-sqrt(disc);vec3 bn=normalize(bo+lr*hit);
    float rim=pow(1.-abs(dot(bn,lr)),2.2);
    float aa=max(.000001,fwidth(disc));float cover=smoothstep(0.,aa*1.5,disc);
    vec3 bubbleReflection=studio(toWorld(reflect(lr,bn)));
    float glint=pow(max(0.,dot(bn,normalize(vec3(-.45,.75,.5)))),16.);
    vec3 airColor=col*(1.14-.72*rim)+bubbleReflection*rim*.38+uHighlightColor*glint*1.15;
    airColor*=exp(-uAbsorption*max(0.,hit-enter)*.16);
    col=mix(col,airColor,cover*.90);
   }
  }
 }

 if(id<4.5){
  float lightAmount=uHoverWeights[int(id+.1)]*uHoverIntensity;
  col+=uHoverColor*lightAmount*(.12+.85*pow(1.-facing,2.)+.4*spot(glassReflection,vec3(-.25,.55,1.),22.));
 }
 // A curved ink print in vessel-local spherical coordinates. It follows
 // rotations, growing radii, self-occlusion and the glass lighting.
 if(id<4.5){
  vec3 decalNormal=normalize(lp-section.xyz);
  vec3 worldDecalNormal=toWorld(decalNormal);
  vec3 cameraDecalNormal=vec3(worldDecalNormal.x,dot(worldDecalNormal,normalize(vec3(0.,1.,-.18))),dot(worldDecalNormal,normalize(vec3(0.,.18,1.))));
  // Reorient the curved print towards the viewer as this flask is selected.
  decalNormal=normalize(mix(decalNormal,cameraDecalNormal,uHoverWeights[int(id+.1)]));
  vec2 decalUv=vec2(atan(decalNormal.x,decalNormal.z),asin(clamp(decalNormal.y,-1.,1.)))/2.2+.5;
  if(decalNormal.z>0.&&all(greaterThan(decalUv,vec2(0.)))&&all(lessThan(decalUv,vec2(1.)))){
   vec4 ink=texture(uLogos,vec2(decalUv.x,(4.-id+decalUv.y)/5.));
   vec3 inkColor=pow(ink.rgb,vec3(2.2));
   if(id<.5)inkColor=vec3(.001);
   vec3 printLight=inkColor*(.65+.35*facing)+bronzeReflection*wallReflection*.015;
   col=mix(col,printLight,ink.a);
  }
 }
 col=finishGold(col,glassReflection);
 outColor=vec4(col,coverage);
}
`;


function createDropFragment(source){
 let out=source.replace(/\/\*[\s\S]*?\*\//g,'').replace(/\/\/[^\n]*/g,'');
 function removeBlock(prefix){
  let start;while((start=out.indexOf(prefix))>=0){const begin=out.indexOf('{',start),statementEnd=out.indexOf(';',start);if(statementEnd>=0&&(begin<0||statementEnd<begin)){out=out.slice(0,start)+out.slice(statementEnd+1);continue;}let depth=1,end=begin+1;
   while(depth&&end<out.length){if(out[end]==='{')depth++;else if(out[end]==='}')depth--;end++;}
   if(depth)throw Error('Unbalanced GLSL '+prefix);out=out.slice(0,start)+out.slice(end);
  }
 }
 for(const prefix of ['if(uOnlyDrops<.5)','if(id<12.&&innerLength','if(id<5.&&innerLength','if(id>=5.&&id<12.&&conduit<.5)','if(totalInternal>.5)','if(surfaceHit>=0.&&totalInternal<.5)','if(id<4.5&&waterLength','if(id<4.5)','if(id<12.||id>19.)'])removeBlock(prefix);
 for(const name of ['join','tube','armParameter','armPoint','armTube','field','normalAt','fluidDensity','surfaceField','cavityField','sphereHit','stemHit','junctionBounds']){
  const match=out.match(new RegExp('(?:float|vec3|void) '+name+'\\('));if(match)removeBlock(match[0]);
 }
 return out;
}
const dropFragment=createDropFragment(liquidFragment);

function createBackgroundFlow(THREE){
 const scene=new THREE.Scene();
 function freshFluid(){return new RenderFluid('stream');}
 let fluid=freshFluid(),lastTime=0;
 const texture=new THREE.Data3DTexture(fluid.field,128,32,16);
 texture.format=THREE.RedFormat;texture.type=THREE.FloatType;texture.unpackAlignment=1;
 texture.minFilter=texture.magFilter=THREE.LinearFilter;texture.generateMipmaps=false;mozerUploadTexture(texture);
 const boundsData=new Float32Array(128*4),boundsTexture=new THREE.DataTexture(boundsData,128,1,THREE.RGBAFormat,THREE.FloatType);
 boundsTexture.minFilter=boundsTexture.magFilter=THREE.NearestFilter;boundsTexture.needsUpdate=true;
 function updateBounds(){
  for(let i=0;i<128;i++){boundsData[i*4]=3;boundsData[i*4+1]=-3;}
  const alpha=fluid.clock?Math.min(1,fluid.accumulator/(1/60)):1;
  for(let k=0;k<fluid.p.length;k+=3){
   const x=fluid.old[k]+(fluid.p[k]-fluid.old[k])*alpha,y=fluid.old[k+1]+(fluid.p[k+1]-fluid.old[k+1])*alpha;
   const lo=Math.max(0,Math.floor((x-.30+4.4)/8.8*128)),hi=Math.min(127,Math.ceil((x+.30+4.4)/8.8*128));
   for(let i=lo;i<=hi;i++){boundsData[i*4]=Math.min(boundsData[i*4],y-.30);boundsData[i*4+1]=Math.max(boundsData[i*4+1],y+.30);}
  }
  boundsTexture.needsUpdate=true;
 }
 updateBounds();
 const curveData=new Float32Array(1024*4);for(let i=0;i<1024;i++){const t=(i+.5)/1024*1.08-.04;curveData[i*4]=-.02+Math.pow(Math.max(0,(t-.48)/.52),1.65)*.30+.11*Math.sin(t*6.283185+.5)+.035*Math.sin(t*12.56637-1.);}
 const curveTexture=new THREE.DataTexture(curveData,1024,1,THREE.RGBAFormat,THREE.FloatType);curveTexture.minFilter=curveTexture.magFilter=THREE.LinearFilter;mozerUploadTexture(curveTexture);
 const uniforms={uCurve:{value:curveTexture},...makeSceneLightingUniforms(THREE),uReflectionCeiling:{value:2.8},uFlowOffset:{value:new THREE.Vector2()},uFlowInv:{value:new THREE.Matrix3()},uFlowScale:{value:new THREE.Vector3(1,1,1)},uBounds:{value:boundsTexture},uView:{value:new THREE.Vector2()},uField:{value:texture},uOilColor:{value:new THREE.Color()},uShadeColor:{value:new THREE.Color()},uHighlightColor:{value:new THREE.Color()},uAbsorption:{value:new THREE.Vector3()},uDensity:{value:2.1},uGlassColor:{value:new THREE.Color()},uGlassRoughness:{value:.03}};
 const studioSource=liquidFragment.slice(liquidFragment.indexOf('float spot('),liquidFragment.indexOf('// One reconstructed'));
 const fragment=`precision highp float;precision highp sampler3D;in vec2 vUv;out vec4 outColor;
 uniform vec2 uView,uFlowOffset;uniform vec3 uFlowScale;uniform mat3 uFlowInv;uniform sampler3D uField;uniform sampler2D uBounds,uCurve;uniform vec3 uOilColor,uShadeColor,uHighlightColor,uAbsorption;uniform float uDensity,uGlassRoughness;uniform vec3 uGlassColor;
 ${studioSource}
 ${oilOptics}
 float density(vec3 p){vec3 uv=vec3((p.x+4.4)/8.8,(p.y+.6)/2.4,(p.z+.6)/1.2);if(any(lessThan(uv,vec3(0.)))||any(greaterThan(uv,vec3(1.))))return 0.;return texture(uField,uv).r;}

 const vec2 flowDesign=vec2(13.1612903226,6.8);
 vec3 gridPoint(vec3 localPoint){
  float t=localPoint.x/(flowDesign.x*1.22)+.5;
  float centerY=texture(uCurve,vec2((t+.04)/1.08,.5)).r*flowDesign.y;
  return vec3((t-.5)*8.3,(localPoint.y-centerY)/(flowDesign.y*.17),localPoint.z/(flowDesign.y*.17));
 }
 float spatialDensity(vec3 localPoint){
  vec3 p=gridPoint(localPoint);
  if(p.x< -4.4||p.x>4.4||p.y< -.6||p.y>1.8||abs(p.z)>.6)return 0.;
  vec2 extent=texture(uBounds,vec2((p.x+4.4)/8.8,.5)).rg;
  if(p.y<extent.x||p.y>extent.y)return 0.;
  return density(p);
 }
 void main(){
  vec2 screen=(vUv-.5)*uView-uFlowOffset;
  vec3 ro=(uFlowInv*vec3(screen,flowDesign.y*2.))/uFlowScale;
  vec3 rd=(uFlowInv*vec3(0.,0.,-1.))/uFlowScale;
  vec3 bounds=vec3(flowDesign.x*.65,flowDesign.y*.85,flowDesign.y*.102);
  vec3 inv=1./(rd+vec3(.000001));
  vec3 ta=(-bounds-ro)*inv,tb=(bounds-ro)*inv;
  vec3 nearT=min(ta,tb),farT=max(ta,tb);
  float begin=max(0.,max(nearT.x,max(nearT.y,nearT.z))),end=min(farT.x,min(farT.y,farT.z));
  if(end<=begin)discard;
  float first=-1.,path=0.,peak=0.,stride=(end-begin)/${MOZER_PROFILE.flowSamples}.;vec3 hit=ro,lastInside=ro;
  for(int i=0;i<${MOZER_PROFILE.flowSamples};i++){
   float at=begin+(float(i)+.5)*stride;vec3 p=ro+rd*at;float d=spatialDensity(p);
   peak=max(peak,d);path+=smoothstep(.22,.28,d)*stride;if(d>.22)lastInside=p;
   if(d>.22&&first<0.){
    float lo=max(begin,at-stride),hi=at;
    for(int j=0;j<5;j++){float mid=(lo+hi)*.5;if(spatialDensity(ro+rd*mid)>.22)hi=mid;else lo=mid;}
    first=(lo+hi)*.5;hit=ro+rd*first;
   }
  }
  float edgeWidth=max(.008,fwidth(peak)*.85),edgeCoverage=smoothstep(.25-edgeWidth,.25+edgeWidth,peak);
  if(first<0.||edgeCoverage<.005)discard;
  float e=.065;
  vec3 localNormal=-normalize(vec3(spatialDensity(hit+vec3(e,0.,0.))-spatialDensity(hit-vec3(e,0.,0.)),spatialDensity(hit+vec3(0.,e,0.))-spatialDensity(hit-vec3(0.,e,0.)),spatialDensity(hit+vec3(0.,0.,e))-spatialDensity(hit-vec3(0.,0.,e)))+vec3(0.,0.,.000001));
  vec3 n=normalize(transpose(uFlowInv)*(localNormal/uFlowScale));
  vec3 view=vec3(0.,0.,1.);if(dot(n,view)<0.)n=-n;float f=.036+.964*pow(1.-abs(dot(n,view)),5.);
  vec3 ray=refract(-view,n,1./1.47),reflection=reflect(-view,n);
  float facing=max(dot(n,view),0.);
  // Shared oil optics. A rear point on the original camera ray is NOT
  // an exit of the refracted ray; using it fabricated total internal reflection.
  // Use the same traced-thickness/front-interface approximation as the vessel fluid.
  vec3 background=oilTransmission(ray,0.,vec3(1.));
  vec3 col=oilInterfaceRadiance(background,path/max(.1,flowDesign.y*.17*uFlowScale.z)*1.35,reflection,f,vec3(1.));
  float slope=length(n.xy);
  
  // Polished oil has no diffuse/frosted material layer.
  outColor=vec4(finishGold(col,reflection),edgeCoverage);
 }`;
 const material=new THREE.ShaderMaterial({uniforms,vertexShader:liquidVertex,fragmentShader:fragment,glslVersion:THREE.GLSL3,transparent:true,depthWrite:false});
 const mesh=new THREE.Mesh(new THREE.PlaneGeometry(1,1),material);mesh.frustumCulled=false;scene.add(mesh);
  const lightPositions=new Float32Array(96*3),lightSeeds=new Float32Array(96);
 for(let i=0;i<96;i++)lightSeeds[i]=i*.71;
 const lightGeometry=new THREE.BufferGeometry();
 lightGeometry.setAttribute('position',new THREE.BufferAttribute(lightPositions,3));
 lightGeometry.setAttribute('seed',new THREE.BufferAttribute(lightSeeds,1));
 const lightUniforms={uLightColor:{value:new THREE.Color()},uTime:{value:0},uPixelScale:{value:1},uBrightness:{value:1},uLayer:{value:0},uBokeh:{value:.65},uGlow:{value:.6}};
 const lightMaterial=new THREE.ShaderMaterial({uniforms:lightUniforms,transparent:true,depthWrite:false,
 vertexShader:`attribute float seed;uniform float uTime,uPixelScale,uBrightness,uLayer,uBokeh;varying float intensity;varying float blur;
 void main(){blur=uBokeh*(.35+.65*fract(seed*1.73));float front=step(.66,fract(seed/.71*.61803398875));intensity=(uLayer<.5?1.-front:front)*uBrightness*(.35+.65*pow(.5+.5*sin(uTime*.65+seed*3.),3.));gl_PointSize=(16.+22.*(.5+.5*sin(seed*7.)))*(1.+blur*1.6)*uPixelScale;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.);}`,
 fragmentShader:`uniform vec3 uLightColor;uniform float uGlow;varying float intensity;varying float blur;void main(){vec2 p=gl_PointCoord*2.-1.;float d=dot(p,p);if(d>1.)discard;float halo=exp(-d*4.)*.18;float core=exp(-d*28.)*.75;float rays=(exp(-abs(p.x)*45.-abs(p.y)*6.)+exp(-abs(p.y)*45.-abs(p.x)*6.))*.30;float soft=exp(-d*mix(24.,5.,blur))*.4;float glow=exp(-d*3.)*uGlow*.17;float alpha=mix(halo+core+rays,soft,blur)+glow;gl_FragColor=vec4(uLightColor,min(.8,intensity*alpha)*(1.-smoothstep(.72,1.,d)));}`});
 const frontMaterial=lightMaterial.clone();frontMaterial.uniforms={...lightUniforms,uLayer:{value:1}};frontMaterial.depthTest=false;const frontParticles=new THREE.Points(lightGeometry,frontMaterial);frontParticles.frustumCulled=false;frontParticles.renderOrder=12;
 const lights=new THREE.Points(lightGeometry,lightMaterial);lights.frustumCulled=false;scene.add(lights);

 let particleTime=0,lastMotionAt=performance.now();
 let uploadedFlowVersion=-1;
 const flowMatrix=new THREE.Matrix4();
 function update(time,viewW,viewH,settings){
  uniforms.uView.value.set(viewW,viewH);uniforms.uFlowOffset.value.set(viewW*settings.waveX/100,viewH*settings.waveY/100);flowMatrix.makeRotationFromEuler(new THREE.Euler(settings.waveTiltX*Math.PI/180,settings.waveTiltY*Math.PI/180,settings.waveAngle*Math.PI/180));uniforms.uFlowInv.value.setFromMatrix4(flowMatrix).invert();uniforms.uFlowScale.value.set(settings.waveWidth,settings.waveHeight,settings.waveDepth);updateSceneLighting(uniforms,settings);lightGeometry.setDrawRange(0,settings.particleCount);lightUniforms.uLightColor.value.set(settings.particleColor);mesh.scale.set(viewW,viewH,1);
  uniforms.uOilColor.value.set(settings.oil);uniforms.uShadeColor.value.set(settings.shade);uniforms.uHighlightColor.value.set(settings.highlight);uniforms.uDensity.value=settings.density;uniforms.uGlassColor.value.set(settings.glass);uniforms.uGlassRoughness.value=settings.roughness;
  const oil=uniforms.uOilColor.value,max=Math.max(oil.r,oil.g,oil.b,.001);
  setOilAbsorption(uniforms.uAbsorption.value,oil);
  fluid.flowSpeed=settings.speed;fluid.force=settings.waveForce;fluid.amplitude=settings.waveAmplitude;fluid.splash=settings.waveSplash;fluid.viscosity=settings.waveViscosity;fluid.turbulence=settings.waveTurbulence;fluid.swirl=settings.waveSwirl;
  if(time>lastTime)fluid.step(Math.min(.05,(time-lastTime)*settings.waveSpeed));if(uploadedFlowVersion!==fluid.version){texture.image.data=fluid.reconstruct();mozerUploadTexture(texture);updateBounds();uploadedFlowVersion=fluid.version;}
  lastTime=time;
    const nowMotion=performance.now();if(!settings.paused)particleTime+=Math.min(.1,(nowMotion-lastMotionAt)/1000)*settings.particleSpeed;lastMotionAt=nowMotion;lightUniforms.uTime.value=particleTime;lightUniforms.uPixelScale.value=settings.particleSize;lightUniforms.uBrightness.value=settings.particleBrightness;lightUniforms.uBokeh.value=settings.cinemaBokeh??.65;lightUniforms.uGlow.value=settings.cinemaGlow??.6;
  for(let i=0;i<96;i++){
   const seed=lightSeeds[i],x=((i*.61803398875)%1-.5)*viewW,y=((i*.41421356+.15)%1-.5)*viewH;
   const spanX=viewW+1.,spanY=viewH+1.;
   const driftX=particleTime*(.22+.12*Math.sin(seed*2.)),driftY=particleTime*(.10+.06*Math.cos(seed*3.));
   lightPositions[i*3]=((x+driftX+settings.particleAmplitude*.25*Math.sin(particleTime*.8+seed)+spanX*.5)%spanX+spanX)%spanX-spanX*.5;
   lightPositions[i*3+1]=((y+driftY+settings.particleAmplitude*.20*Math.cos(particleTime*.65+seed*2.)+spanY*.5)%spanY+spanY)%spanY-spanY*.5;lightPositions[i*3+2]=-4.;
  }
  lightGeometry.attributes.position.needsUpdate=true;
 }
 return {scene,frontParticles,update,reset(){fluid.dispose();fluid=freshFluid();lastTime=0;texture.image.data=fluid.reconstruct();mozerUploadTexture(texture);},ready:()=>fluid.p.length>0,particleCount:()=>fluid.count};
}
function createBackgroundFlowUnused(THREE){
 const scene=new THREE.Scene();
 function freshFluid(){const f=new StreamFluid();f.flowSpeed=fluidDefaults.speed;f.force=fluidDefaults.force;f.amplitude=fluidDefaults.amplitude;f.viscosity=fluidDefaults.viscosity;f.splash=fluidDefaults.splash;f.turbulence=.10;for(let i=0;i<360;i++)f.step(1/60);f.reconstruct();return f;}
 let fluid=freshFluid(),lastTime=0;
 const texture=new THREE.Data3DTexture(fluid.field,128,32,16);
 texture.format=THREE.RedFormat;texture.type=THREE.FloatType;texture.unpackAlignment=1;
 texture.minFilter=texture.magFilter=THREE.LinearFilter;texture.generateMipmaps=false;mozerUploadTexture(texture);
 const boundsData=new Float32Array(128*4),boundsTexture=new THREE.DataTexture(boundsData,128,1,THREE.RGBAFormat,THREE.FloatType);
 boundsTexture.minFilter=boundsTexture.magFilter=THREE.NearestFilter;boundsTexture.needsUpdate=true;
 function updateBounds(){
  for(let i=0;i<128;i++){boundsData[i*4]=3;boundsData[i*4+1]=-3;}
  const alpha=fluid.clock?Math.min(1,fluid.accumulator/(1/60)):1;
  for(let k=0;k<fluid.p.length;k+=3){
   const x=fluid.old[k]+(fluid.p[k]-fluid.old[k])*alpha,y=fluid.old[k+1]+(fluid.p[k+1]-fluid.old[k+1])*alpha;
   const lo=Math.max(0,Math.floor((x-.30+4.4)/8.8*128)),hi=Math.min(127,Math.ceil((x+.30+4.4)/8.8*128));
   for(let i=lo;i<=hi;i++){boundsData[i*4]=Math.min(boundsData[i*4],y-.30);boundsData[i*4+1]=Math.max(boundsData[i*4+1],y+.30);}
  }
  boundsTexture.needsUpdate=true;
 }
 updateBounds();
 const uniforms={uBounds:{value:boundsTexture},uView:{value:new THREE.Vector2()},uField:{value:texture},uOilColor:{value:new THREE.Color()},uShadeColor:{value:new THREE.Color()},uHighlightColor:{value:new THREE.Color()},uAbsorption:{value:new THREE.Vector3()},uDensity:{value:2.1},uGlassColor:{value:new THREE.Color()},uGlassRoughness:{value:.03}};
 const studioSource=liquidFragment.slice(liquidFragment.indexOf('float spot('),liquidFragment.indexOf('// One reconstructed'));
 const fragment=`precision highp float;precision highp sampler3D;in vec2 vUv;out vec4 outColor;
 uniform vec2 uView;uniform sampler3D uField;uniform sampler2D uBounds;uniform vec3 uOilColor,uShadeColor,uHighlightColor,uAbsorption;uniform float uDensity,uGlassRoughness;uniform vec3 uGlassColor;
 ${studioSource}
 ${oilOptics}
 float density(vec3 p){vec3 uv=vec3((p.x+4.4)/8.8,(p.y+.6)/2.4,(p.z+.6)/1.2);if(any(lessThan(uv,vec3(0.)))||any(greaterThan(uv,vec3(1.))))return 0.;return texture(uField,uv).r;}
 void main(){
  vec2 world=(vUv-.5)*uView;float t=world.x/(uView.x*1.22)+.5;
  float centerY=-uView.y*.02+pow(max(0.,(t-.48)/.52),1.65)*uView.y*.30+uView.y*.11*sin(t*6.283185+.5)+uView.y*.035*sin(t*12.56637-1.);
  vec3 p=vec3((t-.5)*8.3,(world.y-centerY)/(uView.y*.17),.60);
  if(p.x< -4.4||p.x>4.4||p.y< -.6||p.y>1.8)discard;
  vec2 extent=texture(uBounds,vec2((p.x+4.4)/8.8,.5)).rg;if(p.y<extent.x||p.y>extent.y)discard;
  float first=-1.,path=0.,peak=0.;vec3 hit=p;
  for(int i=0;i<32;i++){
   p.z=.60-(float(i)+.5)*1.20/32.;float d=density(p);
   peak=max(peak,d);path+=smoothstep(.22,.28,d)*1.20/32.;
   if(d>.22){if(first<0.){
    float lo=p.z,hi=p.z+1.2/32.;
    for(int j=0;j<5;j++){float mid=(lo+hi)*.5;if(density(vec3(p.xy,mid))>.22)lo=mid;else hi=mid;}
    hit=vec3(p.xy,(lo+hi)*.5);first=hit.z;
   }}
  }
  float edgeWidth=max(.008,fwidth(peak)*.85);
  float edgeCoverage=smoothstep(.25-edgeWidth,.25+edgeWidth,peak);
  if(first<0.||edgeCoverage<.005)discard;
  vec3 e=vec3(.055,.060,.060);
  vec3 n=-normalize(vec3(density(hit+vec3(e.x,0.,0.))-density(hit-vec3(e.x,0.,0.)),density(hit+vec3(0.,e.y,0.))-density(hit-vec3(0.,e.y,0.)),density(hit+vec3(0.,0.,e.z))-density(hit-vec3(0.,0.,e.z)))+vec3(0.,0.,.000001));
  vec3 view=vec3(0.,0.,1.);float f=.02+.98*pow(1.-max(dot(n,view),0.),5.);
  vec3 ray=refract(-view,n,1./1.47),reflection=reflect(-view,n);
  float facing=max(dot(n,view),0.);
  vec3 shell=goldShellTransmission(.06,facing);
  vec3 col=oilTransmission(ray,path*uView.y*.17,shell)*(1.-f)+studio(reflection)*f;
  float glassF=.042+.958*pow(1.-facing,5.);
  col+=goldReflection(reflection)*(glassF+.12*pow(1.-facing,1.5)+.025);
  float slope=length(n.xy);
  
  // Polished oil has no diffuse/frosted material layer.
  outColor=vec4(finishGold(col,reflection),edgeCoverage);
 }`;
 const material=new THREE.ShaderMaterial({uniforms,vertexShader:liquidVertex,fragmentShader:fragment,glslVersion:THREE.GLSL3,transparent:true,depthWrite:false});
 const mesh=new THREE.Mesh(new THREE.PlaneGeometry(1,1),material);mesh.frustumCulled=false;scene.add(mesh);
  const lightPositions=new Float32Array(32*3),lightSeeds=new Float32Array(32);
 for(let i=0;i<32;i++)lightSeeds[i]=i*.71;
 const lightGeometry=new THREE.BufferGeometry();
 lightGeometry.setAttribute('position',new THREE.BufferAttribute(lightPositions,3));
 lightGeometry.setAttribute('seed',new THREE.BufferAttribute(lightSeeds,1));
 const lightUniforms={uTime:{value:0},uPixelScale:{value:1}};
 const lightMaterial=new THREE.ShaderMaterial({uniforms:lightUniforms,transparent:true,depthWrite:false,
 vertexShader:`attribute float seed;uniform float uTime,uPixelScale;varying float intensity;
 void main(){intensity=.35+.65*pow(.5+.5*sin(uTime*.65+seed*3.),3.);gl_PointSize=(16.+22.*(.5+.5*sin(seed*7.)))*uPixelScale;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.);}`,
 fragmentShader:`varying float intensity;void main(){vec2 p=gl_PointCoord*2.-1.;float d=dot(p,p);if(d>1.)discard;float halo=exp(-d*4.)*.18;float core=exp(-d*28.)*.75;float rays=(exp(-abs(p.x)*45.-abs(p.y)*6.)+exp(-abs(p.y)*45.-abs(p.x)*6.))*.30;gl_FragColor=vec4(vec3(1.,.79,.28),intensity*(halo+core+rays));}`});
 const lights=new THREE.Points(lightGeometry,lightMaterial);lights.frustumCulled=false;scene.add(lights);

 function update(time,viewW,viewH,settings){
  uniforms.uView.value.set(viewW,viewH);mesh.scale.set(viewW,viewH,1);
  uniforms.uOilColor.value.set(settings.oil);uniforms.uShadeColor.value.set(settings.shade);uniforms.uHighlightColor.value.set(settings.highlight);uniforms.uDensity.value=settings.density;uniforms.uGlassColor.value.set(settings.glass);uniforms.uGlassRoughness.value=settings.roughness;
  const oil=uniforms.uOilColor.value,max=Math.max(oil.r,oil.g,oil.b,.001);
  setOilAbsorption(uniforms.uAbsorption.value,oil);
  fluid.flowSpeed=settings.speed;fluid.force=settings.force;fluid.amplitude=settings.amplitude;fluid.splash=settings.splash;fluid.viscosity=settings.viscosity;fluid.turbulence=.10;
  if(time>lastTime){fluid.step(Math.min(.05,(time-lastTime)*.35));texture.image.data=fluid.reconstruct();mozerUploadTexture(texture);updateBounds();}
  lastTime=time;
    lightUniforms.uTime.value=time;
  for(let i=0;i<32;i++){
   const seed=lightSeeds[i],x=((i*.61803398875)%1-.5)*viewW,y=((i*.41421356+.15)%1-.5)*viewH;
   lightPositions[i*3]=x+.05*Math.sin(time*.19+seed);
   lightPositions[i*3+1]=y+.09*Math.sin(time*.26+seed*2.);lightPositions[i*3+2]=-4.;
  }
  lightGeometry.attributes.position.needsUpdate=true;
 }
 return {scene,update,reset(){fluid=freshFluid();lastTime=0;texture.image.data=fluid.reconstruct();mozerUploadTexture(texture);},particleCount:()=>fluid.count};
}

const fluidDefaults={...MOZER_DEFAULTS};
const fluidSettings={...fluidDefaults,paused:false};
if(MOZER_PROFILE.mobile)Object.assign(fluidSettings,{particleCount:Math.min(MOZER_PROFILE.particles,fluidSettings.particleCount),airBubbleCount:Math.min(MOZER_PROFILE.airBubbles,fluidSettings.airBubbleCount),dofStrength:0,dofBackground:0,renderScale:Math.min(MOZER_PROFILE.maxDpr,fluidSettings.renderScale)});
const controls=document.createElement('div');
controls.innerHTML='<button id="fluid-reset"></button><button id="fluid-stir"></button><button id="fluid-pour"></button>';
const MOZER_ASSET_PROMISES=[];
const MOZER_BACKDROP_SCALE={value:new THREE.Vector2(1,1)};
const MOZER_BACKDROP_OFFSET={value:new THREE.Vector2(0,0)};
const MOZER_BACKDROP={value:null};
let mozerBackdropURL='',mozerBackdropLoading=null;
function loadMozerBackdrop(url){
 if(url===mozerBackdropURL)return mozerBackdropLoading||Promise.resolve();
 mozerBackdropURL=url;
 mozerBackdropLoading=new Promise(resolve=>{
  const loader=new THREE.TextureLoader();loader.setCrossOrigin('anonymous');
  loader.load(url,texture=>{
   const previous=MOZER_BACKDROP.value;MOZER_BACKDROP.value=texture;
   updateMozerBackdrop();if(previous)previous.dispose();resolve();
  },undefined,()=>{console.warn('Background optical reference unavailable');resolve();});
 });return mozerBackdropLoading;
}
function mozerBackdropUniform(){
 if(!MOZER_BACKDROP.value){
  const pixel=new THREE.DataTexture(new Uint8Array([235,230,220,255]),1,1);pixel.needsUpdate=true;MOZER_BACKDROP.value=pixel;
  // Give the embedding page a chance to supply the real Shape image first.
  MOZER_ASSET_PROMISES.push(new Promise(resolve=>setTimeout(()=>{
   loadMozerBackdrop(MOZER_HOST.background?.url||new URL('background-reference.jpg',MOZER_ASSET_ROOT).href).then(resolve);
  },180)));
 }
 return MOZER_BACKDROP;
}
function updateMozerBackdrop(){
 const config=MOZER_HOST.background;
 if(config?.url&&config.url!==mozerBackdropURL){loadMozerBackdrop(config.url);return;}
 const image=MOZER_BACKDROP.value?.image;
 const iw=image?.naturalWidth||image?.width,ih=image?.naturalHeight||image?.height;if(!iw||!ih)return;
 const fw=MOZER_HOST.displayWidth||innerWidth*MOZER_HOST.scale,fh=MOZER_HOST.displayHeight||innerHeight*MOZER_HOST.scale;
 const bw=config?.width||fw,bh=config?.height||fh,position=config?.position||[.5,.5];
 const factor=config?.size==='contain'?Math.min(bw/iw,bh/ih):Math.max(bw/iw,bh/ih);
 let dw=iw*factor,dh=ih*factor;
 const size=(config?.size||'cover').split(/\s+/);
 if(size[0]?.endsWith('%')){dw=bw*parseFloat(size[0])/100;dh=dw*ih/iw;}
 const left=(config?.x||0)+(bw-dw)*position[0],top=(config?.y||0)+(bh-dh)*position[1];
 MOZER_BACKDROP_SCALE.value.set(fw/dw,fh/dh);
 MOZER_BACKDROP_OFFSET.value.set(-left/dw,1-(fh-top)/dh);
}

let studioEnvironmentTexture;
function makeStudioEnvironment(THREE){
 if(studioEnvironmentTexture)return studioEnvironmentTexture;
 const texture=new THREE.DataTexture(new Float32Array([1,.8,.5,1]),1,1,THREE.RGBAFormat,THREE.FloatType);
 texture.wrapS=THREE.RepeatWrapping;texture.minFilter=THREE.LinearMipmapLinearFilter;texture.magFilter=THREE.LinearFilter;texture.generateMipmaps=true;mozerUploadTexture(texture);
 const file='studio.bin',w=256,h=128;
 MOZER_ASSET_PROMISES.push(fetch(new URL(file,MOZER_ASSET_ROOT)).then(r=>{if(!r.ok)throw Error('Studio map '+r.status);return r.arrayBuffer();}).then(buffer=>{texture.image={data:new Float32Array(buffer),width:w,height:h};mozerUploadTexture(texture);}).catch(error=>console.warn('Studio fallback',error)));
 studioEnvironmentTexture=texture;return texture;
}
function setOilAbsorption(target,color){
 // THREE.Color is linear RGB. Keep all three absolute channel values.
 // At a unit optical path a white background transmits the selected swatch.
 target.set(-Math.log(Math.max(.002,color.r)),-Math.log(Math.max(.002,color.g)),-Math.log(Math.max(.002,color.b)));
}
function makeSceneLightingUniforms(THREE){return {uBackdropUvScale:MOZER_BACKDROP_SCALE,uBackdropUvOffset:MOZER_BACKDROP_OFFSET,uReflectionColor:{value:new THREE.Color("#d4ac69")},uReflectionReferenceColor:{value:new THREE.Color("#d4ac69")},uGlassReferenceColor:{value:new THREE.Color("#dfbd79")},uOilTransparency:{value:.5},uReflectionCeiling:{value:0},uBackdropImage:mozerBackdropUniform(THREE),uStudioMap:{value:makeStudioEnvironment(THREE)},uEdgeColor:{value:new THREE.Color('#dbad52')},uEdgeCenter:{value:.35},uEdgeSoftness:{value:.75},uGlassSpots:{value:Array.from({length:3},()=>new THREE.Vector4())},uSpotSoftness:{value:new THREE.Vector3(.75,.85,.85)},uSpotPowers:{value:new THREE.Vector3()},uRoundGloss:{value:1},uRoundGlossSize:{value:1},uWallTint:{value:.65},uStudioStrength:{value:1},uStudioSoftness:{value:1},uAmbient:{value:.8},uKey:{value:1},uRim:{value:1},uExposure:{value:1},uAmbientColor:{value:new THREE.Color()},uKeyColor:{value:new THREE.Color()},uRimColor:{value:new THREE.Color()},uLightInv:{value:new THREE.Matrix3()}};}
function updateSceneLighting(u,s){
 u.uReflectionColor.value.set(s.reflectionColor??"#d4ac69");
 u.uOilTransparency.value=(s.oilTransparency??50)/100;
 u.uEdgeColor.value.set(s.edgeColor??'#dbad52');u.uEdgeCenter.value=s.edgeCenter??.35;u.uEdgeSoftness.value=s.edgeSoftness??.75;
 for(let i=0;i<3;i++)u.uGlassSpots.value[i].set(s['glossX'+i]??0,s['glossY'+i]??.6,s['glossWidth'+i]??.3,s['glossHeight'+i]??.2);
 u.uSpotSoftness.value.set(s.glossSoftness0??.75,s.glossSoftness1??.85,s.glossSoftness2??.85);
 u.uSpotPowers.value.set(s.glossPower0??.9,s.glossPower1??.55,s.glossPower2??.45);
 u.uRoundGloss.value=s.roundGloss??1;u.uRoundGlossSize.value=s.roundGlossSize??1;u.uWallTint.value=s.wallTint??.65;
 u.uStudioStrength.value=s.studioStrength??1;u.uStudioSoftness.value=s.studioSoftness??1;
 u.uAmbient.value=s.lightAmbient;u.uKey.value=s.lightKey;u.uRim.value=s.lightRim;u.uExposure.value=s.lightExposure;
 u.uAmbientColor.value.set(s.lightAmbientColor);u.uKeyColor.value.set(s.lightKeyColor);u.uRimColor.value.set(s.lightRimColor);
 const lightMatrix=new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(s.lightPitch*Math.PI/180,s.lightYaw*Math.PI/180,0));u.uLightInv.value.setFromMatrix4(lightMatrix).invert();
}

// Keep the pressure solver and density reconstruction off the render thread.
const fluidWorkerURL=new URL('fluid-worker.js?v=20261110-ready1',MOZER_ASSET_ROOT).href;
const fluidWorkers=Array.from({length:Math.max(1,Math.min(MOZER_PROFILE.workers,(navigator.hardwareConcurrency||4)-1))},()=>new Worker(fluidWorkerURL));
const workerFluids=new Map();let fluidWorkerId=0;
for(const worker of fluidWorkers)worker.onmessage=({data:m})=>{const f=workerFluids.get(m.id);if(!f)return;Object.assign(f,m);f.version++;f.busy=false;};
let fluidWorkerError='';
for(const worker of fluidWorkers){
 worker.onerror=e=>{fluidWorkerError=e.message||'worker failed';window.dispatchEvent(new Event('mozer-fluid-error'));console.error('Fluid worker',e.message);};
 worker.onmessageerror=()=>{fluidWorkerError='worker message decode failed';window.dispatchEvent(new Event('mozer-fluid-error'));};
}
async function waitForFluid(){
 const began=performance.now();
 while([...workerFluids.values()].some(f=>!f.version)){
  if(fluidWorkerError)throw Error(fluidWorkerError);
  if(performance.now()-began>10000)throw Error('Fluid worker timeout');
  await new Promise(resolve=>setTimeout(resolve,40));
 }
}
class RenderFluid {
 constructor(kind='volume',seed=0){
  this.fillAmount=(fluidSettings.fillAmount??26)/100;this.version=0;
  this.id=++fluidWorkerId;this.worker=fluidWorkers[kind==='stream'?0:(fluidWorkers.length===1?0:1+(Math.round(seed/.7)%(fluidWorkers.length-1)))];this.field=new Float32Array(kind==='stream'?128*32*16:32**3);this.p=new Float32Array(0);this.old=this.p;this.clock=0;this.accumulator=0;this.count=kind==='stream'?477:191;this.busy=true;this.pending=0;
  Object.assign(this,{flowSpeed:fluidDefaults.speed,force:kind==='stream'?fluidDefaults.waveForce:fluidDefaults.force,amplitude:kind==='stream'?fluidDefaults.waveAmplitude:fluidDefaults.amplitude,viscosity:kind==='stream'?fluidDefaults.waveViscosity:fluidDefaults.viscosity,splash:kind==='stream'?fluidDefaults.waveSplash:fluidDefaults.splash,turbulence:fluidDefaults.waveTurbulence,swirl:fluidDefaults.waveSwirl});
  workerFluids.set(this.id,this);this.worker.postMessage({id:this.id,op:'init',kind,seed,settings:this.settings()});
 }
 settings(){const {flowSpeed,force,amplitude,viscosity,splash,turbulence,swirl,fillAmount}=this;return {flowSpeed,force,amplitude,viscosity,splash,turbulence,swirl,fillAmount};}
 setAmplitude(value){this.amplitude=value;}
 step(dt,...args){this.pending=Math.min(.1,this.pending+dt);if(this.busy)return;this.busy=true;const advance=Math.min(.05,this.pending);this.pending-=advance;this.worker.postMessage({id:this.id,op:'step',settings:this.settings(),args:[advance,...args]});}
 reconstruct(){return this.field;}
 impulse(...args){this.worker.postMessage({id:this.id,op:'impulse',args});}
 dispose(){workerFluids.delete(this.id);this.worker.postMessage({id:this.id,op:'dispose'});}
}

mozerReport('scriptsAndUI');
const fpsIndicator=document.createElement('span');
fpsIndicator.id='mozer-fps';
fpsIndicator.style.cssText='position:fixed;top:10px;right:10px;z-index:2147483647;padding:5px 8px;border-radius:5px;background:rgba(0,0,0,.65);color:#fff;font:12px/1.3 monospace;pointer-events:none;';
fpsIndicator.textContent='ready1 · загрузка…';
document.body.append(fpsIndicator);
const stage = document.querySelector('#stage');
const labels = document.querySelector('#labels');
const coreLabel = document.querySelector('#core-label');
const brands = [{"id":"mozer","url":"https://mozer-oil.ru/","name":"MOZER","html":"<img class=\"logo logo-mozer\" alt=\"MOZER\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-0.svg\" />","x":-2.3,"y":1.65},{"id":"michi","url":"https://michioil.com/","name":"MICHI","html":"<img class=\"logo logo-michi\" alt=\"MICHI\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-1.svg\" />","x":2.3,"y":1.65},{"id":"wezer","url":"https://mozergroup.ru/brendy/wezer","name":"WEZER","html":"<img class=\"logo logo-wezer\" alt=\"WEZER\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-2.svg\" />","x":-2.3,"y":-1.65},{"id":"unix","url":"https://mozergroup.ru/brendy/unix","name":"UNIX","html":"<img class=\"logo logo-unix\" alt=\"UNIX\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-3.svg\" />","x":2.3,"y":-1.65}];
const motionPreference = matchMedia('(prefers-reduced-motion: reduce)');
let reducedMotion=false;let systemReducedMotion=motionPreference.matches;
motionPreference.addEventListener('change', e => { systemReducedMotion=e.matches; });

let renderer;
try {
  renderer = new THREE.WebGLRenderer({antialias:false,alpha:true,powerPreference:'high-performance'});
} catch (error) {
  stage.classList.add('no-webgl');
  stage.setAttribute('aria-label','Для анимации требуется браузер с поддержкой WebGL');
  console.error('WebGL unavailable',error);
}
if(renderer){
 mozerReport('renderer');
 stage.dataset.parallelCompile=String(!!renderer.getContext().getExtension('KHR_parallel_shader_compile'));
 stage.prepend(renderer.domElement);
 renderer.domElement.setAttribute('aria-hidden','true');
 const scene=new THREE.Scene();
 const camera=new THREE.OrthographicCamera(-6,6,4,-4,.1,100);
 camera.position.set(0,0,20);
 // Measure render passes separately; a timer covering CPU work overstates GPU cost.
const gpu=renderer.getContext(),timerExt=gpu.getExtension('EXT_disjoint_timer_query_webgl2'),pendingGpu=[];
const passTimes={main:[],background:[]};let perfFrame=0,activeQuery=null;
function startGpuPass(label){if(!timerExt||perfFrame%12||pendingGpu.length>8)return;const q=gpu.createQuery();gpu.beginQuery(timerExt.TIME_ELAPSED_EXT,q);activeQuery={q,label};}
function endGpuPass(){if(!activeQuery)return;gpu.endQuery(timerExt.TIME_ELAPSED_EXT);pendingGpu.push(activeQuery);activeQuery=null;}
function pollGpu(){while(pendingGpu.length&&gpu.getQueryParameter(pendingGpu[0].q,gpu.QUERY_RESULT_AVAILABLE)){const {q,label}=pendingGpu.shift();if(!gpu.getParameter(timerExt.GPU_DISJOINT_EXT)){const ms=gpu.getQueryParameter(q,gpu.QUERY_RESULT)/1e6;passTimes[label].push(ms);if(passTimes[label].length>36)passTimes[label].shift();stage.dataset[label+'GpuMs']=ms.toFixed(2);}gpu.deleteQuery(q);}}
let renderFactor=1,rearFactor=1,perfAt=performance.now(),lastResolutionAt=perfAt,lastRaiseAt=perfAt,renderSetting=fluidSettings.renderScale,frameSamples=[],performanceHistory=[];
let resolutionPending=false,lastRenderSize='';
let targetFps=MOZER_PROFILE.fps,physicsHz=MOZER_PROFILE.lite?30:MOZER_PROFILE.physicsHz,lastPaceChange=performance.now(),paceTrial=false;
let adaptiveBackgroundHz=MOZER_PROFILE.lite?24:MOZER_PROFILE.backgroundHz;
let fluidUploads=0,fluidCountAt=performance.now(),fluidDisplayHz=0;
let paceTested=false;
function choosePace(fps,p95,now){
 if(!MOZER_PROFILE.lite)return;
 if(targetFps===30&&!paceTested){
  const cpu=Number(stage.dataset.cpuMs||0),main=percentile(passTimes.main.slice(-12),.8),rear=percentile(passTimes.background.slice(-12),.8);
  const gpuOK=!passTimes.main.length||main+rear<14;
  if(fps>=28&&p95<38&&cpu<8&&gpuOK){targetFps=60;paceTrial=true;paceTested=true;adaptiveBackgroundHz=30;lastPaceChange=now;}
 }else if(targetFps===60&&now-lastPaceChange>=900){
  if(fps<50||p95>29){targetFps=30;adaptiveBackgroundHz=18;paceTrial=false;lastPaceChange=now;}
  else paceTrial=false;
 }
 stage.dataset.targetFps=String(targetFps);stage.dataset.paceTrial=String(paceTrial);
}
function requestQuality(nextMain,nextRear,now){
 const changed=Math.abs(renderFactor-nextMain)>.001||Math.abs(rearFactor-nextRear)>.001;
 if(changed){renderFactor=nextMain;rearFactor=nextRear;resolutionPending=true;}
 lastResolutionAt=now;lastRaiseAt=now;return changed;
}
function qualityPlan(){return MOZER_RENDER_QUALITY.plan({width,height,scale:MOZER_HOST.scale,dpr:MOZER_HOST.deviceDpr||devicePixelRatio||1,profile:MOZER_PROFILE,renderScale:fluidSettings.renderScale,autoQuality:!!fluidSettings.autoQuality});}
function maximumPixelRatio(){return qualityPlan().maximum;}
function startingQuality(){return qualityPlan().initial;}
function minimumQuality(){return qualityPlan().minimum;}
function applyRenderResolution(){const plan=qualityPlan(),s=MOZER_RENDER_QUALITY.sizes({width,height,plan,mainFactor:renderFactor,rearFactor});const sizeKey=[width,height,s.ratio,s.rearWidth,s.rearHeight].join(':');if(sizeKey===lastRenderSize)return;lastRenderSize=sizeKey;renderer.setPixelRatio(s.ratio);renderer.setSize(width,height);backgroundTarget.setSize(s.rearWidth,s.rearHeight);if(MOZER_PROFILE.postProcess)dofTarget.setSize(s.width,s.height);backgroundAt=-Infinity;stage.dataset.pixelRatio=s.ratio.toFixed(2);stage.dataset.visiblePixelRatio=s.visibleDpr.toFixed(2);stage.dataset.canvasPixels=s.width+'x'+s.height;stage.dataset.backgroundPixels=s.rearWidth+'x'+s.rearHeight;stage.dataset.renderScale=renderFactor.toFixed(2);stage.dataset.rearScale=rearFactor.toFixed(2);}
function percentile(list,fraction){if(!list.length)return 0;const sorted=list.slice().sort((a,b)=>a-b);return sorted[Math.min(sorted.length-1,Math.floor(sorted.length*fraction))];}
function recordFrame(elapsed,now){
 perfFrame++;pollGpu();
 // Background WebViews can throttle rAF to one callback per second while
 // the GPU still renders in a few milliseconds. Do not treat that as load.
 const lastMain=passTimes.main.at(-1)||0,lastRear=passTimes.background.at(-1)||0;
 if(elapsed>250&&Number(stage.dataset.cpuMs||0)<80&&lastMain+lastRear<100){frameSamples=[];perfAt=now;lastResolutionAt=now;stage.dataset.performanceState='background-throttle';fpsIndicator.textContent='— FPS';return;}
 stage.dataset.performanceState='measuring';
 if(renderSetting!==fluidSettings.renderScale){renderSetting=fluidSettings.renderScale;renderFactor=startingQuality();rearFactor=1;resolutionPending=true;}
 if(!fluidSettings.autoQuality&&(renderFactor<1||rearFactor!==1)){renderFactor=1;rearFactor=1;resolutionPending=true;}
 if(elapsed>0&&elapsed<1500)frameSamples.push(elapsed);
 if(now-perfAt<1000||frameSamples.length<8)return;
 const mean=frameSamples.reduce((sum,t)=>sum+t,0)/frameSamples.length,p95=percentile(frameSamples,.95),fps=Math.round(1000/mean);
 performanceHistory.push({fps,p95:Number(p95.toFixed(1)),scale:renderFactor,focus:Number(stage.dataset.focus)});if(performanceHistory.length>20)performanceHistory.shift();stage.dataset.performanceHistory=JSON.stringify(performanceHistory);
 stage.dataset.backgroundHz=String(adaptiveBackgroundHz);stage.dataset.fps=String(fps);stage.dataset.frameP95=p95.toFixed(1);stage.dataset.quality=renderFactor.toFixed(2);stage.dataset.framesOver25=String(frameSamples.filter(t=>t>25).length);
 if(fluidSettings.autoQuality&&now-lastResolutionAt>2200){
  if(fps<targetFps-3||(p95>1500/targetFps&&frameSamples.filter(t=>t>1500/targetFps).length>frameSamples.length*.1)){requestQuality(Math.max(minimumQuality(),renderFactor*.90),Math.max(.85,rearFactor*.94),now);
   // Reduce the expensive rear pass frequency before sacrificing further spatial detail.
   adaptiveBackgroundHz=Math.max(Math.min(MOZER_PROFILE.lite?18:MOZER_PROFILE.backgroundHz,MOZER_PROFILE.mobile?15:20),adaptiveBackgroundHz-5);}
  else if(fps>=targetFps-1&&p95<1150/targetFps&&now-lastRaiseAt>8000&&renderFactor<1&&(lastMain*.5+lastRear*.25)<7){renderFactor=Math.min(1,renderFactor+.03);rearFactor=Math.min(1,rearFactor+.03);resolutionPending=true;lastResolutionAt=now;lastRaiseAt=now;}
 }
 choosePace(fps,p95,now);fluidDisplayHz=Math.round(fluidUploads*1000/Math.max(1,now-fluidCountAt));fluidUploads=0;fluidCountAt=now;stage.dataset.fluidHz=String(fluidDisplayHz);fpsIndicator.textContent='ready1 · '+fps+' FPS / '+targetFps+' · жидкость '+fluidDisplayHz+' Гц'+(paceTrial?' · проба 60':'');
 frameSamples=[];perfAt=now;
}

controls.addEventListener('fluid-reset',()=>{renderFactor=startingQuality();rearFactor=1;lastResolutionAt=lastRaiseAt=performance.now();frameSamples=[];applyRenderResolution();});


 const backgroundFlow=createBackgroundFlow(THREE);scene.add(backgroundFlow.frontParticles);

 const backgroundTarget=new THREE.WebGLRenderTarget(1,1,{minFilter:THREE.LinearFilter,magFilter:THREE.LinearFilter,depthBuffer:false,samples:0});

 const dofTarget=new THREE.WebGLRenderTarget(1,1,{minFilter:THREE.LinearFilter,magFilter:THREE.LinearFilter,depthBuffer:false});
 const dofUniforms={uMap:{value:dofTarget.texture},uPixel:{value:new THREE.Vector2()},uView:{value:new THREE.Vector2()},uForms:{value:Array.from({length:5},()=>new THREE.Vector4())},uFocusDepth:{value:1},uStrength:{value:4},uBackgroundBlur:{value:1.8}};
 const dofScene=new THREE.Scene();
 const dofPlane=new THREE.Mesh(new THREE.PlaneGeometry(1,1),new THREE.ShaderMaterial({uniforms:dofUniforms,transparent:true,depthTest:false,depthWrite:false,vertexShader:liquidVertex,fragmentShader:
 `varying vec2 vUv;uniform sampler2D uMap;uniform vec2 uPixel,uView;uniform vec4 uForms[5];uniform float uFocusDepth,uStrength,uBackgroundBlur;
 void main(){vec2 p=(vUv-.5)*uView;float nearest=-100.;float mask=0.;
 for(int i=0;i<5;i++){vec4 f=uForms[i];float r=length(p-f.xy)/f.w;if(r<1.08){float z=f.z+f.w*.65;if(z>nearest){nearest=z;mask=1.-smoothstep(.95,1.08,r);}}}
 float far=clamp((uFocusDepth-nearest)/1.2,0.,1.);float radius=mix(uBackgroundBlur,far*uStrength,mask);vec2 d=uPixel*radius;
 vec4 c=texture2D(uMap,vUv)*.20;
 c+=(texture2D(uMap,vUv+vec2(d.x,0.))+texture2D(uMap,vUv-vec2(d.x,0.))+texture2D(uMap,vUv+vec2(0.,d.y))+texture2D(uMap,vUv-vec2(0.,d.y)))*.12;
 c+=(texture2D(uMap,vUv+d*.707)+texture2D(uMap,vUv-d*.707)+texture2D(uMap,vUv+vec2(d.x,-d.y)*.707)+texture2D(uMap,vUv+vec2(-d.x,d.y)*.707))*.08;
 if(c.a>.0001)c.rgb/=c.a;gl_FragColor=c; }`}));
 dofPlane.frustumCulled=false;dofScene.add(dofPlane);
 let dofFocus=1;const dofPoint=new THREE.Vector3();
 function updateDepthOfField(dt){dofUniforms.uView.value.set(viewW,viewH);dofUniforms.uPixel.value.set(1/dofTarget.width,1/dofTarget.height);dofPlane.scale.set(viewW,viewH,1);
 let near=-100;for(let i=0;i<5;i++){const n=i?uniforms.uNodes.value[i-1]:null;dofPoint.set(n?n.x:0,n?n.y:0,n?n.z:0).applyMatrix4(rotation).multiplyScalar(zoom).add(center);const r=(n?n.w:uniforms.uCoreRadius.value)*zoom;dofUniforms.uForms.value[i].set(dofPoint.x,dofPoint.y-.18*dofPoint.z,dofPoint.z,r);near=Math.max(near,dofPoint.z+r*.65);}
 dofFocus+=(near+(fluidSettings.dofFocus||0)-dofFocus)*(1-Math.exp(-dt*4));dofUniforms.uFocusDepth.value=dofFocus;dofUniforms.uStrength.value=fluidSettings.dofStrength??4;dofUniforms.uBackgroundBlur.value=fluidSettings.dofBackground??1.8;stage.dataset.dofFocus=dofFocus.toFixed(3);stage.dataset.dofDepths=dofUniforms.uForms.value.map(f=>f.z.toFixed(2)).join(',');
 }

 const backgroundPlane=new THREE.Mesh(new THREE.PlaneGeometry(1,1),new THREE.ShaderMaterial({uniforms:{uMap:{value:backgroundTarget.texture}},transparent:true,depthWrite:false,vertexShader:'varying vec2 vUv;void main(){vUv=uv;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.);}',fragmentShader:'varying vec2 vUv;uniform sampler2D uMap;void main(){vec4 color=texture2D(uMap,vUv);if(color.a>.0001)color.rgb/=color.a;gl_FragColor=color;}'}));
 backgroundPlane.renderOrder=-10;backgroundPlane.frustumCulled=false;scene.add(backgroundPlane);
 let backgroundAt=-Infinity,backgroundKey='';
 function createBasins(){const unique=Array.from({length:MOZER_PROFILE.volumes},(_,i)=>new RenderFluid('volume',i*.7));return Array.from({length:5},(_,i)=>unique[Math.min(i,unique.length-1)]);}
 const basins=createBasins();
 let appliedFill=fluidSettings.fillAmount??26,pendingFill=appliedFill,fillChangedAt=0;
 // Persistent air pockets are carried by actual PBF particle neighborhoods.
 const airPixels=new Float32Array(32*5*4),airSeeds=Array.from({length:5},()=>null);
 const airTexture=new THREE.DataTexture(airPixels,32,5,THREE.RGBAFormat,THREE.FloatType);
 airTexture.minFilter=airTexture.magFilter=THREE.NearestFilter;airTexture.generateMipmaps=false;
 function updateAir(basin,index){
  const p=basin.p,count=p.length/3;if(!count)return;
  if(!airSeeds[index]){
   const ranked=Array.from({length:count},(_,j)=>j).sort((a,b)=>p[a*3]**2+p[a*3+1]**2+p[a*3+2]**2-p[b*3]**2-p[b*3+1]**2-p[b*3+2]**2);
   airSeeds[index]=Array.from({length:32},(_,j)=>ranked[Math.min(count-1,j*4+3)]);
  }
  for(let j=0;j<Math.min(32,fluidSettings.airBubbleCount??16);j++){
   const seed=airSeeds[index][j]*3;let x=0,y=0,z=0,weight=0;
   // Stay embedded in the same moving fluid parcel, with the surrounding fluid smoothing its motion.
   for(let k=0;k<p.length;k+=3){const d=(p[k]-p[seed])**2+(p[k+1]-p[seed+1])**2+(p[k+2]-p[seed+2])**2;
    if(d>.065)continue;const w=Math.exp(-d/.022);x+=p[k]*w;y+=p[k+1]*w;z+=p[k+2]*w;weight+=w;
   }
   const off=(index*32+j)*4;airPixels[off]=x/weight;airPixels[off+1]=y/weight;airPixels[off+2]=z/weight;
   airPixels[off+3]=.023+.012*(.5+.5*Math.sin(j*5.3+index));
  }
  airTexture.needsUpdate=true;
 }
 const wavePixels=new Float32Array(32*32*32*5);
 const basinVersions=new Int32Array(5).fill(-1);let uploadedAirCount=-1;
 const waveTexture=new THREE.Data3DTexture(wavePixels,32,32,32*5);
 waveTexture.format=THREE.RedFormat;waveTexture.type=THREE.FloatType;
 waveTexture.unpackAlignment=1;
 waveTexture.minFilter=waveTexture.magFilter=THREE.LinearFilter;
 waveTexture.generateMipmaps=false;mozerUploadTexture(waveTexture);
 // Printed decals are sampled at the actual glass ray intersection.
 const logoCanvas=document.createElement('canvas');logoCanvas.width=512;logoCanvas.height=2560;
 const logoContext=logoCanvas.getContext('2d');
 const logoTexture=new THREE.CanvasTexture(logoCanvas);
 logoTexture.minFilter=THREE.LinearMipmapLinearFilter;logoTexture.magFilter=THREE.LinearFilter;
 const uniforms={
  ...makeSceneLightingUniforms(THREE),uHoverColor:{value:new THREE.Color()},uHoverIntensity:{value:2},uHoverWeights:{value:new Float32Array(5)},uVesselTint:{value:Array.from({length:5},()=>new THREE.Color(1,1,1))},uVesselTintStrength:{value:.45},uDropIndex:{value:-1},uOnlyDrops:{value:0},uDrawDrops:{value:0},
  uBackground:{value:backgroundTarget.texture},
  uAirCount:{value:16},uAirSize:{value:1},uAirBubbles:{value:airTexture},uLogos:{value:logoTexture},
  uView:{value:new THREE.Vector2(12,8)},
  uCenter:{value:new THREE.Vector3()},
  uRigidInv:{value:new THREE.Matrix3()},
  uCoreRadius:{value:1.1},uArmBends:{value:Array.from({length:4},()=>new THREE.Vector3())},uArmScales:{value:new Float32Array([1,1,1,1])},uNodes:{value:brands.map(()=>new THREE.Vector4())},
  uRoots:{value:Array.from({length:5},()=>new THREE.Vector4())},
  uTips:{value:Array.from({length:5},()=>new THREE.Vector4())},
  uTwigRoots:{value:Array.from({length:10},()=>new THREE.Vector4())},
  uTwigTips:{value:Array.from({length:10},()=>new THREE.Vector4())},
  uDrops:{value:Array.from({length:8},()=>new THREE.Vector4())},
  uDropShapes:{value:Array.from({length:8},()=>new THREE.Vector3(1,1,1))},
  uTime:{value:0},uFocus:{value:-1},uScale:{value:1},uWaves:{value:waveTexture},
  uAmplitude:{value:1},uGlassRoughness:{value:.035},uDensity:{value:1},uOilColor:{value:new THREE.Color()},uAbsorption:{value:new THREE.Vector3()},uGlassColor:{value:new THREE.Color()},uShadeColor:{value:new THREE.Color()},uHighlightColor:{value:new THREE.Color()},
 };
 const material=new THREE.ShaderMaterial({uniforms,vertexShader:liquidVertex,fragmentShader:liquidFragment,glslVersion:THREE.GLSL3,transparent:true,depthWrite:false});
 const vessel=new THREE.Mesh(new THREE.PlaneGeometry(1,1),material);
 vessel.frustumCulled=false;scene.add(vessel);
 // Background droplets share the oil optics, but have an independent world
 // transform and render into the rear layer before the foreground molecule.
 const emptyBackground=new THREE.DataTexture(new Uint8Array([0,0,0,0]),1,1);
 emptyBackground.needsUpdate=true;
 const dropUniforms={...uniforms,uOilColor:{value:new THREE.Color()},uAbsorption:{value:new THREE.Vector3()},uGlassColor:{value:new THREE.Color()},uOnlyDrops:{value:1},uDrawDrops:{value:1},uRigidInv:{value:new THREE.Matrix3()},uCenter:{value:new THREE.Vector3()},uScale:{value:1},uBackground:{value:emptyBackground}};
 const dropGeometry=new THREE.PlaneGeometry(1,1);
 const dropVertex=`uniform vec2 uView;varying vec2 vUv;void main(){vec4 world=modelMatrix*vec4(position,1.);vUv=world.xy/uView+.5;gl_Position=projectionMatrix*viewMatrix*world;}`;
 const backgroundDropTiles=Array.from({length:MOZER_PROFILE.drops},(_,i)=>{
  const mesh=new THREE.Mesh(dropGeometry,new THREE.ShaderMaterial({uniforms:{...dropUniforms,uDropIndex:{value:i}},vertexShader:dropVertex,fragmentShader:dropFragment,glslVersion:THREE.GLSL3,transparent:true,depthWrite:false}));
  mesh.frustumCulled=false;mesh.renderOrder=2;backgroundFlow.scene.add(mesh);return mesh;
 });
 const rotation=new THREE.Matrix4(),inverse=new THREE.Matrix4();
 const euler=new THREE.Euler();
 const temp=new THREE.Vector3();
 const center=new THREE.Vector3();
 const boundPoint=new THREE.Vector3();let moleculeScissor={x:0,y:0,w:1,h:1};
 vessel.onBeforeRender=()=>{const b=moleculeScissor;renderer.setScissor(b.x,b.y,b.w,b.h);renderer.setScissorTest(true);};
 vessel.onAfterRender=()=>renderer.setScissorTest(false);
 function updateRenderBounds(){
  let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
  function include(x,y,z,r){boundPoint.set(x,y,z).applyMatrix4(rotation).multiplyScalar(zoom).add(center);const radius=(r+.16)*zoom;const sy=boundPoint.y-.18*boundPoint.z;minX=Math.min(minX,boundPoint.x-radius);maxX=Math.max(maxX,boundPoint.x+radius);minY=Math.min(minY,sy-radius*1.017);maxY=Math.max(maxY,sy+radius*1.017);}
  include(0,0,0,uniforms.uCoreRadius.value);
  for(const name of ['uNodes','uRoots','uTips','uTwigRoots','uTwigTips'])for(const p of uniforms[name].value)include(p.x,p.y,p.z,Math.max(p.w,.12));
  const x=Math.max(0,Math.floor((minX/viewW+.5)*width)-3),y=Math.max(0,Math.floor((minY/viewH+.5)*height)-3);
  const right=Math.min(width,Math.ceil((maxX/viewW+.5)*width)+3),top=Math.min(height,Math.ceil((maxY/viewH+.5)*height)+3);
  moleculeScissor={x,y,w:Math.max(1,right-x),h:Math.max(1,top-y)};stage.dataset.shadedArea=(moleculeScissor.w*moleculeScissor.h/(width*height)).toFixed(3);
  let area=0;backgroundDropTiles.forEach((mesh,i)=>{const d=uniforms.uDrops.value[i],shape=uniforms.uDropShapes.value[i];const w=2*d.w*shape.x+.025,h=2*d.w*Math.hypot(shape.y,.18*shape.z)+.025;mesh.position.set(d.x,d.y-.18*d.z,0);mesh.scale.set(w,h,1);area+=w*h;});stage.dataset.dropShadedArea=(area/(viewW*viewH)).toFixed(4);
 }
 const gravity=new THREE.Vector3(),lastCenter=new THREE.Vector3(),lastVelocity=new THREE.Vector3();
 const bodyVelocity=new THREE.Vector3(),bodyAcceleration=new THREE.Vector3();
 let width=1,height=1,viewW=12,viewH=8;
 let focused=-1,hoverX=0,hoverY=0;const hoverWeights=new Float32Array(5);
 let pointerBlend=0,mx=0,my=0,px=0,py=0,zoom=.98,pointerInside=false,time=0,previous=performance.now();
 let averageMs=16.7,qualityAt=previous,frames=0,frameId=0,simulationTime=0,physicsAccumulator=0,sceneReady=false;
 const oilColor=new THREE.Color();
 controls.addEventListener('fluid-reset',()=>{airSeeds.fill(null);airPixels.fill(0);airTexture.needsUpdate=true;basinVersions.fill(-1);new Set(basins).forEach(b=>b.dispose());basins.splice(0,5,...createBasins());simulationTime=0;backgroundFlow.reset();});
 controls.querySelector('#fluid-stir').addEventListener('click',()=>{basins.forEach((basin,i)=>basin.impulse(Math.sin(i),.35,2.5));});
 const nodes=brands.map((brand,index)=>{
  const a=document.createElement('div');a.className='brand';
  a.setAttribute('aria-label',brand.name);a.innerHTML=brand.html;
  labels.append(a);
  const focus=()=>{focused=index;};
  const blur=()=>{if(focused===index)focused=-1;};
  a.addEventListener('pointerenter',focus);a.addEventListener('pointerleave',blur);
  a.addEventListener('focus',focus);a.addEventListener('blur',blur);
  return {...brand,a,pos:new THREE.Vector3(),base:new THREE.Vector3()};
 });
 coreLabel.querySelector('img').src='assets/mozer-v2/logos/logo-4.png';
 const logoImages=[coreLabel.querySelector('img'),...nodes.map(n=>n.a.querySelector('img'))];
 const logoWidths=[.84,.67,.69,.80,.77];
 logoImages.forEach((image,i)=>{
  const paint=()=>{
   const size=512,w=size*logoWidths[i],h=Math.min(size*.74,w*image.naturalHeight/image.naturalWidth);
   logoContext.drawImage(image,(size-w)/2,i*size+(size-h)/2,w,h);logoTexture.needsUpdate=true;
   image.style.visibility='hidden';
   if(logoImages.every(img=>img.style.visibility==='hidden'))stage.dataset.decals='surface';
  };
  if(image.complete&&image.naturalWidth)paint();else image.addEventListener('load',paint,{once:true});
 });
 const rootBases=Array.from({length:5},()=>new THREE.Vector4());
 const dropBases=Array.from({length:8},()=>new THREE.Vector4());
 const tipBases=Array.from({length:5},()=>new THREE.Vector4());
 const cursorLocal=new THREE.Vector3();
 function resolution(){
  applyRenderResolution();
 }
 function resize(){
  width=stage.clientWidth;height=stage.clientHeight;
  const mobile=MOZER_HOST.screen?MOZER_HOST.screen<640:width<600;
  const displayAspect=MOZER_HOST.displayWidth&&MOZER_HOST.displayHeight?MOZER_HOST.displayWidth/MOZER_HOST.displayHeight:width/height;
  viewH=mobile?10.2:6.8;viewW=viewH*displayAspect;
  if(mobile&&viewW<4.5){viewW=4.5;viewH=viewW/displayAspect;}
  if(!mobile&&viewW<8.4){viewW=8.4;viewH=viewW/displayAspect;}
  camera.left=-viewW/2;camera.right=viewW/2;camera.top=viewH/2;camera.bottom=-viewH/2;
  camera.updateProjectionMatrix();renderFactor=startingQuality();resolution();vessel.scale.set(viewW,viewH,1);zoom=fluidSettings.moleculeScale*mozerLayout().scale;updateMozerBackdrop();
  backgroundPlane.scale.set(viewW,viewH,1);
  // Droplet tiles are sized from their projected silhouettes each frame.
  uniforms.uView.value.set(viewW,viewH);
  const sx=1,sy=1;
  nodes.forEach((n,i)=>{n.anchor=new THREE.Vector3(n.x*sx,n.y*sy,0);n.base.copy(n.anchor);n.pos.copy(n.base);uniforms.uNodes.value[i].set(n.pos.x,n.pos.y,0,.58);});
  // Keep undeformed anchors to prevent accumulation during pointer attraction.
  // Secondary stems originate only on the core, between the four main arms.
  const branchAngles=[0,1.18,2.02,3.14,4.86];
  const branchStyles=[0,1,3,4,6];
  branchAngles.forEach((angle,i)=>{
   const dx=Math.cos(angle),dy=Math.sin(angle),reach=1.48+(branchStyles[i]%3)*.16;
   uniforms.uRoots.value[i].set(dx*.88,dy*.88,-.12,.069);
   uniforms.uTips.value[i].set(dx*reach,dy*reach,-.12,.13+(branchStyles[i]%3)*.02);
  });
  const drops=[[-3.4,.3,.15],[3.4,-.3,.18],[-1.6,2.85,.12],[.70,2.7,.18],[-.9,-2.80,.14],[1.40,-2.82,.16],[-2.9,-1.0,.08],[2.9,1.0,.10]];
  drops.forEach(([x,y,r],i)=>uniforms.uDrops.value[i].set(x*(mobile?.53:1),y*(mobile?1.4:1),-.20,r));
  dropBases.forEach((v,i)=>v.copy(uniforms.uDrops.value[i]));
  rootBases.forEach((v,i)=>v.copy(uniforms.uRoots.value[i]));
  tipBases.forEach((v,i)=>v.copy(uniforms.uTips.value[i]));
 }
 new ResizeObserver(resize).observe(stage);resize();
 function handleScenePointerMove(e){
  if(e.pointerType==='touch')return;
  pointerInside=true;
  const rect=stage.getBoundingClientRect();
  mx=(e.clientX-rect.left)/width*2-1;my=1-(e.clientY-rect.top)/height*2;
  const x=mx*viewW/2,y=my*viewH/2;
  const projected=new THREE.Vector3(),baseScreen=new THREE.Vector3();
  baseScreen.copy(center);baseScreen.y-=.18*baseScreen.z;
  let nearest=-1,best=.24;
  nodes.forEach((n,i)=>{
   projected.copy(n.pos).applyMatrix4(rotation).multiplyScalar(zoom).add(center);projected.y-=.18*projected.z;
   const dx=projected.x-baseScreen.x,dy=projected.y-baseScreen.y;
   const t=Math.max(.35,Math.min(1,((x-baseScreen.x)*dx+(y-baseScreen.y)*dy)/(dx*dx+dy*dy)));
   const tubeDistance=Math.hypot(x-baseScreen.x-t*dx,y-baseScreen.y-t*dy);
   const sphereDistance=Math.hypot(x-projected.x,y-projected.y)-uniforms.uNodes.value[i].w*zoom;
   const distance=Math.min(tubeDistance,sphereDistance);
   if(distance<best){nearest=i;best=distance;}
  });
  if(nearest<0&&Math.hypot(x-baseScreen.x,y-baseScreen.y)<uniforms.uCoreRadius.value*zoom)nearest=4;
  focused=nearest;
 }
 stage.addEventListener('pointermove',handleScenePointerMove);
 stage.addEventListener('pointerleave',()=>{mx=0;my=0;pointerInside=false;focused=-1;});
 function handleScenePointerDown(e){
  if(reducedMotion||e.target.closest('a'))return;
  const rect=stage.getBoundingClientRect();
  temp.set(((e.clientX-rect.left)/width-.5)*viewW,(.5-(e.clientY-rect.top)/height)*viewH,0);
  temp.sub(center).applyMatrix4(inverse).divideScalar(zoom);
  let index=0,radius=uniforms.uCoreRadius.value,bx=0,by=0,distance=Math.hypot(temp.x,temp.y)/radius;
  nodes.forEach((n,i)=>{const d=Math.hypot(temp.x-n.pos.x,temp.y-n.pos.y)/.58;if(d<distance){distance=d;index=i+1;radius=.58;bx=n.pos.x;by=n.pos.y;}});
  if(distance<1.)basins[index].impulse(Math.max(-.7,Math.min(.7,(temp.x-bx)/radius)),.35,.65);
 }
 stage.addEventListener('pointerdown',handleScenePointerDown);
 window.addEventListener('message',event=>{
  const m=event.data;if(event.source!==parent||m?.type!=='mozer-v2:pointer')return;
  if(!m.inside){mx=0;my=0;pointerInside=false;focused=-1;return;}
  if(!Number.isFinite(m.x)||!Number.isFinite(m.y)||m.x<0||m.x>1||m.y<0||m.y>1)return;
  const r=stage.getBoundingClientRect(),e={pointerType:'mouse',clientX:r.left+m.x*width,clientY:r.top+m.y*height,target:stage};
  handleScenePointerMove(e);if(m.kind==='down')handleScenePointerDown(e);
 });
 function place(el,point,radius){
  const d=radius*2*height/viewH*zoom;
  if(el.dataset.size!==String(d)){el.style.width=el.style.height=d+'px';el.dataset.size=String(d);}
  const x=(point.x/viewW+.5)*width-d/2,y=(.5-(point.y-.18*point.z)/viewH)*height-d/2;
  el.style.transform=`translate(${x}px,${y}px) perspective(900px) rotateX(${-euler.x}rad) rotateY(${euler.y}rad) rotateZ(${-euler.z}rad)`;
 }
 let externallyActive=true,lastDrawAt=0,wakeUntil=0,scrollTurn=0;
 function frame(now){
  frameId=0;if(document.hidden||!externallyActive)return;
  if(targetFps<60){const interval=1000/targetFps,delta=now-lastDrawAt;if(delta<interval-.5){frameId=requestAnimationFrame(frame);return;}lastDrawAt+=Math.floor((delta+.5)/interval)*interval;}else lastDrawAt=now;
  const sceneLayout=mozerLayout();
  fluidSettings.coreSize=sceneLayout.molecule.coreSize;
  for(let i=0;i<4;i++)fluidSettings['sphereSize'+i]=sceneLayout.molecule.sphereSizes[i];
  fluidSettings.waveX=sceneLayout.wave.x-50;fluidSettings.waveY=50-sceneLayout.wave.y;
  fluidSettings.waveWidth=MOZER_DEFAULTS.waveWidth*sceneLayout.wave.scale;fluidSettings.waveHeight=MOZER_DEFAULTS.waveHeight*sceneLayout.wave.scale;fluidSettings.waveDepth=MOZER_DEFAULTS.waveDepth*sceneLayout.wave.scale;
  fluidSettings.waveTiltX=sceneLayout.wave.rotationX;fluidSettings.waveTiltY=sceneLayout.wave.rotationY;fluidSettings.waveAngle=sceneLayout.wave.rotationZ;
  stage.dataset.layoutScreen=MOZER_HOST.screen||mozerScreen(innerWidth);
  if(stage._layoutState!==sceneLayout){stage._layoutState=sceneLayout;stage.dataset.layoutState=JSON.stringify(sceneLayout);}
  if(!window.mozerStartup.phases.firstFrameStart)mozerReport('firstFrameStart');
  if(document.hidden)return;
  if(resolutionPending){resolutionPending=false;applyRenderResolution();}
  const cpuStart=performance.now();let fluidCpu=0,backCpu=0;const elapsed=now-previous,dt=Math.max(0,Math.min(elapsed/1000,.05));previous=now;
  // First-order settling is monotonic: no spring, inertia, overshoot or local scale.
  const settle=1-Math.exp(-4.5*dt);
  pointerBlend+=((pointerInside?1:0)-pointerBlend)*(1-Math.exp(-2.4*dt));
  px+=((mx*pointerBlend*(systemReducedMotion?.25:1))-px)*settle;py+=((my*pointerBlend*(systemReducedMotion?.25:1))-py)*settle;
  if(!reducedMotion&&!fluidSettings.paused){time+=dt*fluidSettings.timeScale;simulationTime+=dt*fluidSettings.timeScale;}
  const lightBreath=1+(fluidSettings.cinemaBreath||0)*Math.sin(time*.55);
  uniforms.uExposure.value=fluidSettings.lightExposure*lightBreath;
  uniforms.uGlassRoughness.value=fluidSettings.roughness;
  uniforms.uCoreRadius.value=1.1*(fluidSettings.coreSize??1);
  uniforms.uAirCount.value=fluidSettings.airBubbleCount??16;uniforms.uAirSize.value=fluidSettings.airBubbleSize??1;
  uniforms.uAmplitude.value=1;uniforms.uDensity.value=fluidSettings.density;uniforms.uOilColor.value.set(fluidSettings.oil);
  dropUniforms.uOilColor.value.set(fluidSettings.dropColor||'#c7b075');
  dropUniforms.uGlassColor.value.copy(dropUniforms.uOilColor.value);
  const dc=dropUniforms.uOilColor.value,dm=Math.max(dc.r,dc.g,dc.b,.001);
  setOilAbsorption(dropUniforms.uAbsorption.value,dc);
  stage.dataset.dropColor=fluidSettings.dropColor||'#c7b075';
  oilColor.set(fluidSettings.oil);
  const maxChannel=Math.max(oilColor.r,oilColor.g,oilColor.b,.001);
  setOilAbsorption(uniforms.uAbsorption.value,oilColor);
  uniforms.uGlassColor.value.set(fluidSettings.glass);uniforms.uShadeColor.value.set(fluidSettings.shade);uniforms.uHighlightColor.value.set(fluidSettings.highlight);
  const targetBranch=focused>=0&&focused<4?nodes[focused].base:null;
  const hoverSettle=1-Math.exp(-dt/Math.max(.05,fluidSettings.hoverFade));
  hoverX+=((targetBranch?targetBranch.y*.17:0)-hoverX)*hoverSettle;
  hoverY+=((targetBranch?-targetBranch.x*.17:0)-hoverY)*hoverSettle;
  hoverWeights.forEach((v,i)=>hoverWeights[i]+=((focused===i?1:0)-v)*hoverSettle);
  const hoverAmount=Math.max(...hoverWeights);
  uniforms.uFocus.value=focused+1;
  uniforms.uHoverWeights.value[0]=hoverWeights[4];for(let i=0;i<4;i++)uniforms.uHoverWeights.value[i+1]=hoverWeights[i];
  uniforms.uHoverColor.value.set(fluidSettings.hoverColor);uniforms.uHoverIntensity.value=fluidSettings.hoverIntensity;
  uniforms.uVesselTintStrength.value=fluidSettings.moleculeTintStrength;
  ['coreTint','mozerTint','michiTint','wezerTint','unixTint'].forEach((key,i)=>uniforms.uVesselTint.value[i].set(fluidSettings[key]));
  updateSceneLighting(uniforms,fluidSettings);
  stage.dataset.focus=String(focused);stage.dataset.turn=hoverX.toFixed(3)+','+hoverY.toFixed(3);
  const targetZoom=1;
  zoom+=((targetZoom)*fluidSettings.moleculeScale*sceneLayout.scale-zoom)*(1-Math.exp(-3.6*dt));uniforms.uScale.value=zoom;
  scrollTurn+=((reducedMotion?0:MOZER_HOST.scroll)-scrollTurn)*(1-Math.exp(-4*dt));
  const scrollX=scrollTurn*sceneLayout.molecule.scrollTiltX*Math.PI/180,scrollY=scrollTurn*sceneLayout.molecule.scrollTiltY*Math.PI/180,scrollZ=scrollTurn*sceneLayout.molecule.scrollTiltZ*Math.PI/180;
  euler.set(-py*.115+hoverX+fluidSettings.moleculeTiltX*Math.PI/180+scrollX,px*.145+hoverY+fluidSettings.moleculeTiltY*Math.PI/180+scrollY,-px*.047+py*.012+fluidSettings.moleculeTiltZ*Math.PI/180+scrollZ);
  stage.dataset.scrollTurn=[scrollX,scrollY,scrollZ].map(v=>(v*180/Math.PI).toFixed(2)).join(',');rotation.makeRotationFromEuler(euler);
  inverse.copy(rotation).invert();uniforms.uRigidInv.value.setFromMatrix4(inverse);
  center.set(viewW*sceneLayout.x/100+px*.075,viewH*sceneLayout.y/100+py*.055,0);uniforms.uCenter.value.copy(center);uniforms.uTime.value=time;
  dropBases.forEach((base,i)=>{
   const drop=uniforms.uDrops.value[i],phase=time*.36+i*1.71;
   const wrap=(value,span)=>((value+span/2)%span+span)%span-span/2;
   const direction=i%2?1:-1;
   drop.copy(base);
   drop.x=wrap(base.x+direction*time*(.19+.025*(i%4))+.10*Math.sin(phase),viewW+.75);
   drop.y=wrap(base.y+time*(.095+.012*(i%3))+.12*Math.sin(phase*.8),viewH+.75);
   drop.z=-.45;
   const stretch=1.+.025*Math.sin(phase*.9);
   uniforms.uDropShapes.value[i].set(1/Math.sqrt(stretch),stretch,1/Math.sqrt(stretch));
  });
  if(dt>.0001){
   bodyVelocity.copy(center).sub(lastCenter).divideScalar(dt);
   bodyAcceleration.copy(bodyVelocity).sub(lastVelocity).divideScalar(dt).applyMatrix4(inverse);
   lastCenter.copy(center);lastVelocity.copy(bodyVelocity);
  }
  gravity.set(0,-4,0).applyMatrix4(inverse);
  const requestedFill=fluidSettings.fillAmount??26;
  if(requestedFill!==pendingFill){pendingFill=requestedFill;fillChangedAt=now;}
  if(appliedFill!==pendingFill&&now-fillChangedAt>180){
   appliedFill=pendingFill;new Set(basins).forEach(b=>b.dispose());basins.splice(0,5,...createBasins());
   airSeeds.fill(null);airPixels.fill(0);airTexture.needsUpdate=true;wavePixels.fill(0);basinVersions.fill(-1);mozerUploadTexture(waveTexture);
  }
  physicsAccumulator+=dt;
  if(physicsAccumulator>=1/physicsHz){
   const fluidStart=performance.now();const physicsDt=1/physicsHz;physicsAccumulator=Math.min(1/physicsHz,physicsAccumulator-physicsDt);
       let densityChanged=false;const airCountChanged=uploadedAirCount!==fluidSettings.airBubbleCount;
    basins.forEach((basin,i)=>{
    basin.setAmplitude(fluidSettings.amplitude);basin.force=fluidSettings.force;basin.viscosity=fluidSettings.viscosity;basin.splash=fluidSettings.splash;
    if((i===0||basin!==basins[i-1])&&!reducedMotion&&!fluidSettings.paused)basin.step(physicsDt*fluidSettings.timeScale,gravity.x-Math.max(-2,Math.min(2,bodyAcceleration.x)),gravity.z,1.75*fluidSettings.speed);
    if(basinVersions[i]!==basin.version||airCountChanged)updateAir(basin,i);
    if(basinVersions[i]!==basin.version){wavePixels.set(basin.reconstruct(),i*32*32*32);basinVersions[i]=basin.version;densityChanged=true;}
   });
   if(densityChanged){mozerUploadTexture(waveTexture);fluidUploads++;}uploadedAirCount=fluidSettings.airBubbleCount;fluidCpu=performance.now()-fluidStart;
  }
  stage.dataset.streamParticles=String(backgroundFlow.particleCount());stage.dataset.streamReady=String(backgroundFlow.ready());
  stage.dataset.particles=String(basins.reduce((count,basin)=>count+basin.count,0));
  cursorLocal.set(px*viewW*.5,py*viewH*.5,0).sub(center).applyMatrix4(inverse).divideScalar(zoom);
  const activity=(systemReducedMotion?.25:1)*Math.min(1,Math.hypot(px,py)*2.8);
  const elastic=fluidSettings.armElasticity??1,response=fluidSettings.armResponse??5,damping=fluidSettings.armDamping??.85;
  nodes.forEach((n,i)=>{
   if(!n.elasticOffset){n.elasticOffset=new THREE.Vector3();n.elasticVelocity=new THREE.Vector3();}
   let otherHover=hoverWeights[4];for(let j=0;j<4;j++)if(j!==i)otherHover=Math.max(otherHover,hoverWeights[j]);
   const anchor=n.anchor||n.base;
   const lengthSetting=fluidSettings['armLength'+i]??[.80,1.13,.98,.72][i];
   const anchorLength=anchor.length(),fixedEnds=1.1*(fluidSettings.coreSize??1)+.58*(fluidSettings['sphereSize'+i]??1);
   n.base.copy(anchor).multiplyScalar((fixedEnds+(anchorLength-1.68)*lengthSetting)/anchorLength);
   const sphereOffset=sceneLayout.molecule.sphereOffsets[i];n.base.x+=sphereOffset[0]*.06;n.base.y-=sphereOffset[1]*.06;
   if(n.base.length()<fixedEnds+.08)n.base.setLength(fixedEnds+.08);
   const radialLength=n.base.length();
   const cursorPull=activity*Math.max(-1,Math.min(1,cursorLocal.dot(n.base)/(radialLength*Math.max(cursorLocal.length(),.5))));
   const stretch=elastic*(.13*(fluidSettings.armExtension??1)*hoverWeights[i]-.10*(fluidSettings.armContraction??1)*otherHover+.045*cursorPull);
   temp.copy(n.base).multiplyScalar(stretch);temp.z=.12*elastic*hoverWeights[i];
   const steps=Math.max(1,Math.ceil(dt*120)),h=dt/steps;
   for(let k=0;k<steps;k++){
    n.elasticVelocity.x+=(response*response*(temp.x-n.elasticOffset.x)-2*response*damping*n.elasticVelocity.x)*h;
    n.elasticVelocity.y+=(response*response*(temp.y-n.elasticOffset.y)-2*response*damping*n.elasticVelocity.y)*h;
    n.elasticVelocity.z+=(response*response*(temp.z-n.elasticOffset.z)-2*response*damping*n.elasticVelocity.z)*h;
    n.elasticOffset.addScaledVector(n.elasticVelocity,h);
   }
   n.pos.copy(n.base).add(n.elasticOffset);
   const bend=uniforms.uArmBends.value[i];
   temp.copy(cursorLocal).addScaledVector(n.base,-cursorLocal.dot(n.base)/(radialLength*radialLength));
   temp.multiplyScalar(.045*activity*(fluidSettings.armBend??1));temp.clampLength(0,.20);
   bend.lerp(temp,1.-Math.exp(-response*dt));
   const actualStretch=n.pos.length()/radialLength;
   const armScale=(fluidSettings.armThickness??.65)*(1-.10*otherHover)/Math.sqrt(Math.max(.75,actualStretch));uniforms.uArmScales.value[i]=armScale;
   const depth=temp.copy(n.pos).applyMatrix4(rotation).z;
   n.radius=.58*(fluidSettings['sphereSize'+i]??1)*(1+.12*hoverWeights[i]-.10*otherHover+.045*depth);
   uniforms.uNodes.value[i].set(n.pos.x,n.pos.y,n.pos.z,n.radius);
  });
  stage.dataset.armStretch=nodes.map(n=>(n.pos.length()/n.base.length()).toFixed(3)).join(',');
  stage.dataset.armScales=Array.from(uniforms.uArmScales.value).map(v=>v.toFixed(3)).join(',');let twigIndex=0;
  const childCounts=[1,3,1,3,2];
  for(let i=0;i<5;i++){
   const root=rootBases[i],tip=tipBases[i];
   const currentRoot=uniforms.uRoots.value[i],currentTip=uniforms.uTips.value[i];
   currentRoot.copy(root);currentTip.copy(tip);
   const coreSize=fluidSettings.coreSize??1,rootLen=Math.hypot(root.x,root.y),shift=1.1*(coreSize-1);
   currentRoot.x=root.x*coreSize;currentRoot.y=root.y*coreSize;
   currentTip.x+=root.x/rootLen*shift;currentTip.y+=root.y/rootLen*shift;
   // Small molecular stems keep rigid local anchors; no cursor attraction.
   const direction=Math.atan2(currentTip.y,currentTip.x);
   for(let child=0;child<childCounts[i];child++){
    const index=twigIndex++,count=childCounts[i];
    const offset=count===1?.30:(child/(count-1)-.5)*2.05;
    const angle=direction+offset+.13*Math.sin(i*1.7+child);
    const length=.48+.055*((i+child)%3);
    const zDirection=[.62,-.60,.10][(i+child)%3];
    const planar=Math.sqrt(1.-zDirection*zDirection);
    const twig=uniforms.uTwigRoots.value[index],tip=uniforms.uTwigTips.value[index];
    twig.set(currentTip.x,currentTip.y,currentTip.z,.026);
    const depth=currentTip.z+zDirection*length;
    tip.set(currentTip.x+Math.cos(angle)*length*planar,currentTip.y+Math.sin(angle)*length*planar,depth,(.075+.012*((i+child)%3))*(1.+depth*.18));
   }
  }
  temp.set(0,.18,uniforms.uCoreRadius.value*.927).applyMatrix4(rotation).multiplyScalar(zoom).add(center);place(coreLabel,temp,uniforms.uCoreRadius.value*.955);
  nodes.forEach(n=>{temp.copy(n.pos).applyMatrix4(rotation).multiplyScalar(zoom).add(center);place(n.a,temp,n.radius);});
  updateRenderBounds();
  const currentBackgroundKey=fluidSettings.oil+fluidSettings.highlight+fluidSettings.shade+fluidSettings.glass+fluidSettings.roughness;
  if(now-backgroundAt>1000/adaptiveBackgroundHz||currentBackgroundKey!==backgroundKey){
   const backStart=performance.now();backgroundFlow.update(time,viewW,viewH,fluidSettings);
   renderer.setRenderTarget(backgroundTarget);startGpuPass("background");renderer.render(backgroundFlow.scene,camera);endGpuPass();renderer.setRenderTarget(null);
   backCpu=performance.now()-backStart;backgroundAt=now;backgroundKey=currentBackgroundKey;
  }
  stage.dataset.frontParticles=String(Math.floor(fluidSettings.particleCount/3));
  uniforms.uExposure.value=fluidSettings.lightExposure*lightBreath;stage.dataset.cinemaBreath=String(fluidSettings.cinemaBreath||0);
  startGpuPass("main");if(MOZER_PROFILE.postProcess&&((fluidSettings.dofStrength||0)>0||(fluidSettings.dofBackground||0)>0)){updateDepthOfField(dt);renderer.setRenderTarget(dofTarget);renderer.render(scene,camera);renderer.setRenderTarget(null);renderer.render(dofScene,camera);}else{renderer.setRenderTarget(null);renderer.render(scene,camera);}endGpuPass();stage.dataset.cpuMs=(performance.now()-cpuStart).toFixed(2);stage.dataset.fluidMs=fluidCpu.toFixed(2);stage.dataset.solverClock=basins[0].clock.toFixed(2);stage.dataset.simulationClock=time.toFixed(2);stage.dataset.solverPending=basins[0].pending.toFixed(2);stage.dataset.solverMs=basins.reduce((n,b)=>n+(b.solverMs||0),0).toFixed(2);stage.dataset.backgroundMs=backCpu.toFixed(2);
  if(!window.mozerStartup.phases.firstFrame){mozerReport('firstFrame');stage.dataset.firstFrameCpuMs=(performance.now()-cpuStart).toFixed(1);}
  recordFrame(elapsed,now);
  if(!reducedMotion||now<wakeUntil)frameId=requestAnimationFrame(frame);
 }
 document.addEventListener('visibilitychange',()=>{
  cancelAnimationFrame(frameId);
  if(!document.hidden&&sceneReady&&externallyActive){previous=performance.now();wakeUntil=previous+500;frameId=requestAnimationFrame(frame);}
 });
 window.addEventListener('mozer-fluid-error',()=>{fpsIndicator.textContent='ready1 · ошибка физики';stage.dataset.fluidError=fluidWorkerError;});
 mozerReport('sceneBuilt');
 async function prepareScene(){
  sceneReady=false;stage.dataset.startupState='compiling';fpsIndicator.textContent='Загрузка…';
  // Let the background and controls paint before creating GPU programs.
  await new Promise(resolve=>requestAnimationFrame(()=>setTimeout(resolve,0)));
  try{
   await Promise.all(MOZER_ASSET_PROMISES);await waitForFluid();physicsAccumulator=1/physicsHz;updateMozerBackdrop();mozerReport('compileStart');
   renderer.setRenderTarget(backgroundTarget);const rearReady=renderer.compileAsync(backgroundFlow.scene,camera);
   renderer.setRenderTarget(MOZER_PROFILE.postProcess?dofTarget:null);const mainReady=renderer.compileAsync(scene,camera);
   renderer.setRenderTarget(null);const postReady=MOZER_PROFILE.postProcess?renderer.compileAsync(dofScene,camera):Promise.resolve();
   await Promise.all([rearReady,mainReady,postReady]);
   mozerReport('compileReady');sceneReady=true;stage.dataset.startupState='ready';
   previous=performance.now();perfAt=previous;frameSamples=[];
   wakeUntil=performance.now()+500;if(!document.hidden&&externallyActive)frameId=requestAnimationFrame(frame);
  }catch(error){renderer.setRenderTarget(null);stage.dataset.startupState='error';fpsIndicator.textContent='ready1 · ошибка запуска жидкости';console.error('Scene startup',error);}
 }
 window.addEventListener('pagehide',event=>{
  if(event.persisted)return;
  sceneReady=false;cancelAnimationFrame(frameId);fluidWorkers.forEach(worker=>worker.terminate());
  renderer.dispose();renderer.forceContextLoss();
 },{once:true});
 stage.dataset.texturePrecision='float16-linear';stage.dataset.deviceProfile=MOZER_PROFILE.mobile?'mobile':(MOZER_PROFILE.lite?'desktop-lite':'desktop');stage.dataset.targetFps=MOZER_PROFILE.fps;stage.dataset.volumeSimulations=MOZER_PROFILE.volumes;
 function applyMozerMaterials(){
  const s=MOZER_RESPONSIVE.get().sceneSettings;Object.assign(fluidSettings,s);
  if(MOZER_PROFILE.mobile)Object.assign(fluidSettings,{particleCount:Math.min(MOZER_PROFILE.particles,s.particleCount),airBubbleCount:Math.min(MOZER_PROFILE.airBubbles,s.airBubbleCount),dofStrength:0,dofBackground:0,renderScale:Math.min(MOZER_PROFILE.maxDpr,s.renderScale)});
  renderer.domElement.style.filter=s.cinemaContrast===1&&s.cinemaSaturation===1&&s.cinemaWarmth===0&&s.cinemaExposure===1?'none':'contrast('+s.cinemaContrast+') saturate('+s.cinemaSaturation+') sepia('+s.cinemaWarmth+') brightness('+s.cinemaExposure+')';
  document.documentElement.style.setProperty('--mozer-vignette',s.cinemaVignette);
  stage.dataset.materialState=JSON.stringify(s);resolution();backgroundAt=-Infinity;
 }
 window.addEventListener('mozer-responsive-change',applyMozerMaterials);applyMozerMaterials();
 let lastLayoutWidth=innerWidth,lastLayoutHeight=innerHeight,lastHostScale=MOZER_HOST.scale,lastDisplayAspect=0;
 window.addEventListener('mozer-layout',()=>{
  externallyActive=MOZER_HOST.active;cancelAnimationFrame(frameId);frameId=0;
  updateMozerBackdrop();backgroundAt=-Infinity;
  const displayAspect=MOZER_HOST.displayWidth/Math.max(1,MOZER_HOST.displayHeight);
  if(innerWidth!==lastLayoutWidth||innerHeight!==lastLayoutHeight||MOZER_HOST.scale!==lastHostScale||Math.abs(displayAspect-lastDisplayAspect)>.0001){resize();lastLayoutWidth=innerWidth;lastLayoutHeight=innerHeight;lastHostScale=MOZER_HOST.scale;lastDisplayAspect=displayAspect;}
  if(sceneReady&&externallyActive&&!document.hidden){previous=performance.now();wakeUntil=previous+500;frameId=requestAnimationFrame(frame);}
 });
 setTimeout(prepareScene,0);
}
