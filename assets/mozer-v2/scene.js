
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
  if(time>lastTime){fluid.step(Math.min(.05,(time-lastTime)*settings.waveSpeed));if(uploadedFlowVersion!==fluid.version){texture.image.data=fluid.reconstruct();mozerUploadTexture(texture);updateBounds();uploadedFlowVersion=fluid.version;}}
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
const fluidWorkerURL=URL.createObjectURL(new Blob([VolumeFluid.toString().replace('Math.pow(weight/.754,4)','((weight/.754)*(weight/.754))*((weight/.754)*(weight/.754))').replaceAll('Math.hypot(p[k],p[k+1],p[k+2])','Math.sqrt(p[k]*p[k]+p[k+1]*p[k+1]+p[k+2]*p[k+2])').replaceAll('Math.hypot(sx,sy,sz)','Math.sqrt(sx*sx+sy*sy+sz*sz)')+'\n'+StreamFluid.toString()+`
const STREAM_SEED={"p":[3.019531488418579,0.4975968599319458,-0.36803194880485535,2.668201446533203,0.1645212471485138,-0.4776298701763153,3.0552029609680176,-0.15194977819919586,0.023353958502411842,3.0641512870788574,0.3603553771972656,-0.06094425544142723,2.55783748626709,-0.15620610117912292,-0.47999998927116394,3.184173345565796,-0.23976509273052216,0.3539910316467285,2.8398830890655518,0.2985948920249939,-0.21057452261447906,2.7634217739105225,0.1392367035150528,-0.22373440861701965,3.0725221633911133,-0.2391418069601059,0.37964901328086853,3.140780210494995,0.5526714324951172,-0.3170473277568817,3.1422486305236816,0.37243857979774475,-0.19769923388957977,3.185866117477417,-0.1830693632364273,0.05801573023200035,3.259556531906128,0.3508327007293701,-0.09284152835607529,3.029771327972412,0.16678476333618164,0.04261065274477005,3.2962028980255127,-0.23994526267051697,0.271473228931427,3.1659188270568848,0.2778475284576416,0.06082303822040558,3.1391522884368896,-0.006522876210510731,0.09314759075641632,3.0935652256011963,-0.23931968212127686,0.16780754923820496,3.4310851097106934,0.3858783543109894,-0.30980047583580017,3.374159336090088,0.32042399048805237,-0.1972600817680359,3.339592218399048,-0.199714794754982,0.02867688424885273,3.2733120918273926,0.34282615780830383,-0.3158796727657318,3.3208327293395996,0.26137682795524597,0.06513655185699463,3.2211809158325195,-0.23985227942466736,0.47999998927116394,3.422142505645752,0.27964842319488525,-0.048273246735334396,3.2821240425109863,0.009635290130972862,0.1576547920703888,3.2146239280700684,-0.23983941972255707,0.1750817447900772,3.6192328929901123,0.006024802569299936,-0.4196552038192749,3.4864602088928223,0.1657402217388153,-0.2643466889858246,3.418715715408325,-0.239990696310997,-0.034795891493558884,3.2908897399902344,0.5191006064414978,-0.3093031346797943,3.5448086261749268,-0.036758359521627426,0.053912486881017685,3.4135961532592773,-0.23998995125293732,0.36802491545677185,3.550607204437256,0.07691198587417603,-0.09650158882141113,3.4314303398132324,-0.021752934902906418,0.1425737738609314,3.2493412494659424,-0.23989738523960114,-0.058655958622694016,3.826174736022949,-0.023332495242357254,-0.36918938159942627,3.5869524478912354,0.027572114020586014,-0.22199463844299316,3.4493141174316406,-0.23999419808387756,-0.14634999632835388,3.578064203262329,0.3115975856781006,-0.23157882690429688,3.574631929397583,-0.13305726647377014,-0.01161466259509325,3.5331673622131348,-0.23999850451946259,0.47999998927116394,3.635301113128662,-0.06089433655142784,-0.11142686009407043,3.6008412837982178,-0.14519597589969635,0.15852688252925873,3.411717653274536,-0.23998965322971344,0.2013445645570755,3.730919122695923,-0.05244049057364464,-0.4799618721008301,3.4649806022644043,-0.18150727450847626,-0.47999998927116394,3.688573122024536,-0.2399999052286148,-0.04397156834602356,3.638714075088501,-0.03126893192529678,-0.3244016468524933,3.48181414604187,-0.239996537566185,-0.2672775983810425,3.4186930656433105,-0.239990696310997,0.47999998927116394,3.46173095703125,-0.1318390816450119,-0.19385048747062683,3.4491872787475586,-0.23999418318271637,0.10689926147460938,3.511789560317993,-0.23999786376953125,0.2965604364871979,3.9589850902557373,0.049824148416519165,-0.28976961970329285,3.4120607376098633,-0.23998971283435822,-0.4259813129901886,3.8034791946411133,-0.23999999463558197,0.028562385588884354,3.6781489849090576,-0.08880125731229782,-0.2101467251777649,3.6153149604797363,-0.23999963700771332,-0.1593601554632187,3.6183154582977295,-0.2399996519088745,0.47316041588783264,3.5684542655944824,-0.2399991750717163,-0.47999998927116394,3.6308302879333496,-0.23999972641468048,0.09233260154724121,3.7234244346618652,-0.2399999499320984,0.1868343949317932,3.6812171936035156,-0.2399998903274536,-0.2668721675872803,3.6146883964538574,-0.23999963700771332,-0.3885372579097748,3.8318893909454346,-0.23999999463558197,-0.09644106775522232,3.8717362880706787,-0.23999999463558197,0.13010917603969574,3.8343358039855957,-0.23999999463558197,-0.35634055733680725,3.6219534873962402,-0.2399996817111969,0.28560832142829895,3.8900561332702637,-0.009534304961562157,-0.47999149560928345,3.9444568157196045,-0.23999999463558197,-0.0967046245932579,3.764423370361328,-0.23999997973442078,0.33626654744148254,3.7932538986206055,-0.23999997973442078,-0.21930094063282013,3.701775550842285,-0.239999920129776,-0.47999998927116394,3.9543046951293945,-0.23999999463558197,-0.2815491855144501,3.7171220779418945,-0.2399999499320984,0.47988608479499817,-3.968857526779175,-0.23999999463558197,-0.19660109281539917,3.8201634883880615,-0.23999999463558197,0.4799674153327942,4.023456573486328,-0.23999999463558197,-0.30162566900253296,4.039516448974609,-0.23999999463558197,-0.1249334067106247,3.9807400703430176,-0.23999999463558197,0.08494046330451965,3.987934112548828,-0.23999999463558197,0.3010599911212921,3.8091163635253906,-0.23999999463558197,-0.47999998927116394,-4.014642238616943,-0.23999999463558197,-0.30983006954193115,3.915709972381592,-0.23999999463558197,0.47264716029167175,-3.8288891315460205,-0.23999999463558197,-0.46249714493751526,3.8718490600585938,-0.23999999463558197,0.282306969165802,4.026793479919434,-0.23999999463558197,0.18102267384529114,-4.120486259460449,-0.23999999463558197,-0.12290126085281372,4.099283695220947,-0.23999999463558197,0.008303177542984486,4.122860908508301,-0.23999999463558197,0.47990337014198303,-3.9310214519500732,-0.033374592661857605,-0.11940789967775345,-4.091165065765381,-0.23999999463558197,-0.07570625096559525,-3.9664273262023926,-0.23999999463558197,0.4791516661643982,-3.954324722290039,-0.23999999463558197,-0.47999998927116394,4.005679607391357,-0.23999999463558197,0.47995302081108093,-4.090801239013672,-0.23999999463558197,0.47955864667892456,-4.108911514282227,-0.23999999463558197,0.0658142939209938,-3.6850955486297607,-0.23999999463558197,0.06771291792392731,-3.878472089767456,-0.23999999463558197,0.47610950469970703,-3.698228359222412,-0.09801046550273895,-0.26173684000968933,-3.2247116565704346,-0.23999999463558197,0.13412830233573914,-3.364323139190674,-0.23999999463558197,0.4635438024997711,-3.0851387977600098,-0.23999999463558197,-0.04741594195365906,-3.6448974609375,-0.23999999463558197,-0.47999998927116394,-4.138831615447998,-0.23999999463558197,0.46019288897514343,-3.435699462890625,-0.23999999463558197,-0.40321266651153564,-3.2193081378936768,-0.23999999463558197,0.02052922360599041,-2.704564332962036,-0.23999999463558197,-0.2644306421279907,-3.7986555099487305,-0.23999999463558197,-0.32146161794662476,-3.3041083812713623,-0.23999999463558197,-0.47999998927116394,-3.2456371784210205,-0.23999999463558197,0.47999998927116394,-4.133191108703613,-0.23999999463558197,-0.47999998927116394,-3.2618231773376465,-0.23999999463558197,0.23606528341770172,-3.6794066429138184,-0.22961951792240143,-0.08548750728368759,-3.2038357257843018,-0.23999999463558197,-0.47999998927116394,4.133640289306641,-0.23999999463558197,0.30725520849227905,-2.8011269569396973,-0.23999999463558197,-0.2834284007549286,3.9290812015533447,-0.23999999463558197,-0.47999998927116394,-3.536482334136963,-0.23999999463558197,-0.3101101219654083,-3.413879871368408,-0.23999999463558197,0.470467209815979,-3.3865768909454346,0.23070067167282104,-0.07334613800048828,-3.1232993602752686,-0.23999999463558197,0.27650022506713867,3.998119592666626,-0.23999999463558197,-0.47999998927116394,-2.9529528617858887,-0.23999999463558197,0.033332280814647675,-3.0069057941436768,-0.23999999463558197,0.3290398418903351,-3.4899842739105225,0.5910924673080444,0.47999998927116394,-3.280407428741455,-0.23999999463558197,-0.04181048274040222,-3.3417727947235107,0.08541179448366165,-0.1561659723520279,-3.8411362171173096,0.10595406591892242,0.38212189078330994,-3.7573227882385254,-0.23999999463558197,-0.47999998927116394,-2.9871504306793213,-0.23999999463558197,0.20248976349830627,-3.646782875061035,-0.09232413023710251,-0.10489584505558014,-3.0699267387390137,0.029601197689771652,-0.2531375288963318,-3.554762363433838,0.2883065342903137,0.47999998927116394,-3.084393262863159,-0.23999999463558197,0.14957918226718903,-3.2067511081695557,-0.058261509984731674,-0.04397943243384361,-2.852597713470459,-0.23999999463558197,-0.010061902925372124,-2.933164119720459,-0.23999999463558197,-0.2211845964193344,-3.309440851211548,-0.13160072267055511,0.35042229294776917,-2.567209482192993,-0.23999997973442078,-0.47999998927116394,-3.0940067768096924,-0.11790946871042252,0.4040125906467438,-2.896411180496216,-0.23999999463558197,-0.10539248585700989,-2.826003074645996,-0.23999999463558197,0.26822495460510254,-3.4990549087524414,-0.0811290293931961,0.47999998927116394,-3.395548105239868,0.1209903433918953,0.47999998927116394,-2.9178261756896973,-0.23999999463558197,0.4783700108528137,-3.290801525115967,-0.05660954490303993,0.47999998927116394,-2.471632957458496,-0.23999984562397003,0.47936341166496277,-2.6536617279052734,-0.23999999463558197,-0.47999998927116394,-2.7737364768981934,0.7411705255508423,0.060065388679504395,-3.1529297828674316,-0.09491604566574097,0.47999998927116394,-2.8362698554992676,-0.23999999463558197,0.12870155274868011,-2.6469342708587646,-0.23999999463558197,0.47999998927116394,-2.369513511657715,-0.23999886214733124,0.4705353379249573,-2.01354718208313,-0.17024125158786774,0.47999998927116394,-2.7128586769104004,-0.23999999463558197,0.30888473987579346,-1.9849226474761963,-0.23947659134864807,0.038207054138183594,-3.0153377056121826,-0.23999999463558197,0.47978267073631287,-2.586294412612915,1.2508009672164917,0.47999998927116394,-2.8295648097991943,-0.23999999463558197,0.47691982984542847,-2.7260985374450684,-0.23999999463558197,0.11160093545913696,-2.260951280593872,-0.23999182879924774,0.47999998927116394,-2.1925036907196045,-0.239974245429039,-0.2351720929145813,-1.5283772945404053,-0.20682594180107117,0.3316360116004944,-2.2039990425109863,-0.14886091649532318,0.47999998927116394,-2.1878018379211426,-0.23997221887111664,0.22738973796367645,-3.5723273754119873,-0.23999999463558197,-0.47999998927116394,-2.7419259548187256,0.07542341202497482,0.47999998927116394,-2.535778045654297,-0.20840992033481598,0.4796465039253235,-2.884430170059204,0.08934036642313004,-0.3759663999080658,-1.8996065855026245,-0.23852814733982086,-0.026744984090328217,-2.139885187149048,-0.23994094133377075,0.45930027961730957,-1.749809741973877,-0.2332174926996231,-0.17389632761478424,-2.469076633453369,-0.23999984562397003,-0.32243603467941284,-2.5556793212890625,0.8987659811973572,0.47999998927116394,-3.144038438796997,-0.23999999463558197,0.47999998927116394,-2.36120867729187,1.1180518865585327,0.09411028027534485,-1.8276746273040771,-0.23679119348526,0.47999998927116394,-3.0918996334075928,-0.23999999463558197,-0.2577309012413025,-1.7988638877868652,-0.23571822047233582,0.04985011741518974,-2.3819687366485596,-0.23999911546707153,0.09054256230592728,-3.027650833129883,-0.23999999463558197,-0.47999998927116394,-2.008168935775757,-0.2396131455898285,0.4454515278339386,-2.1190145015716553,-0.23991894721984863,0.009804830886423588,-1.8864656686782837,-0.23829218745231628,0.18372900784015656,-3.2453625202178955,0.05296039953827858,-0.2714022397994995,-2.111107587814331,-0.23990876972675323,-0.09767258167266846,-2.924624443054199,-0.23999999463558197,-0.3130077123641968,-1.8538926839828491,-0.17262747883796692,-0.22380122542381287,-2.6314456462860107,-0.23999999463558197,-0.2037784308195114,-3.441025972366333,-0.23999999463558197,-0.47999998927116394,-1.624069094657898,-0.22156982123851776,0.2326143980026245,-4.085684776306152,-0.23999999463558197,-0.47999998927116394,-2.928558111190796,-0.23999999463558197,-0.47999998927116394,-1.7080225944519043,-0.23027019202709198,-0.06349116563796997,-2.469952344894409,-0.23999984562397003,0.1852082461118698,-2.3726389408111572,-0.2399989366531372,-0.25746962428092957,-2.2944114208221436,-0.23999546468257904,0.18792305886745453,-2.2805867195129395,-0.23999419808387756,-0.47999998927116394,-2.47236704826355,-0.23999986052513123,-0.47999998927116394,-1.5184674263000488,-0.20504379272460938,0.14580227434635162,-3.10225772857666,-0.23999999463558197,-0.47999998927116394,-2.732785940170288,-0.23999999463558197,0.4783787429332733,-2.0561094284057617,-0.23979830741882324,0.18262135982513428,-2.3706979751586914,-0.23999889194965363,-0.47999998927116394,-2.8314530849456787,-0.23999999463558197,-0.47999998927116394,-1.8056660890579224,-0.23599523305892944,0.3067611753940582,-2.7407007217407227,-0.23999999463558197,-0.47999998927116394,-1.621747374534607,-0.22127200663089752,-0.17080669105052948,-1.8714079856872559,-0.17028751969337463,0.4798974394798279,-2.2999861240386963,-0.23999589681625366,-0.1771858036518097,-1.9247483015060425,-0.23890122771263123,-0.14641639590263367,-2.039469003677368,1.671656608581543,0.47999998927116394,-2.4231812953948975,-0.23999960720539093,-0.0495431087911129,-2.0471651554107666,-0.23977160453796387,-0.20895151793956757,-2.077669858932495,-0.23985135555267334,-0.47999998927116394,-2.1793713569641113,-0.2399681955575943,-0.47999998927116394,-1.5209439992904663,-0.20549276471138,0.028806589543819427,-1.8563870191574097,-0.2376251518726349,-0.47999998927116394,-2.0307345390319824,-0.23971395194530487,-0.36230170726776123,-2.3207502365112305,-0.23999717831611633,0.2946283221244812,-0.8284440040588379,1.1996835470199585,0.47999998927116394,-2.2277679443359375,-0.2399856001138687,-0.3373982310295105,-2.4707188606262207,-0.23999984562397003,0.292775958776474,-1.6362606287002563,-0.22308219969272614,-0.47999998927116394,-0.9369891285896301,-0.1609819531440735,-0.311644047498703,-1.3111441135406494,-0.1676701456308365,0.33903074264526367,-1.3838732242584229,-0.17928440868854523,-0.3292456269264221,-1.9072564840316772,-0.23865194618701935,-0.32013481855392456,-1.6515055894851685,-0.22485174238681793,0.06494317203760147,-1.4257932901382446,-0.18723531067371368,-0.20137307047843933,-1.7534071207046509,-0.23343370854854584,-0.34919148683547974,-1.5288503170013428,-0.20691002905368805,-0.11524176597595215,-1.9681997299194336,-0.2393529713153839,-0.47999998927116394,-0.7713474035263062,-0.14655710756778717,-0.23257239162921906,-1.1283620595932007,-0.14747630059719086,-0.2681419253349304,-0.7678042650222778,-0.16041739284992218,-0.026955632492899895,-1.5311886072158813,-0.2073242962360382,-0.47999998927116394,-1.7253620624542236,-0.23159420490264893,0.4745630621910095,-1.4631996154785156,-0.1945532262325287,0.46276187896728516,-1.209578514099121,-0.15769676864147186,-0.47851675748825073,-1.5120528936386108,-0.20387040078639984,-0.32166966795921326,-0.5152506828308105,-0.23825906217098236,0.3469218313694,-0.5972299575805664,-0.05651230737566948,-0.3715238571166992,-1.1324204206466675,-0.15815958380699158,-0.12692822515964508,-0.8661893606185913,-0.20778611302375793,0.08555566519498825,-0.5591045618057251,-0.057165760546922684,-0.08420996367931366,-1.7477272748947144,-0.23308975994586945,-0.47999998927116394,-1.1121909618377686,-0.15172253549098969,-0.4139039218425751,-1.32140052318573,-0.16905374825000763,-0.47990092635154724,-1.4273619651794434,-0.1875370740890503,-0.47999998927116394,-1.0237947702407837,-0.17791084945201874,-0.02499643713235855,-0.4010816514492035,0.012423614040017128,-0.08315445482730865,-1.3572077751159668,-0.17460475862026215,0.07211009413003922,-1.4062998294830322,-0.18347929418087006,0.21700197458267212,-1.012325644493103,-0.14641356468200684,-0.203106090426445,-1.7467190027236938,-0.23302721977233887,0.1791141778230667,-0.6486676335334778,-0.23330968618392944,0.41595885157585144,-1.2560577392578125,-0.16084931790828705,-0.1836194396018982,-1.6275604963302612,-0.22201178967952728,-0.33376193046569824,-0.644883394241333,-0.1221710741519928,-0.2432374656200409,-0.6685430407524109,-0.12023774534463882,-0.11148552596569061,-0.4895361363887787,-0.025160567834973335,-0.19781319797039032,-0.805019736289978,0.030378997325897217,-0.32793161273002625,-0.6315780878067017,-0.1304151713848114,0.03511631861329079,-1.6137347221374512,-0.22022005915641785,0.47458019852638245,-0.3364354372024536,0.08992055058479309,-0.1807779222726822,-0.3666112422943115,0.03304777294397354,0.11644188314676285,-1.2519454956054688,-0.15967893600463867,-0.32963743805885315,-1.0931097269058228,-0.16711783409118652,0.16387337446212769,-1.3469949960708618,-0.17293375730514526,0.47999998927116394,-0.40917953848838806,0.16033658385276794,-0.3096782863140106,-1.2083920240402222,-0.16004155576229095,0.2669000029563904,-0.9052413105964661,-0.17651304602622986,-0.044065095484256744,-1.400246262550354,-0.18233095109462738,-0.053453899919986725,-0.4846963882446289,-0.03008759394288063,0.025581935420632362,-1.0362553596496582,-0.1757262647151947,0.38614213466644287,-1.2551753520965576,-0.1619480401277542,0.12439621984958649,-0.7365306615829468,-0.21825101971626282,0.11399827152490616,-0.29492056369781494,0.1755577027797699,0.09810522198677063,-0.1297152042388916,0.1860606074333191,0.20698130130767822,-0.029039347544312477,-0.23999889194965363,0.3342595100402832,-0.4678497016429901,-0.2389938235282898,0.24668274819850922,-1.2623724937438965,-0.16249437630176544,-0.05472308769822121,-0.2930181920528412,-0.0703878179192543,0.3259652853012085,-0.9854651093482971,-0.1850552260875702,0.11387438327074051,-1.151033639907837,-0.16153208911418915,0.05506407842040062,-0.25238746404647827,-0.23994757235050201,0.2713114321231842,-0.38165369629859924,-0.2396620362997055,0.4716740846633911,-1.0203747749328613,-0.15863482654094696,-0.4724903702735901,0.31246647238731384,-0.23999999463558197,0.3035811483860016,-0.8757036328315735,-0.20609673857688904,0.299794465303421,-0.45170316100120544,-0.09600073844194412,0.16252589225769043,-0.5983042120933533,-0.20500624179840088,0.16733931005001068,-0.21163517236709595,0.08236464112997055,0.2930701971054077,0.057976141571998596,-0.23999980092048645,0.2978357970714569,0.2508622109889984,-0.23999999463558197,0.4369361400604248,0.29658666253089905,-0.16933676600456238,-0.17098918557167053,-0.880365788936615,-0.15039528906345367,-0.18347737193107605,0.5496566891670227,-0.23999999463558197,-0.008825813420116901,0.176217719912529,-0.019349541515111923,0.12660372257232666,0.0509047657251358,-0.04689542204141617,0.4393936097621918,-0.16253647208213806,0.2508201003074646,-0.09525890648365021,0.03641702979803085,0.15682001411914825,0.2191181778907776,0.4042481780052185,-0.23999999463558197,0.15725825726985931,0.1350436508655548,-0.021977970376610756,0.2949201762676239,0.27390462160110474,-0.23999999463558197,0.03064946085214615,-0.37220919132232666,-0.23970232903957367,0.3208700120449066,0.5482141375541687,-0.23999999463558197,0.24777421355247498,-0.2930445075035095,0.24734944105148315,-0.24778886139392853,0.1653100997209549,-0.23999997973442078,0.31219950318336487,-0.055350206792354584,0.10025768727064133,0.34491050243377686,-0.5075626373291016,0.04270467534661293,-0.33410438895225525,0.6863629817962646,-0.2399999350309372,0.12878040969371796,-0.04530128836631775,0.23238396644592285,0.08662670105695724,-0.1335299164056778,0.38191425800323486,-0.13617564737796783,-0.2811494767665863,0.13661299645900726,-0.057719871401786804,0.6899986863136292,-0.2399999350309372,0.2736843228340149,-0.9139281511306763,-0.17218047380447388,-0.45200005173683167,0.2584803104400635,-0.23999999463558197,0.15470322966575623,0.8141803741455078,-0.23999938368797302,0.19734594225883484,0.542988121509552,-0.23999999463558197,0.11067810654640198,0.6756806373596191,-0.2399999499320984,-0.005062837153673172,-0.6970656514167786,-0.14499324560165405,-0.3777134418487549,-0.2209276705980301,0.3645583689212799,-0.24644774198532104,0.1529867798089981,0.2916509807109833,-0.03138361871242523,-0.142311692237854,-0.23999135196208954,0.3562580943107605,-0.8073462247848511,-0.1771458089351654,-0.4120827615261078,-0.1702367663383484,0.1944539099931717,0.0368812121450901,0.08674801141023636,0.17316500842571259,0.06284169107675552,0.07078219205141068,0.520774245262146,-0.04463879019021988,0.3925202786922455,-0.23999999463558197,-0.0025048269890248775,0.5470026135444641,0.04028759151697159,-0.120987169444561,0.47394847869873047,-0.1779368668794632,-0.11896944046020508,0.6024664044380188,-0.23999999463558197,-0.47999998927116394,-0.048681776970624924,0.4852389395236969,-0.10850023478269577,0.3393113315105438,-0.23999999463558197,-0.33355557918548584,0.9333127737045288,-0.2399955838918686,0.22562085092067719,0.2163514792919159,-0.04044690355658531,-0.02887221798300743,0.4671744406223297,-0.23999999463558197,-0.3915301263332367,0.3979438841342926,-0.23999999463558197,-0.47999998927116394,0.21110887825489044,0.4700082540512085,-0.0464889258146286,0.8261713981628418,-0.12275312840938568,-0.2962309420108795,0.7870416045188904,-0.23999962210655212,-0.08188454806804657,0.5984153747558594,-0.1375938057899475,-0.21955835819244385,0.8832944631576538,-0.23999802768230438,-0.15268737077713013,0.5462221503257751,-0.23999999463558197,-0.17969805002212524,0.7103817462921143,-0.14764143526554108,-0.47999998927116394,0.32420048117637634,-0.23999999463558197,-0.13254332542419434,1.0637444257736206,-0.2399687021970749,0.2571025788784027,0.42875224351882935,0.1842094212770462,-0.02415984310209751,0.567909836769104,-0.23999999463558197,-0.3283097743988037,0.3580458164215088,0.02568989247083664,-0.06202498823404312,0.6820592284202576,-0.1304798126220703,-0.3529825806617737,0.9694944620132446,-0.23999223113059998,-0.24210712313652039,1.2374283075332642,-0.2396944761276245,0.10302571207284927,0.7395665645599365,-0.23999983072280884,-0.30383968353271484,0.7061377763748169,-0.2399999052286148,-0.17916907370090485,0.8367614150047302,-0.23999910056591034,0.06722554564476013,-0.02252054773271084,0.31226515769958496,-0.04776696488261223,1.0768678188323975,-0.23996232450008392,-0.3649122416973114,1.0266746282577515,-0.23998166620731354,-0.10660453140735626,0.9084569215774536,-0.2399970293045044,-0.47999998927116394,0.2802867293357849,-0.23999999463558197,-0.45403558015823364,1.184657096862793,-0.23984088003635406,-0.05933852866292,1.2959986925125122,-0.12089703232049942,-0.47999998927116394,1.1091121435165405,-0.23994113504886627,-0.21214832365512848,0.9387317895889282,-0.23999519646167755,-0.0014946204610168934,0.9007104635238647,-0.23999738693237305,-0.3627125024795532,1.4711034297943115,-0.2363867610692978,-0.3596058189868927,1.324614405632019,-0.23916824162006378,-0.06596001982688904,0.3398221731185913,0.37577658891677856,-0.005256619770079851,1.0933443307876587,-0.1046941727399826,-0.47999998927116394,1.1795508861541748,-0.23985089361667633,0.2073022872209549,1.3397198915481567,-0.17066673934459686,-0.3381340205669403,1.2250844240188599,-0.2397368997335434,-0.3516072630882263,1.7813743352890015,-0.20655596256256104,-0.47999998927116394,0.77942955493927,-0.2399996668100357,-0.47999998927116394,1.510058879852295,-0.23489585518836975,-0.21822156012058258,1.2286714315414429,-0.23972515761852264,-0.18831869959831238,1.1956489086151123,-0.23981720209121704,-0.47999998927116394,1.6762511730194092,-0.1587107926607132,-0.47999998927116394,1.3516970872879028,-0.23888644576072693,0.06196237727999687,1.6298996210098267,-0.22688403725624084,-0.23191110789775848,1.3636560440063477,-0.23873701691627502,-0.22354139387607574,1.4366331100463867,-0.23738016188144684,0.15345066785812378,1.0415817499160767,-0.23997722566127777,-0.47999998927116394,1.4664661884307861,-0.23653669655323029,-0.47999998927116394,1.6115022897720337,-0.22851994633674622,-0.08632364869117737,1.7792670726776123,-0.0800323337316513,-0.401914119720459,1.6513872146606445,-0.22475776076316833,-0.3672548830509186,1.1091111898422241,-0.23994113504886627,0.04172852262854576,1.506378173828125,-0.23505574464797974,0.03122790902853012,1.6324949264526367,-0.2266397625207901,0.0642184466123581,1.4547007083892822,-0.23689356446266174,-0.09665527194738388,2.176907777786255,-0.13384795188903809,-0.47999998927116394,1.9078320264816284,-0.18134593963623047,-0.47999998927116394,1.7693029642105103,-0.20863176882266998,0.08397585153579712,1.8919258117675781,-0.04340418055653572,-0.47999998927116394,1.8831957578659058,-0.18659579753875732,-0.17941369116306305,1.014216423034668,-0.23998475074768066,0.11745622754096985,2.0508155822753906,-0.152601420879364,-0.47999998927116394,1.8368390798568726,-0.1961452215909958,-0.3268895149230957,1.5834872722625732,-0.230703666806221,-0.47999998927116394,2.3580496311187744,0.04064975678920746,-0.28636300563812256,2.086437463760376,-0.007434464991092682,-0.47999998927116394,1.6980382204055786,-0.21929480135440826,0.19171202182769775,1.3437436819076538,-0.23897691071033478,-0.47999998927116394,2.2999680042266846,-0.14600306749343872,-0.13631121814250946,2.156249761581421,-0.14116664230823517,-0.23462894558906555,2.529343366622925,-0.0637761801481247,-0.2675970494747162,2.0079517364501953,-0.16056758165359497,-0.23081940412521362,2.140528678894043,-0.1413889080286026,-0.352499395608902,2.4688310623168945,-0.1763446182012558,-0.06589809060096741,1.9818989038467407,-0.16571593284606934,0.23459617793560028,2.447411060333252,-0.17178982496261597,-0.21424052119255066,2.2262635231018066,-0.14043012261390686,0.35274091362953186,1.8481636047363281,-0.19386878609657288,-0.0046327621676027775,1.7384425401687622,-0.21359114348888397,-0.08086670935153961,2.3049216270446777,-0.14664898812770844,-0.2475326508283615,1.5612231492996216,-0.23219358921051025,0.2075924426317215,1.5222722291946411,-0.23433418571949005,0.47830215096473694,1.9523757696151733,-0.17182260751724243,-0.05832503363490105,2.717738628387451,-0.00478725228458643,-0.4774121046066284,2.061281442642212,-0.15125907957553864,-0.10684514045715332,1.9549808502197266,-0.17128583788871765,0.3998754322528839,2.473066568374634,-0.17725124955177307,0.06327103078365326,1.8704593181610107,-0.18927399814128876,0.19810880720615387,2.3419485092163086,-0.15182501077651978,-0.0021085659973323345,1.6633187532424927,-0.22347287833690643,0.4500787854194641,1.5126769542694092,-0.23477952182292938,0.34462395310401917,2.45214581489563,-0.17279072105884552,0.2939628064632416,2.5549960136413574,0.01813092641532421,-0.387986958026886,2.704699993133545,-0.15809029340744019,-0.21229299902915955,2.685765266418457,-0.2171175330877304,0.1129927933216095,2.8243894577026367,0.16785623133182526,-0.3649151921272278,2.1203365325927734,-0.14388726651668549,0.22210657596588135,1.9831938743591309,-0.16545464098453522,0.09155934303998947,1.3136852979660034,-0.23926253616809845,0.4420308470726013,2.1965603828430176,-0.139998659491539,0.042918793857097626,2.767211437225342,-0.2266119122505188,0.2776434123516083,2.9067814350128174,-0.20176485180854797,-0.0245077982544899,1.9778058528900146,-0.1665499061346054,-0.3597686290740967,1.4040062427520752,-0.238093763589859,0.27922093868255615,2.814955472946167,-0.14126521348953247,-0.13374565541744232,2.4896011352539062,-0.14342473447322845,-0.3497233986854553,1.758074402809143,-0.21049509942531586,-0.21644139289855957,2.7396585941314697,-0.22380059957504272,-0.11808181554079056,2.5814247131347656,-0.16425390541553497,-0.16051003336906433,2.960479497909546,0.23259080946445465,-0.1127815693616867,2.3171255588531494,-0.14821664988994598,0.16389310359954834,2.8319201469421387,0.3215814530849457,-0.44857361912727356,1.815062403678894,-0.20039068162441254,0.38029080629348755,2.861740827560425,0.08386100083589554,-0.11609314382076263,2.543133020401001,-0.19209147989749908,0.16991132497787476,2.1755757331848145,-0.1403065323829651,-0.08095107972621918,2.606229782104492,-0.20434895157814026,-0.020593758672475815,1.7033708095550537,-0.21859410405158997,0.3147769868373871,2.912782669067383,0.43135327100753784,-0.41937655210494995,2.9949541091918945,0.3590617775917053,-0.2257983237504959,2.8922624588012695,-0.2349971979856491,0.3362805247306824,2.930748701095581,0.27061188220977783,-0.3182753026485443,1.2781155109405518,-0.23950664699077606,0.2819707691669464,2.823032855987549,-0.23116157948970795,0.07439952343702316,2.4369075298309326,-0.08577613532543182,-0.47999998927116394,2.321444272994995,-0.1310516595840454,-0.37567391991615295,2.9547863006591797,-0.03439711779356003,-0.027842411771416664,3.1327552795410156,0.3278495967388153,-0.34050747752189636,3.034637451171875,0.2663082182407379,-0.4094517230987549,2.973893880844116,-0.2376321703195572,0.08406121283769608,2.3034961223602295,-0.12237206101417542,-0.47999998927116394,2.848198175430298,0.129533588886261,-0.4798593521118164,2.9798495769500732,-0.23776523768901825,0.22678424417972565,2.568272590637207,0.022009825333952904,-0.47999998927116394,2.6511924266815186,0.045036397874355316,-0.24971646070480347,2.687314987182617,0.14501668512821198,-0.35608187317848206],"old":[2.9993765354156494,0.4940187633037567,-0.3760961592197418,2.6481447219848633,0.16073490679264069,-0.4779660403728485,3.037306070327759,-0.1409376859664917,0.02859998494386673,3.043818950653076,0.36382633447647095,-0.06754633039236069,2.5343315601348877,-0.1538967788219452,-0.47999998927116394,3.166121244430542,-0.23970727622509003,0.3558383584022522,2.819730520248413,0.29825979471206665,-0.21301448345184326,2.7430384159088135,0.14189578592777252,-0.2229221612215042,3.055027723312378,-0.23896338045597076,0.38065317273139954,3.1199889183044434,0.5498300194740295,-0.3272135257720947,3.1210811138153076,0.3735811412334442,-0.2041337937116623,3.166660785675049,-0.17441554367542267,0.06431015580892563,3.2383971214294434,0.3554556965827942,-0.10108152031898499,3.0094525814056396,0.17554455995559692,0.03858567401766777,3.279853582382202,-0.2399316281080246,0.2742103636264801,3.145691156387329,0.28577902913093567,0.05315915495157242,3.1193442344665527,0.006870696321129799,0.09061390906572342,3.077577829360962,-0.23918797075748444,0.16887067258358002,3.409947633743286,0.38874855637550354,-0.3188543915748596,3.3529417514801025,0.3250252306461334,-0.20521284639835358,3.3201067447662354,-0.19322721660137177,0.03423265367746353,3.251964807510376,0.34436532855033875,-0.32275640964508057,3.2997729778289795,0.27027663588523865,0.0563477985560894,3.205711841583252,-0.23982028663158417,0.47999998927116394,3.400676727294922,0.28719809651374817,-0.05644110590219498,3.261455535888672,0.02622084692120552,0.1524183750152588,3.195495843887329,-0.23979580402374268,0.175892174243927,3.6012556552886963,0.010095043107867241,-0.41924718022346497,3.465517044067383,0.17259910702705383,-0.2713426947593689,3.4028334617614746,-0.23998817801475525,-0.02648857794702053,3.2698142528533936,0.5184485912322998,-0.31943801045417786,3.5256805419921875,-0.02621699497103691,0.04709846153855324,3.3984479904174805,-0.23998737335205078,0.3688185214996338,3.5298266410827637,0.08854074031114578,-0.10230313986539841,3.410275936126709,-0.003789931768551469,0.13673175871372223,3.234952926635742,-0.23987624049186707,-0.04409792646765709,3.804495334625244,-0.02463143691420555,-0.3711978793144226,3.5691468715667725,0.03285323455929756,-0.2272379994392395,3.4396934509277344,-0.23999325931072235,-0.13684682548046112,3.5569729804992676,0.31750598549842834,-0.24123549461364746,3.557424306869507,-0.13087350130081177,-0.010652158409357071,3.519294500350952,-0.23999811708927155,0.47999998927116394,3.6154677867889404,-0.05635092407464981,-0.11282511055469513,3.583158493041992,-0.14197410643100739,0.15167340636253357,3.3952929973602295,-0.23998676240444183,0.20560544729232788,3.714409351348877,-0.05418813228607178,-0.47998949885368347,3.455998182296753,-0.1837095022201538,-0.47999998927116394,3.6727986335754395,-0.23999987542629242,-0.04144647717475891,3.623544692993164,-0.03292008489370346,-0.32302287220954895,3.472137451171875,-0.23999595642089844,-0.2577303647994995,3.405323028564453,-0.23998861014842987,0.47999998927116394,3.4511630535125732,-0.13311108946800232,-0.18663521111011505,3.433034658432007,-0.23999252915382385,0.11028352379798889,3.498394250869751,-0.23999734222888947,0.29645970463752747,3.9390718936920166,0.04699437692761421,-0.288867324590683,3.4042110443115234,-0.23998841643333435,-0.4258161187171936,3.788897752761841,-0.23999997973442078,0.030685942620038986,3.6597766876220703,-0.09220928698778152,-0.21436873078346252,3.6059200763702393,-0.23999956250190735,-0.1547267585992813,3.60422682762146,-0.23999954760074615,0.4740449786186218,3.5602216720581055,-0.23999905586242676,-0.47999998927116394,3.6135094165802,-0.23999962210655212,0.09523127228021622,3.707435369491577,-0.2399999350309372,0.1878620833158493,3.671178102493286,-0.23999986052513123,-0.2681350111961365,3.608825445175171,-0.23999959230422974,-0.38987457752227783,3.817826509475708,-0.23999999463558197,-0.09460672736167908,3.8542044162750244,-0.23999999463558197,0.13034336268901825,3.8264381885528564,-0.23999999463558197,-0.35740673542022705,3.605750799179077,-0.23999956250190735,0.28412896394729614,3.871110200881958,-0.0148462587967515,-0.47999972105026245,3.9283130168914795,-0.23999999463558197,-0.09904170036315918,3.752243995666504,-0.23999996483325958,0.33531633019447327,3.781687021255493,-0.23999997973442078,-0.2192528247833252,3.692671298980713,-0.2399999052286148,-0.47999998927116394,3.9450783729553223,-0.23999999463558197,-0.28428441286087036,3.7025306224823,-0.239999920129776,0.4799592196941376,-4.006574630737305,-0.23999999463558197,-0.22014360129833221,3.8045263290405273,-0.23999999463558197,0.47999998927116394,4.01776647567749,-0.23999999463558197,-0.3063870668411255,4.0214457511901855,-0.23999999463558197,-0.1324131339788437,3.965876340866089,-0.23999999463558197,0.08992039412260056,3.9745290279388428,-0.23999999463558197,0.30427560210227966,3.8007545471191406,-0.23999999463558197,-0.47999998927116394,-4.042867660522461,-0.23999999463558197,-0.324313759803772,3.9017248153686523,-0.23999999463558197,0.4739776849746704,-3.8509151935577393,-0.23999999463558197,-0.44865065813064575,3.8568201065063477,-0.23999999463558197,0.28222963213920593,4.012408256530762,-0.23999999463558197,0.18335461616516113,-4.141611576080322,-0.23999999463558197,-0.12448211014270782,4.081530570983887,-0.23999999463558197,0.011746302247047424,4.102032661437988,-0.23999999463558197,0.47999998927116394,-3.9503333568573,-0.028325188905000687,-0.12077254801988602,-4.10882568359375,-0.23999999463558197,-0.07195387035608292,-3.9851925373077393,-0.23999999463558197,0.47944068908691406,-3.978074789047241,-0.23999999463558197,-0.47999998927116394,3.9901275634765625,-0.23999999463558197,0.47999998927116394,-4.109577178955078,-0.23999999463558197,0.47899171710014343,-4.11539888381958,-0.23999999463558197,0.047007329761981964,-3.7213664054870605,-0.23999999463558197,0.058362431824207306,-3.9001379013061523,-0.23999999463558197,0.47679802775382996,-3.724668025970459,-0.09473281353712082,-0.2591972351074219,-3.2534825801849365,-0.23999999463558197,0.13818711042404175,-3.3842155933380127,-0.23999999463558197,0.4655914902687073,-3.1141514778137207,-0.23999999463558197,-0.04271026700735092,-3.6742210388183594,-0.23999999463558197,-0.47999998927116394,-4.142479419708252,-0.23999999463558197,0.4634738862514496,-3.469780206680298,-0.23999999463558197,-0.40150749683380127,-3.251279592514038,-0.23999999463558197,0.02134374901652336,-2.7320573329925537,-0.23999999463558197,-0.2578141391277313,-3.8289382457733154,-0.23999999463558197,-0.3113408088684082,-3.3259425163269043,-0.23999999463558197,-0.47999998927116394,-3.2726316452026367,-0.23999999463558197,0.47999998927116394,-4.134113311767578,-0.23999999463558197,-0.47999998927116394,-3.2866852283477783,-0.23999999463558197,0.24013637006282806,-3.701014757156372,-0.21515050530433655,-0.08525457978248596,-3.227069139480591,-0.23999999463558197,-0.47999998927116394,4.12525749206543,-0.23999999463558197,0.31112006306648254,-2.83241868019104,-0.23999999463558197,-0.281440794467926,3.922584056854248,-0.23999999463558197,-0.47999998927116394,-3.563401699066162,-0.23999999463558197,-0.3047918677330017,-3.4380834102630615,-0.23999999463558197,0.4710584282875061,-3.406414747238159,0.23321790993213654,-0.07136919349431992,-3.150655746459961,-0.23999999463558197,0.27882474660873413,3.9902498722076416,-0.23999999463558197,-0.47999998927116394,-2.9831740856170654,-0.23999999463558197,0.03589135408401489,-3.036428213119507,-0.23999999463558197,0.33186203241348267,-3.516069173812866,0.565254271030426,0.47999998927116394,-3.302685499191284,-0.23999999463558197,-0.03687356412410736,-3.361520767211914,0.09740690886974335,-0.16341261565685272,-3.855391025543213,0.10885767638683319,0.3749595582485199,-3.786193370819092,-0.23999999463558197,-0.47999998927116394,-3.0140979290008545,-0.23999999463558197,0.2052709460258484,-3.6670870780944824,-0.07699444890022278,-0.10977402329444885,-3.089597463607788,0.04233980551362038,-0.2594549357891083,-3.5863616466522217,0.2667470872402191,0.47999998927116394,-3.1128032207489014,-0.23999999463558197,0.1492583453655243,-3.2275965213775635,-0.0488414540886879,-0.04139084368944168,-2.8822240829467773,-0.23999999463558197,-0.006902489345520735,-2.9548778533935547,-0.23664474487304688,-0.21334517002105713,-3.3298826217651367,-0.12107627093791962,0.3511384129524231,-2.5889666080474854,-0.23999999463558197,-0.47999998927116394,-3.1198909282684326,-0.11021807044744492,0.4076555371284485,-2.9214606285095215,-0.23999999463558197,-0.10038492828607559,-2.854640007019043,-0.23999999463558197,0.27194246649742126,-3.5195701122283936,-0.07559629529714584,0.47999998927116394,-3.4133388996124268,0.1418086737394333,0.47999998927116394,-2.9420063495635986,-0.23999999463558197,0.47882983088493347,-3.3123936653137207,-0.05719493702054024,0.47999998927116394,-2.4945101737976074,-0.2399999052286148,0.4795248508453369,-2.6763813495635986,-0.23999999463558197,-0.47999998927116394,-2.7936675548553467,0.7449048757553101,0.04475461319088936,-3.173779249191284,-0.09684914350509644,0.47999998927116394,-2.8659603595733643,-0.23999999463558197,0.1335485279560089,-2.665180206298828,-0.23999999463558197,0.4795616567134857,-2.3892085552215576,-0.2399992197751999,0.47181373834609985,-2.0306479930877686,-0.17503990232944489,0.47999998927116394,-2.7318997383117676,-0.23999999463558197,0.32206863164901733,-2.0056567192077637,-0.23960012197494507,0.04144909605383873,-3.040127992630005,-0.23999999463558197,0.4799268841743469,-2.6059515476226807,1.263468861579895,0.47999998927116394,-2.85534405708313,-0.23999999463558197,0.4776684045791626,-2.757089853286743,-0.23999999463558197,0.11584270000457764,-2.282663583755493,-0.23999440670013428,0.47999998927116394,-2.2108917236328125,-0.23998093605041504,-0.2361513376235962,-1.5533188581466675,-0.21112512052059174,0.3361927270889282,-2.221837282180786,-0.15013721585273743,0.47999268770217896,-2.2080237865448,-0.23998001217842102,0.22795137763023376,-3.6086058616638184,-0.23999999463558197,-0.47999998927116394,-2.7608189582824707,0.10527543723583221,0.47999998927116394,-2.5521137714385986,-0.20025578141212463,0.4797835648059845,-2.9040720462799072,0.10050845146179199,-0.3803368806838989,-1.921393632888794,-0.23885685205459595,-0.02415725775063038,-2.1594560146331787,-0.23995639383792877,0.46232864260673523,-1.7711759805679321,-0.2344220131635666,-0.17206738889217377,-2.487468957901001,-0.2399998903274536,-0.3208433985710144,-2.574922561645508,0.9091255068778992,0.47999998927116394,-3.169125556945801,-0.23999999463558197,0.47999998927116394,-2.381063222885132,1.1195451021194458,0.06746919453144073,-1.8614355325698853,-0.23775069415569305,0.47999998927116394,-3.1128201484680176,-0.23999999463558197,-0.2576409578323364,-1.8206591606140137,-0.2365533858537674,0.052562545984983444,-2.399502754211426,-0.23999936878681183,0.09033587574958801,-3.0490658283233643,-0.23999999463558197,-0.47999998927116394,-2.025604248046875,-0.23969340324401855,0.4526134133338928,-2.138944625854492,-0.2399400919675827,0.012706196866929531,-1.9085361957550049,-0.23867173492908478,0.18777447938919067,-3.264596700668335,0.06536209583282471,-0.27881187200546265,-2.1293087005615234,-0.23993059992790222,-0.09732484817504883,-2.9480061531066895,-0.23999999463558197,-0.31302350759506226,-1.8761520385742188,-0.17166703939437866,-0.22343656420707703,-2.6548802852630615,-0.23999999463558197,-0.20830902457237244,-3.474766731262207,-0.23999999463558197,-0.47999998927116394,-1.6469781398773193,-0.2243402749300003,0.23428703844547272,-4.091221809387207,-0.23999999463558197,-0.47999998927116394,-2.951366424560547,-0.23999999463558197,-0.47999998927116394,-1.7302628755569458,-0.2319418042898178,-0.062387239187955856,-2.485874652862549,-0.2399998903274536,0.18939149379730225,-2.3920974731445312,-0.23999926447868347,-0.2637767493724823,-2.3136024475097656,-0.2399967908859253,0.18739140033721924,-2.304266929626465,-0.23999619483947754,-0.47999998927116394,-2.495120048522949,-0.2399999052286148,-0.47999998927116394,-1.5405631065368652,-0.2089616060256958,0.14824002981185913,-3.1234889030456543,-0.23999999463558197,-0.47999998927116394,-2.755566358566284,-0.23999999463558197,0.47895610332489014,-2.079153537750244,-0.23985448479652405,0.18746881186962128,-2.3933420181274414,-0.23999927937984467,-0.47999998927116394,-2.8512420654296875,-0.23999999463558197,-0.47999998927116394,-1.8308401107788086,-0.23689386248588562,0.3181491792201996,-2.7638092041015625,-0.23999999463558197,-0.47999998927116394,-1.6439902782440186,-0.22399623692035675,-0.16958333551883698,-1.8926016092300415,-0.1368514597415924,0.4799754023551941,-2.315211772918701,-0.23999688029289246,-0.17755647003650665,-1.9446051120758057,-0.23913410305976868,-0.144013911485672,-2.058826208114624,1.7087959051132202,0.47999998927116394,-2.4410440921783447,-0.23999972641468048,-0.04469576105475426,-2.0671980381011963,-0.23982743918895721,-0.20690196752548218,-2.099400043487549,-0.23989154398441315,-0.47999998927116394,-2.2019236087799072,-0.2399779111146927,-0.47999998927116394,-1.54288911819458,-0.20936180651187897,0.02883804403245449,-1.877357840538025,-0.23810994625091553,-0.47999998927116394,-2.048797607421875,-0.23977670073509216,-0.36341118812561035,-2.339343309402466,-0.239997997879982,0.29738298058509827,-0.8491557240486145,1.2407397031784058,0.47999998927116394,-2.244903326034546,-0.23998922109603882,-0.34015533328056335,-2.48773193359375,-0.2399998903274536,0.29299619793891907,-1.657693862915039,-0.22553175687789917,-0.47999998927116394,-0.9588421583175659,-0.15948231518268585,-0.3150631785392761,-1.3351835012435913,-0.17108361423015594,0.34197697043418884,-1.40644109249115,-0.1834941804409027,-0.3295722007751465,-1.926537275314331,-0.2389242798089981,-0.31902530789375305,-1.6728399991989136,-0.22710418701171875,0.0659864991903305,-1.448411464691162,-0.19165872037410736,-0.20142753422260284,-1.7748446464538574,-0.23461021482944489,-0.3483101725578308,-1.5506126880645752,-0.21067258715629578,-0.1133914589881897,-1.9903576374053955,-0.23951192200183868,-0.47999998927116394,-0.7928568720817566,-0.14268259704113007,-0.23670046031475067,-1.15090811252594,-0.14920873939990997,-0.27114176750183105,-0.7885130047798157,-0.15205000340938568,-0.032824695110321045,-1.552572250366211,-0.21100063621997833,-0.47999998927116394,-1.7596272230148315,-0.23379451036453247,0.47584068775177,-1.4885238409042358,-0.19945277273654938,0.46200746297836304,-1.232406497001648,-0.15941797196865082,-0.4791169762611389,-1.534677505493164,-0.20793813467025757,-0.32175058126449585,-0.5341038107872009,-0.23785682022571564,0.34386685490608215,-0.6187716126441956,-0.05558910220861435,-0.3765174150466919,-1.1535316705703735,-0.15749970078468323,-0.12906841933727264,-0.887869656085968,-0.20388466119766235,0.07929028570652008,-0.578982949256897,-0.05023249611258507,-0.09011929482221603,-1.7688961029052734,-0.23430240154266357,-0.47999998927116394,-1.1342741250991821,-0.15275512635707855,-0.4180926978588104,-1.3438680171966553,-0.17242278158664703,-0.4799615144729614,-1.4488564729690552,-0.19174562394618988,-0.47999998927116394,-1.0455565452575684,-0.17415456473827362,-0.028761280700564384,-0.4214547276496887,0.018233761191368103,-0.08956844359636307,-1.3799501657485962,-0.17856858670711517,0.07272639870643616,-1.4319454431533813,-0.1884344518184662,0.21810148656368256,-1.034778118133545,-0.1458715796470642,-0.20720915496349335,-1.7695646286010742,-0.2343376874923706,0.1801353394985199,-0.6689496636390686,-0.2319965958595276,0.41418081521987915,-1.2775523662567139,-0.16320669651031494,-0.18507224321365356,-1.649815559387207,-0.2246621996164322,-0.333161860704422,-0.6668169498443604,-0.11790912598371506,-0.24800598621368408,-0.68909752368927,-0.11340479552745819,-0.11691175401210785,-0.5101407170295715,-0.02031784877181053,-0.20329879224300385,-0.8250566124916077,0.033388830721378326,-0.3363697826862335,-0.6519460082054138,-0.1212824285030365,0.029338348656892776,-1.6405768394470215,-0.22359687089920044,0.4754303991794586,-0.35656535625457764,0.09318044036626816,-0.1879967600107193,-0.3861734867095947,0.04048668593168259,0.10927627980709076,-1.2742916345596313,-0.1624649614095688,-0.32985037565231323,-1.1148582696914673,-0.16458866000175476,0.16115057468414307,-1.3737373352050781,-0.17746984958648682,0.47999998927116394,-0.42909038066864014,0.16017822921276093,-0.31730684638023376,-1.2325029373168945,-0.16068294644355774,0.27006790041923523,-0.9271016716957092,-0.17001765966415405,-0.04934636503458023,-1.4212290048599243,-0.18634876608848572,-0.052398405969142914,-0.5045768022537231,-0.02255227416753769,0.018978573381900787,-1.057019829750061,-0.1722961664199829,0.3846704661846161,-1.2774149179458618,-0.16380000114440918,0.12288698554039001,-0.7572931051254272,-0.20716984570026398,0.1085873618721962,-0.31386929750442505,0.18099582195281982,0.0884917676448822,-0.14899815618991852,0.19516976177692413,0.19627952575683594,-0.04458460211753845,-0.23999851942062378,0.3323989808559418,-0.4877086281776428,-0.22782441973686218,0.2458263337612152,-1.284523367881775,-0.16452379524707794,-0.0541037879884243,-0.31259748339653015,-0.056920189410448074,0.3183300495147705,-1.007853388786316,-0.18081532418727875,0.10912413895130157,-1.1749796867370605,-0.16028456389904022,0.05203329026699066,-0.27240821719169617,-0.23992876708507538,0.2708417475223541,-0.4017740786075592,-0.23955926299095154,0.4726554751396179,-1.042819857597351,-0.15805475413799286,-0.4735022485256195,0.2902976870536804,-0.23999999463558197,0.30025002360343933,-0.8964087963104248,-0.20230083167552948,0.2987315058708191,-0.4713616967201233,-0.08601245284080505,0.15662404894828796,-0.6188442707061768,-0.19364836812019348,0.16249985992908478,-0.23080453276634216,0.09399622678756714,0.283134400844574,0.044099170714616776,-0.23999974131584167,0.2980021834373474,0.23059575259685516,-0.23999999463558197,0.4330732226371765,0.2819879949092865,-0.17086535692214966,-0.16697785258293152,-0.9019968509674072,-0.147272527217865,-0.1882164031267166,0.5296834111213684,-0.23999999463558197,-0.008574319072067738,0.15678158402442932,-0.00502660172060132,0.1179686114192009,0.03154844045639038,-0.027338294312357903,0.42879560589790344,-0.18205589056015015,0.25354325771331787,-0.10526763647794724,0.016713587567210197,0.16786536574363708,0.20785215497016907,0.3823201656341553,-0.23999999463558197,0.1575179100036621,0.11565384268760681,-0.0043527004308998585,0.2850349545478821,0.2573220729827881,-0.23999999463558197,0.03001108020544052,-0.39245083928108215,-0.2396099716424942,0.3189682960510254,0.528347373008728,-0.23999999463558197,0.24825114011764526,-0.31235697865486145,0.2477327436208725,-0.2572667896747589,0.143697127699852,-0.23999996483325958,0.3095419704914093,-0.07448813319206238,0.11416000127792358,0.3335305154323578,-0.5281487703323364,0.043135255575180054,-0.340349406003952,0.6656470894813538,-0.23999996483325958,0.12885454297065735,-0.06542237848043442,0.23869642615318298,0.07572069019079208,-0.15241874754428864,0.38341382145881653,-0.14904406666755676,-0.3006283938884735,0.14088355004787445,-0.06615717709064484,0.669773519039154,-0.2399999499320984,0.27392852306365967,-0.9364689588546753,-0.1702001988887787,-0.4546893835067749,0.24069127440452576,-0.23999999463558197,0.15635846555233002,0.7936351299285889,-0.23999957740306854,0.1979260891675949,0.5224891304969788,-0.23999999463558197,0.11010285466909409,0.6552116870880127,-0.23999996483325958,-0.004947347566485405,-0.7194235920906067,-0.14257089793682098,-0.3829759359359741,-0.23984716832637787,0.3638913929462433,-0.2566165328025818,0.13288798928260803,0.29705914855003357,-0.043339088559150696,-0.16099399328231812,-0.23998808860778809,0.35469087958335876,-0.8295828104019165,-0.17430512607097626,-0.41659271717071533,-0.18984776735305786,0.1997579038143158,0.027164841070771217,0.06655395030975342,0.18146400153636932,0.05182638019323349,0.05076686292886734,0.5215649008750916,-0.06096332147717476,0.37286511063575745,-0.23999999463558197,0.00012517670984379947,0.5262616276741028,0.05195614695549011,-0.13133549690246582,0.45743080973625183,-0.17313721776008606,-0.12216303497552872,0.5836737155914307,-0.23999999463558197,-0.47999998927116394,-0.067751444876194,0.48531582951545715,-0.12269055098295212,0.3231505751609802,-0.23999999463558197,-0.32838141918182373,0.9135098457336426,-0.2399967759847641,0.2254749983549118,0.19692645967006683,-0.028848282992839813,-0.03644073009490967,0.44905924797058105,-0.23999999463558197,-0.39163869619369507,0.38195353746414185,-0.23999999463558197,-0.47999998927116394,0.19022336602210999,0.47341325879096985,-0.06223953142762184,0.8051645159721375,-0.1235525831580162,-0.2977693974971771,0.7664752006530762,-0.23999974131584167,-0.08256367594003677,0.5788230299949646,-0.13189123570919037,-0.22321419417858124,0.8630223870277405,-0.23999859392642975,-0.15197594463825226,0.5276868939399719,-0.23999999463558197,-0.1778709888458252,0.6922125220298767,-0.148890420794487,-0.47999998927116394,0.30720919370651245,-0.23999999463558197,-0.13269424438476562,1.043895959854126,-0.2399764508008957,0.25657665729522705,0.408037394285202,0.19455263018608093,-0.03708052635192871,0.5502856969833374,-0.23999999463558197,-0.3245045840740204,0.33764129877090454,0.036700088530778885,-0.07105457037687302,0.6627863049507141,-0.12856438755989075,-0.3542391061782837,0.9492826461791992,-0.2399943321943283,-0.24179613590240479,1.2179232835769653,-0.23975896835327148,0.10273357480764389,0.7198041081428528,-0.2399998903274536,-0.30285775661468506,0.6852830648422241,-0.2399999350309372,-0.17813828587532043,0.8163286447525024,-0.23999936878681183,0.06763225048780441,-0.04222512245178223,0.3158622682094574,-0.05958952382206917,1.057010531425476,-0.2399715632200241,-0.3639084994792938,1.0065855979919434,-0.2399863749742508,-0.10630250722169876,0.8883388638496399,-0.23999786376953125,-0.47999998927116394,0.2653866708278656,-0.23999999463558197,-0.4503014385700226,1.1651320457458496,-0.23987610638141632,-0.05956525355577469,1.2754628658294678,-0.1220044419169426,-0.47999998927116394,1.08901047706604,-0.2399553656578064,-0.21230846643447876,0.91867995262146,-0.2399965077638626,-0.0009707874851301312,0.8811665177345276,-0.23999810218811035,-0.3620742857456207,1.4519776105880737,-0.23697152733802795,-0.36027929186820984,1.305111289024353,-0.23932968080043793,-0.0660637691617012,0.31868886947631836,0.38211849331855774,-0.021048055961728096,1.0722577571868896,-0.10705836862325668,-0.47999998927116394,1.1604546308517456,-0.2398833930492401,0.2084304839372635,1.319392204284668,-0.1695658564567566,-0.33683162927627563,1.2050292491912842,-0.23979446291923523,-0.3515298366546631,1.7622919082641602,-0.20980298519134521,-0.47999998927116394,0.7592847347259521,-0.23999977111816406,-0.47999998927116394,1.4901667833328247,-0.23571112751960754,-0.2186194509267807,1.2089647054672241,-0.23978416621685028,-0.1878354400396347,1.176973819732666,-0.23985572159290314,-0.47999998927116394,1.6580660343170166,-0.16069021821022034,-0.47999998927116394,1.3329987525939941,-0.23908871412277222,0.06183680519461632,1.6100788116455078,-0.22863967716693878,-0.23207063972949982,1.3441451787948608,-0.2389724999666214,-0.22452865540981293,1.4177244901657104,-0.23781754076480865,0.15425367653369904,1.0214847326278687,-0.23998302221298218,-0.47999998927116394,1.44655179977417,-0.23712186515331268,-0.47999998927116394,1.591752290725708,-0.2300965040922165,-0.0869654193520546,1.7610864639282227,-0.08431993424892426,-0.4004631042480469,1.6315704584121704,-0.2267271727323532,-0.36745312809944153,1.0883240699768066,-0.23995578289031982,0.04080474376678467,1.487420916557312,-0.23581455647945404,0.030532212927937508,1.612949013710022,-0.22839727997779846,0.06504049152135849,1.4348809719085693,-0.23742364346981049,-0.09671587496995926,2.1566598415374756,-0.13672718405723572,-0.47999998927116394,1.8876934051513672,-0.18564297258853912,-0.47999998927116394,1.7497787475585938,-0.2118287980556488,0.08475098013877869,1.8735027313232422,-0.04641713947057724,-0.47999998927116394,1.8631211519241333,-0.19080094993114471,-0.17875276505947113,0.9943355917930603,-0.23998866975307465,0.11757400631904602,2.030390501022339,-0.15635381639003754,-0.47999998927116394,1.8190628290176392,-0.1996249109506607,-0.3262670040130615,1.5643208026885986,-0.23199857771396637,-0.47999998927116394,2.3381452560424805,0.038312915712594986,-0.28193747997283936,2.0673651695251465,-0.01449061743915081,-0.47999998927116394,1.6791592836380005,-0.2216486930847168,0.19259119033813477,1.3245383501052856,-0.2391689270734787,-0.47999998927116394,2.279003858566284,-0.14377593994140625,-0.1337326616048813,2.135380744934082,-0.14253807067871094,-0.23269487917423248,2.5086398124694824,-0.06034461036324501,-0.26441627740859985,1.9888615608215332,-0.16429811716079712,-0.22861948609352112,2.119270086288452,-0.14357343316078186,-0.35080695152282715,2.4487106800079346,-0.1720641404390335,-0.061572033911943436,1.9622671604156494,-0.16975733637809753,0.2402755320072174,2.4256436824798584,-0.16725566983222961,-0.208063542842865,2.206315279006958,-0.14002485573291779,0.36109721660614014,1.8283437490463257,-0.19782330095767975,-0.0035816302988678217,1.718738317489624,-0.2164861410856247,-0.0806405320763588,2.2833454608917236,-0.14408031105995178,-0.24463137984275818,1.5419350862503052,-0.2333233803510666,0.20676995813846588,1.5035942792892456,-0.23517389595508575,0.4787619113922119,1.9324541091918945,-0.1760636419057846,-0.05737331882119179,2.6953694820404053,-0.0046591865830123425,-0.47477561235427856,2.04182767868042,-0.15441255271434784,-0.10373876988887787,1.936596155166626,-0.17518511414527893,0.40658247470855713,2.4536054134368896,-0.1730998307466507,0.07089638710021973,1.8509217500686646,-0.1933080106973648,0.20115937292575836,2.322115421295166,-0.1488923877477646,0.001873310306109488,1.6441895961761475,-0.22549653053283691,0.45256853103637695,1.4918090105056763,-0.2356482446193695,0.34587663412094116,2.4324581623077393,-0.1686612069606781,0.3019393980503082,2.534928798675537,0.016663124784827232,-0.3867645263671875,2.6827123165130615,-0.14672397077083588,-0.20340649783611298,2.6651859283447266,-0.2141403704881668,0.1226358562707901,2.8030688762664795,0.1668000966310501,-0.36666491627693176,2.1005842685699463,-0.14598888158798218,0.23110686242580414,1.9619932174682617,-0.16981510818004608,0.09386538714170456,1.2953076362609863,-0.23939965665340424,0.4441932737827301,2.1768059730529785,-0.14032688736915588,0.04543682932853699,2.7476844787597656,-0.2246605008840561,0.28414830565452576,2.8865561485290527,-0.1899433583021164,-0.018391191959381104,1.958821415901184,-0.17047922313213348,-0.35901397466659546,1.3849296569824219,-0.23842690885066986,0.27946919202804565,2.793105125427246,-0.13021603226661682,-0.1274728775024414,2.464808225631714,-0.13840578496456146,-0.34643611311912537,1.738351821899414,-0.21360495686531067,-0.21587157249450684,2.7185566425323486,-0.22137436270713806,-0.10984355956315994,2.560258150100708,-0.15449130535125732,-0.15413202345371246,2.9400198459625244,0.23662534356117249,-0.1153549924492836,2.2974278926849365,-0.14576007425785065,0.1740516573190689,2.8116605281829834,0.3178648054599762,-0.452200710773468,1.7960541248321533,-0.20393425226211548,0.3848550617694855,2.841198205947876,0.09040859341621399,-0.11551882326602936,2.5231590270996094,-0.18793611228466034,0.17808383703231812,2.154651403427124,-0.14121732115745544,-0.077405646443367,2.584948778152466,-0.20039281249046326,-0.015018981881439686,1.684856653213501,-0.22095906734466553,0.3157104253768921,2.8929193019866943,0.42694297432899475,-0.42495080828666687,2.974433183670044,0.3589988648891449,-0.23010088503360748,2.8718442916870117,-0.23404590785503387,0.34017956256866455,2.9098784923553467,0.26928555965423584,-0.32087787985801697,1.2594881057739258,-0.23960281908512115,0.28170081973075867,2.803280830383301,-0.229716956615448,0.08300940692424774,2.4151875972747803,-0.08838985860347748,-0.47999998927116394,2.2992489337921143,-0.13089792430400848,-0.37648651003837585,2.9343936443328857,-0.02414744719862938,-0.02616020105779171,3.111705780029297,0.3279455602169037,-0.3453478217124939,3.013030767440796,0.2653111517429352,-0.411559134721756,2.9565956592559814,-0.2372061014175415,0.09049855172634125,2.2830564975738525,-0.12439236789941788,-0.47999998927116394,2.8263492584228516,0.12845614552497864,-0.4799463748931885,2.962401866912842,-0.23735593259334564,0.2296304553747177,2.5479543209075928,0.019379235804080963,-0.47999998927116394,2.630927324295044,0.04894221946597099,-0.2473643571138382,2.6674585342407227,0.1443202644586563,-0.3538512885570526],"v":[1.209604263305664,0.21411588788032532,0.4829092025756836,1.2045823335647583,0.22177132964134216,0.017363307997584343,1.0750982761383057,-0.6516531705856323,-0.31306660175323486,1.2205954790115356,-0.20813363790512085,0.39430320262908936,1.408806324005127,-0.13497331738471985,-0.0019317310070618987,1.080532193183899,-0.003521482227370143,-0.10962091386318207,1.2103171348571777,0.018265200778841972,0.1463642120361328,1.2235428094863892,-0.15794388949871063,-0.04829314723610878,1.050101399421692,-0.010740384459495544,-0.061937347054481506,1.2472413778305054,0.1696966141462326,0.6085253953933716,1.2696102857589722,-0.06978585571050644,0.38563039898872375,1.1492648124694824,-0.5101634860038757,-0.3749951124191284,1.2694939374923706,-0.2775287330150604,0.49312204122543335,1.2192370891571045,-0.5243930220603943,0.2418709695339203,0.9829355478286743,-0.0008478197269141674,-0.1626463234424591,1.214322805404663,-0.4751400351524353,0.45900797843933105,1.1886204481124878,-0.803636908531189,0.1500438004732132,0.9636130928993225,-0.012483268976211548,-0.0688706636428833,1.2681148052215576,-0.17304669320583344,0.5420844554901123,1.272828221321106,-0.2751372456550598,0.4778197109699249,1.162244439125061,-0.3789355158805847,-0.33813241124153137,1.2808167934417725,-0.09244607388973236,0.4138222336769104,1.2630078792572021,-0.5322109460830688,0.5263208746910095,0.9302784204483032,-0.00196703034453094,-0.001822121674194932,1.2873296737670898,-0.45175522565841675,0.48995038866996765,1.2396456003189087,-0.9944438338279724,0.3131445646286011,1.1440125703811646,-0.007453147321939468,-0.05330312252044678,1.0747162103652954,-0.23404037952423096,-0.02460898831486702,1.256084680557251,-0.41107046604156494,0.4201928377151489,0.951030433177948,-0.004429504740983248,-0.4960430860519409,1.2645397186279297,0.038849592208862305,0.6071658730506897,1.1471445560455322,-0.6263455748558044,0.40104928612709045,0.907310962677002,-0.00015676053590141237,-0.04779845476150513,1.2444217205047607,-0.6905961632728577,0.34628164768218994,1.2675681114196777,-1.0715389251708984,0.3504711985588074,0.8705118298530579,-0.011158156208693981,-0.8598887920379639,1.2939964532852173,0.08168290555477142,0.11666911095380783,1.0692509412765503,-0.3123321235179901,0.3088734447956085,0.5822346210479736,0.0004366798675619066,-0.5668695569038391,1.2651052474975586,-0.3541731536388397,0.5783702731132507,1.033204197883606,-0.1355472058057785,-0.053262859582901,0.8325990438461304,-0.000024511527954018675,-0.001179057639092207,1.18535315990448,-0.2678176760673523,0.08624859154224396,1.0594934225082397,-0.19115538895130157,0.3973599374294281,0.9841493964195251,-0.0008971065981313586,-0.25129517912864685,0.9933050274848938,0.10162992030382156,0.0022939422633498907,0.5368594527244568,0.1277477741241455,-0.00003253576505812816,0.943357527256012,-0.0006317310035228729,-0.15130992233753204,0.9157971143722534,0.09197467565536499,-0.07554757595062256,0.5802472829818726,0.0007492542499676347,-0.5655003190040588,0.8044717907905579,-0.0001265889295609668,-0.0008270496618933976,0.63471919298172,0.07310961186885834,-0.43427106738090515,0.9712360501289368,-0.003489552065730095,-0.20471195876598358,0.8084458708763123,-0.00028913363348692656,0.005014119669795036,1.1950433254241943,0.16932381689548492,-0.053407084196805954,0.47333407402038574,0.0026204162277281284,-0.011993086896836758,0.8775005340576172,-8.923227028390102e-7,-0.12607049942016602,1.0972410440444946,0.19344447553157806,0.2463775873184204,0.5706756114959717,0.0003775027289520949,-0.2730781137943268,0.8456528782844543,-0.000006585784831258934,-0.05136220529675484,0.49283260107040405,0.0012574995635077357,0.000941669219173491,1.037636637687683,-0.002349472139030695,-0.16649127006530762,0.9596512317657471,-0.0007331643719226122,-0.058825425803661346,0.6030222177505493,0.0010536597110331059,0.07200267910957336,0.3578086197376251,0.0003264939587097615,0.07679997384548187,0.8446853160858154,5.966049343442137e-7,-0.10661447793245316,1.046684741973877,-1.1955318512946178e-8,-0.018360156565904617,0.4761529564857483,0.00004412387716001831,0.06326376646757126,0.9689070582389832,-0.0007476850878447294,0.08718211203813553,1.1377419233322144,0.314103364944458,0.002068528439849615,0.9672648310661316,-1.5289518540129166e-9,0.13946139812469482,0.736977219581604,-9.242303917744721e-7,0.05526207759976387,0.6935642957687378,0.00039960246067494154,-0.0020446970593184233,0.544282853603363,0.00026684318436309695,0.0010071716969832778,0.5518704056739807,1.4394853364851912e-11,0.16544900834560394,0.8749405741691589,-0.0000018593027562019415,-0.004699133336544037,2.2427046298980713,-0.00006107149238232523,1.3882101774215698,0.9351711869239807,-3.0597263389609e-8,-0.0027751969173550606,0.3496687412261963,1.1550564785233267e-14,0.2817533314228058,1.0774000883102417,0,0.4357774257659912,0.8942516446113586,-1.679584804570311e-9,-0.2921774089336395,0.8045554757118225,-1.2013766370611023e-10,-0.1894829124212265,0.5012534260749817,0.00013685067824553698,0.0006034647230990231,1.6947288513183594,0,0.8685701489448547,0.8419215083122253,-2.5966386818510045e-9,-0.07780618220567703,1.3330734968185425,0,-0.8073376417160034,0.8998567461967468,-1.1326358517749213e-8,0.0023548470344394445,0.8626295328140259,0,-0.14190910756587982,1.26547110080719,0,0.09583298116922379,1.0625407695770264,0,-0.20018768310546875,1.2407145500183105,0,-0.007191188167780638,1.1589950323104858,-0.3029031455516815,0.0822146013379097,1.0634959936141968,0,-0.20205308496952057,1.125465750694275,0,-0.017874164506793022,1.4076546430587769,0,-0.007449619937688112,0.9346137642860413,0,-0.004871443845331669,1.1069797277450562,0,0.02842281386256218,0.3982355296611786,0,1.1119892597198486,2.169196605682373,-0.006954910233616829,0.5564514398574829,1.2959420680999756,0,-0.04072889685630798,1.5848747491836548,-0.20060598850250244,-0.15097245573997498,1.724562406539917,0,-0.2396099716424942,1.2025864124298096,-0.0021913270466029644,-0.11974567174911499,1.7408877611160278,-0.00023635268735233694,-0.27858367562294006,1.7654685974121094,0,-0.0021186687517911196,0.24886827170848846,0,-0.1894950270652771,2.037977695465088,0,-0.10135713964700699,1.9060089588165283,-0.0006784058059565723,-0.055545687675476074,1.6489450931549072,-0.00001256953601114219,-0.3814939856529236,1.8111425638198853,-0.0004810890823137015,-0.6018000245094299,1.3228559494018555,0,-0.000680364784784615,1.6108986139297485,-0.000962293182965368,-0.0016416910802945495,0.06994839012622833,0,0.0008801088551990688,1.495931625366211,-0.0033001648262143135,-0.24156637489795685,1.3027446269989014,-0.8615561127662659,-0.00640546390786767,1.3908377885818481,0,-1.8377612320819026e-7,0.5113846659660339,0,-0.22975300252437592,1.8644797801971436,-0.0012519363081082702,-0.12326324731111526,0.3926997482776642,0.00001628769859962631,0.0010157210053876042,1.6234675645828247,-0.000053994328482076526,-0.3144994080066681,1.445987582206726,-0.0018633034778758883,-0.03731865435838699,1.1903119087219238,-0.15387387573719025,-0.11618077754974365,1.6417046785354614,-0.0010657022939994931,-0.1396012157201767,0.46968841552734375,0,0.0015715466579422355,1.8095916509628296,0,-0.1554412990808487,1.7652548551559448,-0.0015019109705463052,-0.1674741804599762,1.565093994140625,1.5502917766571045,0,1.3516366481781006,-0.000947399705182761,-0.29010528326034546,1.1846143007278442,-0.7170400023460388,0.4323395788669586,0.8552885055541992,-0.174216628074646,0.42973995208740234,1.7269316911697388,0,-0.015063375234603882,1.6209267377853394,0,-0.1654941439628601,1.2212133407592773,-0.9149286150932312,0.2867279052734375,1.179928183555603,-0.7641949653625488,0.3792622983455658,1.8958767652511597,1.2932993173599243,0,1.7038800716400146,0,0.012743504717946053,1.2524640560150146,-0.5629755258560181,-0.15525999665260315,1.7750300168991089,-0.00002606433190521784,-0.19128955900669098,1.3091548681259155,-0.19471509754657745,-0.4585397243499756,1.230659008026123,-0.614423394203186,-0.04384231939911842,1.3068979978561401,9.314362614532001e-7,-0.00008491920016240329,1.5492758750915527,-0.44527462124824524,-0.2126009464263916,1.5069705247879028,-0.0021160070318728685,-0.2995481491088867,1.7107391357421875,0,-0.23095422983169556,1.2319004535675049,-0.3305438756942749,-0.00025750527856871486,1.0678308010101318,-1.247365951538086,0,1.4535151720046997,-0.000049886282795341685,-0.028207704424858093,1.2949565649032593,0.028041435405611992,-0.0006837315740995109,1.3625843524932861,-0.007789911236613989,-0.01046926062554121,1.362303614616394,1.8478790053677585e-8,-0.0000672668611514382,1.1958646774291992,-0.2240610122680664,0.918646514415741,1.2585638761520386,0.1053461804986,-0.003104438539594412,1.7810921669006348,0,-0.28820857405662537,1.0994945764541626,-0.005186447873711586,0.02288585714995861,1.1843005418777466,-0.0014658358413726091,-0.07489199936389923,1.0302937030792236,0.2647625207901001,-0.00833000149577856,1.1539620161056519,-6.147471509621028e-8,-0.7737560868263245,1.2450947761535645,0.0076253232546150684,-0.19379360973834991,1.4887198209762573,-0.0014619791181758046,-0.010636595077812672,1.1794281005859375,-0.7600736618041992,0,1.5417975187301636,0,-0.04507492482662201,1.8559043407440186,0,-0.2555813491344452,1.2974185943603516,0.0010110713774338365,-0.0027480549179017544,1.101397156715393,0.0004184445133432746,0.05907253548502922,1.4958348274230957,0.25714999437332153,-0.2662626802921295,1.0753061771392822,0.07454793900251389,-0.002386859618127346,1.213270664215088,0.0004870221600867808,-0.036320749670267105,2.1660513877868652,0,-0.0011099778348580003,1.133580207824707,-1.791121482849121,0,0.9892081618309021,-0.4732631742954254,-0.008219835348427296,1.1785061359405518,-0.670097827911377,0.26224640011787415,1.3054766654968262,0.01989470049738884,-0.15554222464561462,1.1724786758422852,0.0034138294868171215,-0.17901985347270966,1.2829831838607788,0.07213903218507767,-0.10836673527956009,1.1080960035324097,0.000003020648591700592,-0.08685079216957092,1.1545944213867188,-0.6215715408325195,0,1.5054242610931396,-0.0016648158198222518,-0.0015254516620188951,1.1912727355957031,-0.08959293365478516,1.5984652042388916,2.0100889205932617,0.02804134413599968,-0.003890284337103367,1.2570862770080566,-0.001257835770957172,-0.008472545072436333,1.3078656196594238,0.050618771463632584,-0.16109736263751984,1.0522464513778687,0.000015749757949379273,0.006860004737973213,1.2860256433486938,0,0.0000663358296151273,1.0504976511001587,-0.0003336342051625252,-0.41710612177848816,1.1952239274978638,0.0013508136617019773,-0.17176507413387299,1.325845718383789,0.023573538288474083,-0.24457767605781555,1.1546157598495483,-0.7440180778503418,0.44426533579826355,1.0949594974517822,0.0013621826656162739,-0.024208450689911842,1.4046552181243896,-0.0028153331950306892,-0.007214073091745377,1.3317853212356567,-0.054635223001241684,-0.0246213860809803,1.4118659496307373,3.932739911505223e-9,0.25637319684028625,2.020008087158203,0,-0.001927348435856402,1.3748887777328491,0.16655386984348297,-0.10275755822658539,0.33934855461120605,0,0.002093316288664937,1.3650588989257812,0,0.00018428500334266573,1.3333373069763184,0.10056368261575699,-0.06741268932819366,0.9588916301727295,0.000003234310042898869,-0.24305805563926697,1.1629579067230225,0.000021883110093767755,0.36584362387657166,1.150106430053711,0.00008254846034105867,0.028118129819631577,1.4169468879699707,0.00012626648822333664,0.0008493546047247946,1.3632065057754517,0.000003067146508328733,-0.0002035411016549915,1.3276610374450684,0.234597310423851,-0.1436060667037964,1.2759665250778198,0,0.000012633358892344404,1.364333987236023,-0.000470711849629879,-0.03642434999346733,1.3793696165084839,0.00346200168132782,-0.28728511929512024,1.3587031364440918,0.000025361377993249334,0.00019046242232434452,1.1940252780914307,0,0.000021267287593218498,1.5105557441711426,0.05061879754066467,-0.6652162671089172,1.3837114572525024,2.562819068074873e-9,-0.00012750095629598945,1.333985686302185,0.16317561268806458,-0.07398319244384766,1.2845834493637085,-1.937104344367981,-0.007862582802772522,0.9202820658683777,0.000063876919739414,0.026968862861394882,1.1939218044281006,0.013461141847074032,-0.14269311726093292,1.1614322662353516,-2.2283577919006348,0,1.0711718797683716,0.00000744865292290342,-0.28652310371398926,1.1997041702270508,0.0034018612932413816,-0.11980481445789337,1.3026666641235352,0.0024808638263493776,0.0006917538703419268,1.3512214422225952,0.0006059884908609092,0.001042843796312809,1.3164710998535156,0.23146337270736694,-0.0049005844630301,1.2589099407196045,0.029520658776164055,-0.00004734133472084068,1.0890089273452759,0.0038958166260272264,0.06342383474111557,1.1160054206848145,0.00005452246841741726,-0.16039596498012543,1.2427031993865967,-2.463369369506836,0,1.0336744785308838,0.00022580144286621362,0.16246168315410614,1.0217931270599365,-0.000702674500644207,-0.018386803567409515,1.2860908508300781,0.147000253200531,-0.0001990513119380921,1.3119462728500366,-0.09052342921495438,0.20557714998722076,1.4444574117660522,0.20396432280540466,-0.17428170144557953,1.353643536567688,0.25161972641944885,0.018869685009121895,1.15958833694458,0.01590496487915516,-0.06514107435941696,1.2820849418640137,0.1354304850101471,-0.06299135833978653,1.3559705018997192,0.2641662359237671,0.00232317834161222,1.285662293434143,0.07084145396947861,-0.05205858498811722,1.3061742782592773,0.2252436876296997,-0.10867804288864136,1.3260945081710815,0.009654021821916103,0.0004533317405730486,1.290940761566162,-0.2323172688484192,0.24908937513828278,1.3516383171081543,0.1021362915635109,0.17894983291625977,1.2431097030639648,-0.4997715651988983,0.3517388701438904,1.283652663230896,0.21997487545013428,0.00031238218070939183,2.0447402000427246,0.12097352743148804,-0.07631786167621613,1.5211421251296997,0.29249805212020874,0.040605057030916214,1.368730902671814,0.10419060289859772,0.03809607774019241,1.3565666675567627,0.24310186505317688,0.004075916949659586,1.1336995363235474,-0.03415485471487045,0.1798841804265976,1.2926141023635864,-0.05804981291294098,0.3006245195865631,1.2687878608703613,-0.03750500828027725,0.12901563942432404,1.3006197214126587,-0.24038487672805786,0.37348559498786926,1.1937892436981201,-0.4155420958995819,0.354501873254776,1.2704682350158691,0.07322416454553604,-0.00041733175748959184,1.325828194618225,0.06169018894433975,0.24566198885440826,1.347532868385315,0.20166249573230743,0.004529024474322796,1.2907947301864624,0.2513866722583771,0.0005264616338536143,1.3066771030426025,-0.2247597575187683,0.22654001414775848,1.221706748008728,-0.3476603925228119,0.3854224681854248,1.3638715744018555,0.23612911999225616,-0.03622306138277054,1.533913016319275,0.29459768533706665,-0.06751146167516708,1.3459270000457764,-0.03405705839395523,0.24513933062553406,1.3704513311386108,0.07887028902769089,-0.06751865148544312,1.216189980506897,-0.07831904292106628,0.1073361411690712,1.2904528379440308,0.14049790799617767,0.08588989078998566,1.3345284461975098,0.15901604294776917,-0.03554908186197281,1.314924716949463,-0.2551279067993164,0.28665852546691895,1.2336866855621338,-0.40948304533958435,0.32531705498695374,1.235793948173523,-0.28992438316345215,0.33027029037475586,1.2025268077850342,-0.180742084980011,0.5057317614555359,1.222363829612732,-0.5472707748413086,0.346493661403656,1.6169556379318237,0.20240196585655212,-0.05212904512882233,1.2071502208709717,-0.1957932710647583,0.4334774315357208,1.1734416484832764,-0.4464721083641052,0.4299488961696625,1.3408288955688477,0.16665726900100708,0.015136871486902237,1.307478666305542,-0.14997316896915436,0.16253884136676788,1.6017166376113892,0.27185535430908203,-0.0010034680599346757,1.1949491500854492,0.007496727164834738,0.45804572105407715,1.4444547891616821,0.039945460855960846,-0.184255450963974,1.3109643459320068,-0.3858490288257599,0.3163493573665619,1.2614811658859253,0.24010396003723145,-0.062388479709625244,1.193097710609436,-0.45221853256225586,0.3958689868450165,1.2461402416229248,-0.20568658411502838,0.08787765353918076,1.336625576019287,0.10993917286396027,0.08804330974817276,1.2462162971496582,-0.6579871773719788,0.3247884511947632,1.137799859046936,-0.32732513546943665,0.5753273367881775,1.1575074195861816,-0.5470525026321411,0.6419309973716736,0.9342262744903564,-0.00008366738620679826,0.10935253649950027,1.191035270690918,-0.654399037361145,0.055943120270967484,1.3286038637161255,0.12140609323978424,-0.03353055194020271,1.1749268770217896,-0.8026569485664368,0.4561206102371216,1.342307209968567,-0.25277405977249146,0.2837814688682556,1.432836651802063,-0.07398287206888199,0.179823100566864,1.2003604173660278,-0.0040028877556324005,0.03149031847715378,1.2068933248519897,-0.00623762933537364,-0.056755565106868744,1.3463947772979736,-0.03452536091208458,0.06530066579580307,1.3268718719482422,-8.136493256927224e-9,0.19613011181354523,1.242239236831665,-0.2277078628540039,0.06418050080537796,1.179742693901062,-0.5977285504341125,0.35272151231765747,1.2322338819503784,-0.6787897944450378,0.28933149576187134,1.1506619453430176,-0.697955846786499,0.5963338017463684,0.8413631916046143,-0.00010193357593379915,-0.005242981947958469,1.2177000045776367,-7.566951509829778e-9,0.23086929321289062,0.880760133266449,0.08712685108184814,-0.23293717205524445,1.2983160018920898,-0.18771491944789886,0.2835196852684021,1.197318196296692,-0.0016438609454780817,-0.01439677644520998,1.1661756038665771,-0.8591234087944031,0.5179911851882935,1.1614526510238647,-1.1722337007522583,0.635388970375061,1.1708753108978271,-0.16443173587322235,0.6022593379020691,1.1817052364349365,-0.6609397530555725,0.675331711769104,1.310793399810791,0,-0.015816858038306236,1.163313388824463,-1.0565773248672485,0.592815637588501,0.9993891716003418,-0.0002353448508074507,0.03382604941725731,1.2135430574417114,-0.013722029514610767,0.11287228763103485,1.1933561563491821,1.4797436165281397e-8,-0.027895646169781685,1.158711552619934,-0.023485997691750526,0.5678557753562927,1.2886579036712646,-0.0002799179346766323,0.1570708006620407,1.1483696699142456,-0.8315575122833252,0.6815792918205261,1.2357308864593506,-0.02774158865213394,0.3742152154445648,1.2423371076583862,0.000001875766884040786,-0.004671487491577864,1.2065802812576294,-0.38018229603767395,0.6539782881736755,1.1342785358428955,-0.0895744264125824,0.769746720790863,1.1694386005401611,-0.25621166825294495,0.5068922638893127,1.2137517929077148,9.947038961399812e-7,-0.014881303533911705,1.3517775535583496,-0.11798226088285446,0.16181445121765137,1.0712476968765259,-0.000024576456780778244,-0.09359171986579895,1.2321527004241943,0.000012227537808939815,-0.03393526002764702,1.229905366897583,-2.0277191481454793e-8,0.032497476786375046,1.228088617324829,-0.000001936589569595526,-0.0067340536043047905,1.3403199911117554,-0.14599266648292542,0.31487876176834106,1.135505199432373,0.03758217766880989,0.611390233039856,1.2058216333389282,-0.32482415437698364,0.7179644703865051,1.1185942888259888,-0.0003078373265452683,0.09354028850793839,1.3341000080108643,-0.16913263499736786,0.26962631940841675,1.1763311624526978,-0.3180534541606903,0.5834916830062866,1.2113608121871948,-0.4964406490325928,0.6613668203353882,1.200768232345581,-0.04839276522397995,0.9777735471725464,1.1765776872634888,-0.0012681292137131095,-0.15148554742336273,1.244217038154602,-0.6996539831161499,0.6205371618270874,0.995003879070282,-0.2798907160758972,0.18368449807167053,1.1265000104904175,0.0005253312410786748,-0.0007963561220094562,1.144797921180725,-0.006835431791841984,0.8515065908432007,0.9697632193565369,0.0003652934683486819,-0.3035878539085388,1.188777208328247,0.00007594519411213696,0.008104815147817135,1.165791392326355,-0.6954887509346008,0.45443859696388245,1.0845234394073486,-0.000003964164534409065,0.0015598032623529434,0.9605644941329956,0,-0.004206866957247257,1.2528289556503296,-0.20462235808372498,0.9448346495628357,1.2576570510864258,0.04539191722869873,0.08895565569400787,1.234000563621521,0.000007592604561068583,0.038511723279953,1.1738535165786743,-0.3331851363182068,0.21074631810188293,1.2163419723510742,0.0001469650596845895,-0.04130742698907852,1.1122565269470215,-0.006096475291997194,-0.10341337323188782,1.0928709506988525,0.07131193578243256,0.0007730959332548082,1.0180844068527222,0.0002874970086850226,0.004523145034909248,1.190349817276001,0.00048057062667794526,0.029710492119193077,1.2427585124969482,-0.6208383440971375,0.774238646030426,1.0602563619613647,-0.0020679342560470104,-0.22037062048912048,1.2238287925720215,-0.6606249809265137,0.5422490835189819,1.1562672853469849,-0.11215989291667938,0.07311654835939407,1.2124359607696533,0.0002470092149451375,-0.019007306545972824,1.1700364351272583,0.003931467421352863,0.016670189797878265,1.1860235929489136,-0.0008923894492909312,-0.0570756234228611,1.2491750717163086,-0.0016607040306553245,-0.05946304276585579,1.225549578666687,0.000017055092030204833,-0.024104392156004906,1.1816222667694092,-0.2159595936536789,0.7090274095535278,1.1915507316589355,0.0010188024025410414,-0.05827387794852257,1.2057183980941772,0.00029294186970219016,-0.017580892890691757,1.2066642045974731,0.00011155426182085648,-0.000473039282951504,0.8963176012039185,0,-0.22130173444747925,1.1722379922866821,0.0021647573448717594,0.013463313691318035,1.2298883199691772,0.0636558085680008,-0.00014898504014126956,1.2058426141738892,0.0008778087212704122,0.008361310698091984,1.2034555673599243,0.00008236830763053149,-0.030929766595363617,1.1739170551300049,0.00027985312044620514,-0.03717973455786705,1.1489540338516235,0.03489034250378609,0.03889613226056099,1.1698991060256958,0.00982470903545618,0.006072598043829203,1.2675437927246094,-0.3794788122177124,0.9470154643058777,1.2637568712234497,0.1386796087026596,-0.00008702900231583044,1.1463544368743896,0.001987881027162075,-0.06492133438587189,1.2183338403701782,-0.0632181167602539,-0.07538694143295288,1.2026352882385254,0.003134549129754305,-0.005218003410845995,1.1443310976028442,0.19466596841812134,-0.00030365859856829047,1.2066560983657837,0.000789471494499594,0.00019538647029548883,1.1931577920913696,0.04921840503811836,0.02400955930352211,1.1826841831207275,0.0035568198654800653,-0.027600694447755814,1.1231679916381836,0.0034988895058631897,-0.0001476963807363063,1.0929731130599976,0.1198427602648735,-0.00036235820152796805,1.1232792139053345,0.012343556620180607,0.007141808979213238,1.1891778707504272,0.10555846244096756,0.009577792137861252,1.1713981628417969,0.013791301287710667,0.05701785907149315,1.1347322463989258,0.026508592069149017,-0.046021852642297745,1.2054557800292969,0.0012897637207061052,-0.0006097205332480371,1.193474292755127,0.03535269945859909,0.0003930864913854748,1.1843435764312744,0.0947863906621933,0.03714025393128395,1.0917218923568726,0.25459930300712585,-0.08449921011924744,1.187593698501587,0.11822822690010071,0.01147869136184454,1.2448939085006714,0.0009011098300106823,0.0534287691116333,1.1381776332855225,0.04587240144610405,0.039964016526937485,1.1721949577331543,0.10570614039897919,-0.04757026955485344,1.1885547637939453,0.03205610811710358,0.004391984082758427,1.2154664993286133,0.17340445518493652,-0.0005956069799140096,1.205868124961853,0.2563426196575165,-0.0003997026360593736,1.1713720560073853,0.1914653182029724,-0.04749325290322304,1.107024073600769,0.182813823223114,-0.0009023299207910895,1.2029465436935425,0.2510869801044464,-0.040332943201065063,1.1933350563049316,0.00024313302128575742,-0.006451284978538752,1.2244235277175903,0.22562475502490997,-0.0005885737482458353,1.0695990324020386,0.20922152698040009,-0.037739600986242294,1.1500730514526367,0.0779564380645752,0.0003936447901651263,1.1948095560073853,0.13867893815040588,-0.26468655467033386,1.145939826965332,0.41823554039001465,-0.00013560961815528572,1.1335288286209106,0.14123286306858063,-0.05238548293709755,1.1536158323287964,0.011929306201636791,0.00015986728249117732,1.25788414478302,-0.13280422985553741,-0.15676718950271606,1.2516803741455078,0.08194011449813843,-0.11687891185283661,1.2450520992279053,-0.20745858550071716,-0.1921788901090622,1.1468526124954224,0.22292175889015198,-0.13067638874053955,1.273521900177002,0.1317126452922821,-0.10004889965057373,1.2080785036087036,-0.25786012411117554,-0.26260149478912354,1.178263545036316,0.24160942435264587,-0.3389442265033722,1.3056219816207886,-0.27234604954719543,-0.3658905327320099,1.1966657638549805,-0.023631513118743896,-0.5019506216049194,1.189499855041504,0.2366860806941986,-0.06276749074459076,1.1823761463165283,0.17361736297607422,-0.013755962252616882,1.2942166328430176,-0.15136095881462097,-0.17295388877391815,1.1574246883392334,0.06807773560285568,0.04605213552713394,1.1224361658096313,0.0510595440864563,-0.029424095526337624,1.1953496932983398,0.2536141276359558,-0.05892645940184593,1.339683175086975,-0.004385689273476601,-0.15517331659793854,1.168728232383728,0.18826471269130707,-0.18466369807720184,1.103919506072998,0.23380590975284576,-0.40105903148651123,1.1691982746124268,-0.24843746423721313,-0.4543241560459137,1.1725947856903076,0.24131634831428528,-0.1834910809993744,1.1904152631759644,-0.17490096390247345,-0.24054346978664398,1.1474974155426025,0.12123511731624603,-0.1478828340768814,1.2485369443893433,0.05218705162405968,-0.07325926423072815,1.1814987659454346,-0.24754102528095245,-0.47943195700645447,1.2060201168060303,0.08490917831659317,-0.07352734357118607,1.3178620338439941,-0.6735169291496277,-0.5292518138885498,1.2336220741271973,-0.17843015491962433,-0.5748674273490906,1.2781236171722412,0.06315365433692932,0.1021866500377655,1.1855230331420898,0.12630103528499603,-0.5360730290412903,1.2687584161758423,0.26058149337768555,-0.14037710428237915,1.103016972541809,0.008293887600302696,-0.12889212369918823,1.1864185333251953,0.01811445876955986,-0.15553773939609528,1.1723215579986572,-0.11655338108539581,-0.3894091546535492,1.211747407913208,-0.6956250071525574,-0.3678229749202728,1.140381932258606,0.23521855473518372,-0.045463696122169495,1.1452957391738892,0.020341068506240845,-0.015451272949576378,1.3099082708358765,-0.6557937264442444,-0.3790467083454132,1.4812660217285156,-0.2947595417499542,-0.19507552683353424,1.1826170682907104,0.1862812489271164,-0.03364524990320206,1.2669293880462646,-0.16033171117305756,-0.49189960956573486,1.2702739238739014,-0.5774978399276733,-0.38222536444664,1.2274214029312134,-0.2404233068227768,0.15540584921836853,1.1819097995758057,-0.14688944816589355,-0.6046869158744812,1.2161983251571655,0.22121162712574005,0.21789394319057465,1.139670968055725,0.21147677302360535,-0.27149248123168945,1.2324507236480713,-0.3913830816745758,-0.03428940102458,1.1984323263168335,-0.24876239895820618,-0.49020859599113464,1.253986120223999,0.05392816290259361,-0.21105444431304932,1.275390625,-0.23952220380306244,-0.3371413052082062,1.111863136291504,0.14210999011993408,-0.05866290256381035,1.1925967931747437,0.2626591920852661,0.3343360126018524,1.2315946817398071,0.0024327230639755726,0.25814709067344666,1.222271203994751,-0.057256508618593216,-0.23442767560482025,1.2517662048339844,0.0789414644241333,0.15660302340984344,1.1184519529342651,0.005915712099522352,0.014092889614403248,1.184736728668213,-0.09476534277200699,-0.5140084028244019,1.304173469543457,0.15065661072731018,-0.0009238746133632958,1.3301222324371338,-0.008288996294140816,0.04431222006678581,1.2226639986038208,-0.6150826811790466,-0.10338845103979111,1.263167142868042,-0.005908437538892031,0.29069116711616516,1.2951968908309937,0.05966413393616676,0.1291494518518448,1.0399144887924194,-0.03591989353299141,-0.382983922958374,1.2283642292022705,0.1201983094215393,0.0005946552264504135,1.309514045715332,0.06528232991695404,0.00656732264906168,1.0475701093673706,-0.02471824735403061,-0.17168715596199036,1.2204281091690063,0.15478914976119995,-0.0021481786388903856,1.2155568599700928,-0.23014333844184875,-0.1407148391008377,1.1927564144134521,0.04093264415860176,-0.13004928827285767],"clock":5.999999999999984,"accumulator":0};const fluids=new Map();
onmessage=({data:m})=>{
 if(m.op==='dispose'){fluids.delete(m.id);return;}
 if(m.op==='init'){
  const f=m.kind==='stream'?new StreamFluid():new VolumeFluid(m.seed,m.settings.fillAmount??.26);fluids.set(m.id,f);Object.assign(f,m.settings);
  if(m.kind==='stream'){f.p.set(STREAM_SEED.p);f.old.set(STREAM_SEED.old);f.pred.set(STREAM_SEED.p);f.v.set(STREAM_SEED.v);f.clock=STREAM_SEED.clock;f.accumulator=STREAM_SEED.accumulator;f.rebuildNeighbors();}
 }else if(m.op==='impulse'){fluids.get(m.id)?.impulse(...m.args);return;}
 const f=fluids.get(m.id);if(!f)return;const started=performance.now();
 if(m.op==='step'){Object.assign(f,m.settings);f.step(...m.args);}
 const field=f.reconstruct().slice(),p=f.p.slice(),old=f.old.slice();
 postMessage({id:m.id,field,p,old,solverMs:performance.now()-started,count:f.count,clock:f.clock,accumulator:f.accumulator},[field.buffer,p.buffer,old.buffer]);
};`],{type:'text/javascript'}));
const fluidWorkers=Array.from({length:Math.max(1,Math.min(MOZER_PROFILE.workers,(navigator.hardwareConcurrency||4)-1))},()=>new Worker(fluidWorkerURL));URL.revokeObjectURL(fluidWorkerURL);
const workerFluids=new Map();let fluidWorkerId=0;
for(const worker of fluidWorkers)worker.onmessage=({data:m})=>{const f=workerFluids.get(m.id);if(!f)return;Object.assign(f,m);f.version++;f.busy=false;};
for(const worker of fluidWorkers)worker.onerror=e=>console.error('Fluid worker',e.message);
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
const stage = document.querySelector('#stage');
const labels = document.querySelector('#labels');
const coreLabel = document.querySelector('#core-label');
const brands = [{"id":"mozer","url":"https://mozer-oil.ru/","name":"MOZER","html":"<img class=\"logo logo-mozer\" alt=\"MOZER\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-0.svg\" />","x":-2.3,"y":1.65},{"id":"michi","url":"https://michioil.com/","name":"MICHI","html":"<img class=\"logo logo-michi\" alt=\"MICHI\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-1.svg\" />","x":2.3,"y":1.65},{"id":"wezer","url":"https://mozergroup.ru/brendy/wezer","name":"WEZER","html":"<img class=\"logo logo-wezer\" alt=\"WEZER\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-2.svg\" />","x":-2.3,"y":-1.65},{"id":"unix","url":"https://mozergroup.ru/brendy/unix","name":"UNIX","html":"<img class=\"logo logo-unix\" alt=\"UNIX\" draggable=\"false\" src=\"assets/mozer-v2/logos/logo-3.svg\" />","x":2.3,"y":-1.65}];
const motionPreference = matchMedia('(prefers-reduced-motion: reduce)');
let reducedMotion = motionPreference.matches;
motionPreference.addEventListener('change', e => { reducedMotion = e.matches; });

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
let adaptiveBackgroundHz=MOZER_PROFILE.backgroundHz;
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
 if(now-perfAt<2000||frameSamples.length<20)return;
 const mean=frameSamples.reduce((sum,t)=>sum+t,0)/frameSamples.length,p95=percentile(frameSamples,.95),fps=Math.round(1000/mean);
 performanceHistory.push({fps,p95:Number(p95.toFixed(1)),scale:renderFactor,focus:Number(stage.dataset.focus)});if(performanceHistory.length>20)performanceHistory.shift();stage.dataset.performanceHistory=JSON.stringify(performanceHistory);
 stage.dataset.backgroundHz=String(adaptiveBackgroundHz);stage.dataset.fps=String(fps);stage.dataset.frameP95=p95.toFixed(1);stage.dataset.quality=renderFactor.toFixed(2);stage.dataset.framesOver25=String(frameSamples.filter(t=>t>25).length);
 if(fluidSettings.autoQuality&&now-lastResolutionAt>2200){
  if(fps<MOZER_PROFILE.fps-3||(p95>1500/MOZER_PROFILE.fps&&frameSamples.filter(t=>t>1500/MOZER_PROFILE.fps).length>frameSamples.length*.1)){requestQuality(Math.max(minimumQuality(),renderFactor*.90),Math.max(.85,rearFactor*.94),now);
   // Reduce the expensive rear pass frequency before sacrificing further spatial detail.
   adaptiveBackgroundHz=Math.max(Math.min(MOZER_PROFILE.backgroundHz,MOZER_PROFILE.mobile?15:20),adaptiveBackgroundHz-5);}
  else if(fps>=MOZER_PROFILE.fps-1&&p95<1150/MOZER_PROFILE.fps&&now-lastRaiseAt>8000&&renderFactor<1&&(lastMain*.5+lastRear*.25)<7){renderFactor=Math.min(1,renderFactor+.03);rearFactor=Math.min(1,rearFactor+.03);resolutionPending=true;lastResolutionAt=now;lastRaiseAt=now;}
 }
 fpsIndicator.textContent=fps+' FPS';
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
  if(MOZER_PROFILE.fps<60&&now-lastDrawAt<1000/MOZER_PROFILE.fps-1){frameId=requestAnimationFrame(frame);return;}lastDrawAt=now;
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
  px+=((reducedMotion?0:mx*pointerBlend)-px)*settle;py+=((reducedMotion?0:my*pointerBlend)-py)*settle;
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
  if(physicsAccumulator>=1/MOZER_PROFILE.physicsHz){
   const fluidStart=performance.now();const physicsDt=physicsAccumulator;physicsAccumulator=0;
       let densityChanged=false;const airCountChanged=uploadedAirCount!==fluidSettings.airBubbleCount;
    basins.forEach((basin,i)=>{
    basin.setAmplitude(fluidSettings.amplitude);basin.force=fluidSettings.force;basin.viscosity=fluidSettings.viscosity;basin.splash=fluidSettings.splash;
    if((i===0||basin!==basins[i-1])&&!reducedMotion&&!fluidSettings.paused)basin.step(physicsDt*fluidSettings.timeScale,gravity.x-Math.max(-2,Math.min(2,bodyAcceleration.x)),gravity.z,1.75*fluidSettings.speed);
    if(basinVersions[i]!==basin.version||airCountChanged)updateAir(basin,i);
    if(basinVersions[i]!==basin.version){wavePixels.set(basin.reconstruct(),i*32*32*32);basinVersions[i]=basin.version;densityChanged=true;}
   });
   if(densityChanged)mozerUploadTexture(waveTexture);uploadedAirCount=fluidSettings.airBubbleCount;fluidCpu=performance.now()-fluidStart;
  }
  stage.dataset.streamParticles=String(backgroundFlow.particleCount());stage.dataset.streamReady=String(backgroundFlow.ready());
  stage.dataset.particles=String(basins.reduce((count,basin)=>count+basin.count,0));
  cursorLocal.set(px*viewW*.5,py*viewH*.5,0).sub(center).applyMatrix4(inverse).divideScalar(zoom);
  const activity=reducedMotion?0:Math.min(1,Math.hypot(px,py)*2.8);
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
 mozerReport('sceneBuilt');
 async function prepareScene(){
  sceneReady=false;stage.dataset.startupState='compiling';fpsIndicator.textContent='Загрузка…';
  // Let the background and controls paint before creating GPU programs.
  await new Promise(resolve=>requestAnimationFrame(()=>setTimeout(resolve,0)));
  try{
   await Promise.all(MOZER_ASSET_PROMISES);updateMozerBackdrop();mozerReport('compileStart');
   renderer.setRenderTarget(backgroundTarget);const rearReady=renderer.compileAsync(backgroundFlow.scene,camera);
   renderer.setRenderTarget(MOZER_PROFILE.postProcess?dofTarget:null);const mainReady=renderer.compileAsync(scene,camera);
   renderer.setRenderTarget(null);const postReady=MOZER_PROFILE.postProcess?renderer.compileAsync(dofScene,camera):Promise.resolve();
   await Promise.all([rearReady,mainReady,postReady]);
   mozerReport('compileReady');sceneReady=true;stage.dataset.startupState='ready';
   previous=performance.now();perfAt=previous;frameSamples=[];
   wakeUntil=performance.now()+500;if(!document.hidden&&externallyActive)frameId=requestAnimationFrame(frame);
  }catch(error){renderer.setRenderTarget(null);stage.dataset.startupState='error';fpsIndicator.textContent='Не удалось загрузить сцену';console.error('Scene startup',error);}
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
