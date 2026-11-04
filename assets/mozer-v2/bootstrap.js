const MOZER_ASSET_ROOT = new URL('./', document.currentScript.src);
const MOZER_PROFILE = (() => {
  const forced = new URLSearchParams(location.search).get('quality');
  const mobile = forced === 'mobile' || (forced !== 'desktop' && (matchMedia('(pointer:coarse)').matches || navigator.userAgentData?.mobile || /Android|iPhone|iPad|iPod/i.test(navigator.userAgent) || innerWidth < 600));
  return Object.freeze(mobile
    ? {mobile:true,fps:30,physicsHz:20,backgroundHz:20,workers:2,volumes:2,drops:4,particles:5,airBubbles:6,fluidSamples:32,flowSamples:48,junctionSteps:28,armSteps:32,maxDpr:2,minDpr:1.5,pixelBudget:1600000,postProcess:false}
    : {mobile:false,fps:60,physicsHz:30,backgroundHz:30,workers:3,volumes:5,drops:8,particles:13,airBubbles:32,fluidSamples:32,flowSamples:64,junctionSteps:36,armSteps:48,maxDpr:1.5,pixelBudget:2100000,postProcess:true});
})();
const MOZER_HOST = {scale:1,screen:0,active:true,safeBottom:0,background:null,scroll:0,displayWidth:0,displayHeight:0,deviceDpr:0};
function mozerScreen(width){return [1200,960,640,480,320].find(n=>width>=n)||320;}
let mozerLayoutCache=null;
function mozerLayout(){const s=MOZER_HOST.screen||mozerScreen(innerWidth),entry=MOZER_RESPONSIVE.at(s);if(mozerLayoutCache?.entry!==entry)mozerLayoutCache={entry,value:{x:entry.molecule.x-50,y:50-entry.molecule.y,scale:entry.molecule.scale/MOZER_DEFAULTS.moleculeScale,...entry}};return mozerLayoutCache.value;}
window.mozerStartup={startedAt:new Date().toISOString(),began:performance.now(),phases:{},longTasks:0,longTaskMs:0,maxLongTaskMs:0};
function mozerReport(name){if(name)mozerStartup.phases[name]=Math.round(performance.now());document.documentElement.dataset.startupReport=JSON.stringify(mozerStartup);}
if(new URLSearchParams(location.search).has('debug')){
  try{new PerformanceObserver(list=>{for(const e of list.getEntries()){mozerStartup.longTasks++;mozerStartup.longTaskMs+=Math.round(e.duration);mozerStartup.maxLongTaskMs=Math.max(mozerStartup.maxLongTaskMs,Math.round(e.duration));}mozerReport();}).observe({type:'longtask',buffered:true});}catch{}
}
mozerReport('document');
window.addEventListener('message',event=>{
  if(event.source===parent&&event.data?.type==='mozer-v2:scroll'){
    MOZER_HOST.scroll=Math.max(-1,Math.min(1,Number(event.data.progress)||0));return;
  }
  if(event.source!==parent||!event.data||event.data.type!=='mozer-v2:layout')return;
  const m=event.data;
  MOZER_HOST.scale=Math.max(.25,Math.min(4,Number(m.scale)||1));
  MOZER_HOST.displayWidth=Math.max(0,Number(m.displayWidth)||0);MOZER_HOST.displayHeight=Math.max(0,Number(m.displayHeight)||0);
  MOZER_HOST.deviceDpr=Math.max(0,Math.min(5,Number(m.deviceDpr)||0));
  MOZER_HOST.screen=[320,480,640,960,1200].includes(Number(m.screen))?Number(m.screen):mozerScreen(Number(m.viewportWidth)||innerWidth);
  if(Number.isFinite(m.pixelWidth)&&Number.isFinite(m.pixelHeight)&&m.pixelWidth>0&&m.pixelHeight>0){
    const w=Math.round(m.pixelWidth),h=Math.round(m.pixelHeight);document.documentElement.style.setProperty('--mozer-width',w+'px');document.documentElement.style.setProperty('--mozer-height',h+'px');
    const meta=document.querySelector('meta[name="viewport"]'),content='width='+w+',initial-scale=1';if(meta&&meta.content!==content)meta.content=content;
  }
  MOZER_HOST.active=m.active!==false;
  MOZER_HOST.safeBottom=Math.max(0,Number(m.safeBottom)||0);
  MOZER_HOST.background=m.background||null;
  if(m.responsive){try{MOZER_RESPONSIVE.set(m.responsive);}catch(e){console.warn('MOZER layout rejected:',e.message);}}
  window.dispatchEvent(new Event('mozer-layout'));
});
window.addEventListener('mozer-responsive-change',()=>window.dispatchEvent(new Event('mozer-layout')));
if(parent===window&&new URLSearchParams(location.search).get('layoutEdit')==='1'){
  window.addEventListener('DOMContentLoaded',()=>{const script=document.createElement('script');script.src=new URL('layout-editor.js?v=20261104-pixels1',MOZER_ASSET_ROOT);document.head.append(script);});
}
if(parent!==window)parent.postMessage({type:'mozer-v2:ready'},'*');
if(parent!==window){
  let reported='';function reportViewport(){const stage=document.getElementById('stage'),width=stage?.clientWidth||innerWidth,height=stage?.clientHeight||innerHeight,key=width+':'+height;if(key===reported)return;reported=key;parent.postMessage({type:'mozer-v2:viewport',width,height},'*');}
  window.addEventListener('resize',reportViewport,{passive:true});window.addEventListener('DOMContentLoaded',()=>{reportViewport();new ResizeObserver(reportViewport).observe(document.getElementById('stage'));});
  window.visualViewport?.addEventListener('resize',reportViewport,{passive:true});
}
