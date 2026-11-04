(() => {
  'use strict';
  const script=document.currentScript;
  const release='20261104-editor2';
  const sceneURL=new URL('../../mozer-scene-v2.html',script.src);
  sceneURL.searchParams.set('v',release);
  const allowedOrigin=sceneURL.origin;
  const editing=new URLSearchParams(location.search).get('mozerEdit')==='1'||!!document.querySelector('[data-mozer-scene="v2"][data-mozer-editor="true"]');
  let responsiveReady=Promise.resolve();
  if(editing){
    const load=name=>new Promise((resolve,reject)=>{const s=document.createElement('script'),url=new URL(name+'.js',script.src);url.searchParams.set('v',release);s.src=url;s.onload=resolve;s.onerror=reject;document.head.append(s);});
    responsiveReady=(window.MOZER_RESPONSIVE?Promise.resolve():load('layout-defaults').then(()=>load('layout'))).then(()=>load('layout-editor'));
    responsiveReady.catch(()=>console.error('MOZER: не удалось загрузить панель адаптива'));
  }
  const hosts=document.querySelectorAll('[data-mozer-scene="v2"]:not([data-mozer-mounted])');
  hosts.forEach(host=>{
    host.dataset.mozerMounted='true';
    Object.assign(host.style,{position:'absolute',inset:'0',width:'100%',height:'100%',overflow:'hidden'});
    const record=host.closest('.t-rec')||host.parentElement;
    const artboard=record.querySelector('.t396__artboard')||host.parentElement;
    const iframe=document.createElement('iframe');
    iframe.src=sceneURL.href;iframe.title='MOZER GROUP — анимированная молекула';
    iframe.setAttribute('aria-hidden','true');iframe.tabIndex=-1;
    iframe.loading='eager';iframe.referrerPolicy='strict-origin-when-cross-origin';
    Object.assign(iframe.style,{position:'absolute',inset:'0',width:'100%',height:'100%',border:'0',display:'block',background:'transparent'});
    // Native wheel/touch events must stay in the page's scrolling document.
    // Forward coordinates instead of intercepting or synthesizing wheel events.
    iframe.style.pointerEvents='none';
    host.append(iframe);
    let active=true,raf=0,scrollRaf=0,pointerRaf=0,lastPointer=null,pointerWasInside=false,backgroundURL='',backgroundImage=null,disposed=false;
    function pointerMessage(e,kind){
      const r=iframe.getBoundingClientRect(),blocked=!!e.target?.closest?.('#mozer-layout-editor,a,button,input,select,textarea,[role="button"]');
      const inside=active&&!blocked&&r.width>0&&r.height>0&&e.clientX>=r.left&&e.clientX<=r.right&&e.clientY>=r.top&&e.clientY<=r.bottom;
      if(!inside&&!pointerWasInside)return;
      pointerWasInside=inside;
      iframe.contentWindow?.postMessage({type:'mozer-v2:pointer',kind,inside,x:(e.clientX-r.left)/r.width,y:(e.clientY-r.top)/r.height},allowedOrigin);
    }
    function flushPointer(){pointerRaf=0;if(!disposed&&lastPointer)pointerMessage(lastPointer,'move');}
    function pointerMove(e){if(e.pointerType==='touch')return;lastPointer={clientX:e.clientX,clientY:e.clientY,target:e.target};if(!pointerRaf)pointerRaf=requestAnimationFrame(flushPointer);}
    function pointerDown(e){if(e.pointerType!=='touch')pointerMessage(e,'down');}
    function pointerLeave(){lastPointer=null;if(pointerWasInside){pointerWasInside=false;iframe.contentWindow?.postMessage({type:'mozer-v2:pointer',inside:false},allowedOrigin);}}
    function sendScroll(){
      scrollRaf=0;if(disposed||!iframe.contentWindow)return;
      const r=artboard.getBoundingClientRect(),small=document.documentElement.clientWidth<960;
      const progress=small?Math.max(-1,Math.min(1,-r.top/Math.max(1,Math.min(r.height,innerHeight)))):0;
      iframe.contentWindow.postMessage({type:'mozer-v2:scroll',progress},allowedOrigin);
    }
    function scrolling(){if(!scrollRaf&&!disposed)scrollRaf=requestAnimationFrame(sendScroll);if(lastPointer&&!pointerRaf)pointerRaf=requestAnimationFrame(flushPointer);}
    const positionPart=(v,axis)=>{if(v==='left'||v==='top')return 0;if(v==='right'||v==='bottom')return 1;const n=parseFloat(v);return Number.isFinite(n)&&v.includes('%')?n/100:.5;};
    function background(frame){
      const marked=record.querySelector('.mozer-scene-bg .tn-atom, .mozer-scene-bg.tn-atom');
      const candidates=marked?[marked]:Array.from(artboard.querySelectorAll('.tn-elem[data-elem-type="shape"] .tn-atom'));
      for(const atom of candidates){
        const css=getComputedStyle(atom),match=css.backgroundImage.match(/url\(["']?(.*?)["']?\)/);
        if(!match)continue;
        const r=atom.getBoundingClientRect();
        if(!marked&&(r.width<frame.width*.8||r.height<frame.height*.8))continue;
        const url=new URL(match[1],location.href).href;if(!/^https?:/.test(url))continue;
        if(backgroundURL!==url){backgroundURL=url;backgroundImage=new Image();backgroundImage.onload=schedule;backgroundImage.src=url;}
        const pos=css.backgroundPosition.split(/\s+/);
        return {url,width:r.width,height:r.height,x:r.left-frame.left,y:r.top-frame.top,position:[positionPart(pos[0]),positionPart(pos[1])],size:css.backgroundSize};
      }
      return null;
    }
    function send(){
      raf=0;if(disposed||!iframe.contentWindow)return;
      const r=iframe.getBoundingClientRect();if(!r.width||!r.height)return;
      const scale=r.width/Math.max(1,iframe.clientWidth);
      let safeBottom=0;
      for(const id of ['1785737963199000001','1785737963199000002','1787816720642000004']){
        const el=record.querySelector('[data-elem-id="'+id+'"]');if(!el)continue;
        const b=el.getBoundingClientRect();if(b.width&&b.height)safeBottom=Math.max(safeBottom,(b.bottom-r.top)/scale);
      }
      iframe.contentWindow.postMessage({type:'mozer-v2:layout',viewportWidth:document.documentElement.clientWidth,scale,active:active&&!document.hidden,safeBottom,background:background(r),responsive:editing&&window.MOZER_RESPONSIVE?MOZER_RESPONSIVE.get():null},allowedOrigin);
    }
    function schedule(){if(!raf&&!disposed)raf=requestAnimationFrame(send);}
    function resized(){schedule();scrolling();setTimeout(schedule,120);setTimeout(schedule,500);}
    const ro=new ResizeObserver(schedule);ro.observe(artboard);ro.observe(host);
    record.querySelectorAll('[data-elem-type="text"]').forEach(el=>ro.observe(el));
    const io=new IntersectionObserver(entries=>{active=entries[0].isIntersecting;schedule();},{threshold:0.01});io.observe(artboard);
    const onMessage=e=>{if(e.source===iframe.contentWindow&&e.origin===allowedOrigin&&e.data?.type==='mozer-v2:ready')resized();};
    iframe.addEventListener('load',resized);window.addEventListener('message',onMessage);
    window.addEventListener('mozer-responsive-change',schedule);responsiveReady.then(schedule,()=>{});
    window.addEventListener('resize',resized,{passive:true});document.addEventListener('visibilitychange',schedule);
    window.addEventListener('scroll',scrolling,{passive:true});
    window.addEventListener('pointermove',pointerMove,{passive:true});window.addEventListener('pointerdown',pointerDown,{passive:true});window.addEventListener('blur',pointerLeave);document.documentElement.addEventListener('pointerleave',pointerLeave);
    document.fonts?.ready.then(resized);resized();
    window.addEventListener('pagehide',e=>{if(e.persisted)return;disposed=true;cancelAnimationFrame(raf);cancelAnimationFrame(scrollRaf);cancelAnimationFrame(pointerRaf);ro.disconnect();io.disconnect();window.removeEventListener('pointermove',pointerMove);window.removeEventListener('pointerdown',pointerDown);window.removeEventListener('blur',pointerLeave);document.documentElement.removeEventListener('pointerleave',pointerLeave);window.removeEventListener('message',onMessage);window.removeEventListener('resize',resized);window.removeEventListener('scroll',scrolling);window.removeEventListener('mozer-responsive-change',schedule);document.removeEventListener('visibilitychange',schedule);},{once:true});
  });
})();
