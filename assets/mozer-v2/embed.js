(() => {
  'use strict';
  const script=document.currentScript;
  const release='20261104-pixels1';
  const sceneURL=new URL('../../mozer-scene-v2.html',script.src);
  sceneURL.searchParams.set('v',release);
  const allowedOrigin=sceneURL.origin;
  // Older WebKit returned CSS-zoom-relative rectangles. Feature-test instead
  // of guessing a Safari version; leave all native Tilda transforms intact.
  function cssZoom(el){if(Number.isFinite(el.currentCSSZoom)&&el.currentCSSZoom>0)return el.currentCSSZoom;let value=1;for(let p=el;p;p=p.parentElement){const z=getComputedStyle(p).zoom,n=parseFloat(z);if(Number.isFinite(n)&&n>0)value*=z.endsWith('%')?n/100:n;}return value;}
  const probe=document.createElement('div');probe.style.cssText='position:fixed;left:0;top:0;width:20px;height:20px;zoom:2;visibility:hidden;pointer-events:none';document.body.append(probe);
  const expectedProbe=20*cssZoom(probe),legacyZoomRects=Math.abs(probe.getBoundingClientRect().width-expectedProbe)>1;probe.remove();
  let zoomCache=new WeakMap();
  function visualRect(el){const r=el.getBoundingClientRect();if(!legacyZoomRects)return r;let z=zoomCache.get(el);if(!z){z=cssZoom(el);zoomCache.set(el,z);}return {left:r.left*z,top:r.top*z,right:r.right*z,bottom:r.bottom*z,width:r.width*z,height:r.height*z};}
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
    const record=host.closest('.t-rec')||host.parentElement;
    const artboard=record.querySelector('.t396__artboard')||host.parentElement;
    const reference=host.closest('.tn-elem')||host.parentElement;
    Object.assign(host.style,{position:'absolute',left:'0px',top:'0px',width:'0px',height:'0px',overflow:'hidden',pointerEvents:'none'});
    // Render beside the scaled native HTML element, never inside its CSS zoom.
    // Only this custom viewport is positioned; Tilda's own styles stay intact.
    const viewport=document.createElement('div');viewport.className='mozer-scene-pixel-viewport';
    Object.assign(viewport.style,{position:'absolute',left:'0px',top:'0px',width:'0px',height:'0px',overflow:'hidden',pointerEvents:'none',background:'transparent'});
    artboard.append(viewport);
    let debug=null;if(new URLSearchParams(location.search).get('mozerDebug')==='1'){debug=document.createElement('pre');debug.style.cssText='position:fixed;bottom:8px;right:8px;z-index:2147483645;margin:0;padding:8px;color:#fff;background:#111d;pointer-events:none;font:11px/1.4 monospace;white-space:pre-wrap';document.body.append(debug);}
    const iframe=document.createElement('iframe');
    let pixelWidth=1,pixelHeight=1;
    function fitPixels(){
      const r=visualRect(reference),a=visualRect(artboard);if(!r.width||!r.height)return false;
      const zoom=cssZoom(artboard),sx=a.width/Math.max(1,artboard.clientWidth*zoom),sy=a.height/Math.max(1,artboard.clientHeight*zoom);
      const css=getComputedStyle(reference);let transformed=false;for(let p=reference;p&&p!==artboard;p=p.parentElement)if(getComputedStyle(p).transform&&getComputedStyle(p).transform!=='none')transformed=true;
      const cssWidth=parseFloat(css.width)*cssZoom(reference),cssHeight=parseFloat(css.height)*cssZoom(reference);
      const w=Math.max(1,Math.round(!transformed&&Number.isFinite(cssWidth)?cssWidth:r.width/Math.max(.01,sx))),h=Math.max(1,Math.round(!transformed&&Number.isFinite(cssHeight)?cssHeight:r.height/Math.max(.01,sy)));
      Object.assign(viewport.style,{left:(r.left-a.left)/Math.max(.01,sx)+'px',top:(r.top-a.top)/Math.max(.01,sy)+'px',width:w+'px',height:h+'px',zoom:String(1/zoom),zIndex:getComputedStyle(reference).zIndex});
      if(pixelWidth!==w||pixelHeight!==h){pixelWidth=w;pixelHeight=h;iframe.width=String(w);iframe.height=String(h);iframe.style.width=w+'px';iframe.style.height=h+'px';}
      return true;
    }
    function nativeScreen(){
      const width=parseFloat(getComputedStyle(reference).width),screens=[1200,960,640,480,320];
      const fallback=screens.find(s=>document.documentElement.clientWidth>=s)||320;
      const matches=screens.filter(screen=>{const key=screen===1200?'data-field-width-value':'data-field-width-res-'+screen+'-value';return Math.abs(Number(reference.getAttribute(key)||NaN)-width)<.75;});
      return matches.length===1?matches[0]:fallback;
    }
    iframe.src=sceneURL.href;iframe.title='MOZER GROUP — анимированная молекула';
    iframe.setAttribute('aria-hidden','true');iframe.tabIndex=-1;
    iframe.setAttribute('scrolling','no');
    iframe.loading='eager';iframe.referrerPolicy='strict-origin-when-cross-origin';
    Object.assign(iframe.style,{position:'absolute',left:'0px',top:'0px',width:'1px',height:'1px',border:'0',display:'block',background:'transparent',maxWidth:'none',maxHeight:'none'});
    // Native wheel/touch events must stay in the page's scrolling document.
    // Forward coordinates instead of intercepting or synthesizing wheel events.
    iframe.style.pointerEvents='none';
    fitPixels();viewport.append(iframe);
    let active=true,raf=0,scrollRaf=0,pointerRaf=0,lastPointer=null,pointerWasInside=false,backgroundURL='',backgroundImage=null,disposed=false,childViewport=null;
    host.dataset.geometryMode=legacyZoomRects?'legacy-css-zoom':'standard';
    function pointerMessage(e,kind){
      const r=visualRect(iframe),blocked=!!e.target?.closest?.('#mozer-layout-editor,a,button,input,select,textarea,[role="button"]');
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
      const r=visualRect(artboard),small=nativeScreen()<960;
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
        const r=visualRect(atom);
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
      if(!fitPixels())return;
      const r=visualRect(iframe);if(!r.width||!r.height)return;
      const screen=nativeScreen();if(window.MOZER_ACTIVE_SCREEN!==screen){window.MOZER_ACTIVE_SCREEN=screen;window.dispatchEvent(new Event('mozer-screen'));}
      const scaleX=r.width/Math.max(1,childViewport?.width||iframe.clientWidth),scaleY=r.height/Math.max(1,childViewport?.height||iframe.clientHeight),scale=Math.max(scaleX,scaleY);
      host.dataset.canvasViewport=JSON.stringify({displayWidth:r.width,displayHeight:r.height,childWidth:childViewport?.width||iframe.clientWidth,childHeight:childViewport?.height||iframe.clientHeight,scale});
      if(debug)debug.textContent='MOZER pixels1 · '+screen+'\nnative '+getComputedStyle(reference).width+' × '+getComputedStyle(reference).height+'\nnative zoom '+cssZoom(reference).toFixed(3)+'\niframe '+pixelWidth+' × '+pixelHeight+' px\nchild '+(childViewport?.width||'?')+' × '+(childViewport?.height||'?')+'\nDPR '+(devicePixelRatio||1)+'\nrect '+r.width.toFixed(1)+' × '+r.height.toFixed(1);
      let safeBottom=0;
      for(const id of ['1785737963199000001','1785737963199000002','1787816720642000004']){
        const el=record.querySelector('[data-elem-id="'+id+'"]');if(!el)continue;
        const b=visualRect(el);if(b.width&&b.height)safeBottom=Math.max(safeBottom,(b.bottom-r.top)/scaleY);
      }
      iframe.contentWindow.postMessage({type:'mozer-v2:layout',viewportWidth:document.documentElement.clientWidth,screen,pixelWidth,pixelHeight,scale,displayWidth:r.width,displayHeight:r.height,deviceDpr:devicePixelRatio||1,active:active&&!document.hidden,safeBottom,background:background(r),responsive:editing&&window.MOZER_RESPONSIVE?MOZER_RESPONSIVE.get():null},allowedOrigin);
    }
    function schedule(){if(!raf&&!disposed)raf=requestAnimationFrame(send);}
    function resized(){zoomCache=new WeakMap();schedule();scrolling();setTimeout(schedule,120);setTimeout(schedule,500);}
    const ro=new ResizeObserver(schedule);ro.observe(artboard);ro.observe(reference);
    const mo=new MutationObserver(resized);mo.observe(reference,{attributes:true,attributeFilter:['style','class']});mo.observe(artboard,{attributes:true,attributeFilter:['style','class']});
    record.querySelectorAll('[data-elem-type="text"]').forEach(el=>ro.observe(el));
    const io=new IntersectionObserver(entries=>{active=entries[0].isIntersecting;schedule();},{threshold:0.01});io.observe(artboard);
    const onMessage=e=>{if(e.source!==iframe.contentWindow||e.origin!==allowedOrigin)return;if(e.data?.type==='mozer-v2:ready')resized();if(e.data?.type==='mozer-v2:viewport'&&Number.isFinite(e.data.width)&&Number.isFinite(e.data.height)&&e.data.width>0&&e.data.height>0){childViewport={width:e.data.width,height:e.data.height};resized();}};
    iframe.addEventListener('load',resized);window.addEventListener('message',onMessage);
    window.addEventListener('mozer-responsive-change',schedule);responsiveReady.then(schedule,()=>{});
    window.addEventListener('resize',resized,{passive:true});document.addEventListener('visibilitychange',schedule);
    window.visualViewport?.addEventListener('resize',resized,{passive:true});
    window.addEventListener('scroll',scrolling,{passive:true});
    window.addEventListener('pointermove',pointerMove,{passive:true});window.addEventListener('pointerdown',pointerDown,{passive:true});window.addEventListener('blur',pointerLeave);document.documentElement.addEventListener('pointerleave',pointerLeave);
    document.fonts?.ready.then(resized);resized();
    window.addEventListener('pagehide',e=>{if(e.persisted)return;disposed=true;cancelAnimationFrame(raf);cancelAnimationFrame(scrollRaf);cancelAnimationFrame(pointerRaf);ro.disconnect();mo.disconnect();io.disconnect();viewport.remove();debug?.remove();window.removeEventListener('pointermove',pointerMove);window.removeEventListener('pointerdown',pointerDown);window.removeEventListener('blur',pointerLeave);document.documentElement.removeEventListener('pointerleave',pointerLeave);window.removeEventListener('message',onMessage);window.removeEventListener('resize',resized);window.visualViewport?.removeEventListener('resize',resized);window.removeEventListener('scroll',scrolling);window.removeEventListener('mozer-responsive-change',schedule);document.removeEventListener('visibilitychange',schedule);},{once:true});
  });
})();
