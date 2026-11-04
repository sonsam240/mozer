(() => {
  'use strict';
  const screens=[1200,960,640,480,320], clone=v=>JSON.parse(JSON.stringify(v));
  let config=clone(window.MOZER_LAYOUT_DEFAULTS);
  const limits={x:[-100,200],y:[-100,200],scale:[.05,5],coreSize:[.2,3],rotationX:[-180,180],rotationY:[-180,180],rotationZ:[-180,180],scrollTiltX:[-20,20],scrollTiltY:[-20,20],scrollTiltZ:[-20,20]};
  function valid(raw){
    if(raw?.schema!=='mozer-responsive-v1'||!raw.breakpoints)throw Error('Нужен JSON mozer-responsive-v1');
    const result=clone(window.MOZER_LAYOUT_DEFAULTS);
    const materials=raw.sceneSettings||result.sceneSettings;
    const specifications=new Map((window.MOZER_MATERIAL_CONTROLS||[]).flatMap(x=>x[1]).map(row=>[row[0],row]));
    for(const [key,initial] of Object.entries(result.sceneSettings)){
      const value=materials[key]??initial,spec=specifications.get(key);
      if(typeof initial==='string'){if(typeof value!=='string'||!/^#[0-9a-fA-F]{6}$/.test(value))throw Error('Неверный HEX: '+key);}
      else if(typeof value!=='number'||!Number.isFinite(value)||Math.abs(value)>10000||(spec?.length===5&&(value<spec[2]||value>spec[3])))throw Error('Недопустимое значение: '+key);
      result.sceneSettings[key]=value;
    }
    for(const screen of screens){
      const entry=raw.breakpoints[screen];if(!entry)throw Error('Нет экрана '+screen);
      for(const group of ['molecule','wave']){
        if(!entry[group])throw Error('Нет группы '+group+' / '+screen);
        for(const key of Object.keys(result.breakpoints[screen][group])){
          if(key==='sphereOffsets'){
            const a=entry[group][key]??[[0,0],[0,0],[0,0],[0,0]];
            if(!Array.isArray(a)||a.length!==4||a.some(v=>!Array.isArray(v)||v.length!==2||v.some(n=>typeof n!=='number'||!Number.isFinite(n)||n< -100||n>100)))throw Error('Смещения шаров: −100…100');
            result.breakpoints[screen][group][key]=a.map(p=>p.slice());continue;
          }
          if(key==='sphereSizes'){
            const a=entry[group][key];if(!Array.isArray(a)||a.length!==4||a.some(v=>typeof v!=='number'||!Number.isFinite(v)||v<.2||v>3))throw Error('Размеры шаров: 0.2–3');
            result.breakpoints[screen][group][key]=a.slice();continue;
          }
          const value=entry[group][key]??(key.startsWith('scrollTilt')?result.breakpoints[screen][group][key]:undefined),range=limits[key];
          if(typeof value!=='number'||!Number.isFinite(value)||value<range[0]||value>range[1])throw Error(screen+' / '+key+': '+range.join('…'));
          result.breakpoints[screen][group][key]=value;
        }
      }
    }
    return result;
  }
  window.MOZER_RESPONSIVE={
    screens,limits,screen:w=>screens.find(n=>w>=n)||320,
    get:()=>clone(config),defaults:()=>clone(window.MOZER_LAYOUT_DEFAULTS),
    at:screen=>config.breakpoints[screen],
    set(raw){config=valid(raw);window.dispatchEvent(new Event('mozer-responsive-change'));},
    current(width){return config.breakpoints[this.screen(width)];}
  };
})();
