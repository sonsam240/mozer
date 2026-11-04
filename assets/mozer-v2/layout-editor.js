(() => {
  'use strict';
  if(document.getElementById('mozer-layout-editor'))return;
  const api=window.MOZER_RESPONSIVE, storageKey='mozer-responsive-v1-draft-20261104-hidpi-settings2', root=document.createElement('aside');
  root.id='mozer-layout-editor';root.setAttribute('aria-label','Адаптив MOZER');
  root.innerHTML=`<style>
  #mozer-layout-editor{position:fixed;z-index:2147483646;top:12px;left:12px;width:290px;max-width:calc(100vw - 24px);color:#28251d;background:#fffdf8f5;border:1px solid #d6cebc;border-radius:14px;box-shadow:0 8px 36px #0002;font:12px/1.4 Arial,sans-serif;box-sizing:border-box;text-align:left;}
  #mozer-layout-editor *{box-sizing:border-box}#mozer-layout-editor header{display:flex;justify-content:space-between;align-items:center;padding:12px;gap:8px}#mozer-layout-editor button,#mozer-layout-editor select,#mozer-layout-editor input,#mozer-layout-editor textarea{font:inherit;color:inherit;border:1px solid #d6cebc;border-radius:5px;background:#fff;padding:6px;min-width:0}#mozer-layout-editor button{cursor:pointer}#mozer-layout-editor .body{padding:0 12px 12px;max-height:calc(100dvh - 86px);overflow:auto;overscroll-behavior:contain}#mozer-layout-editor label{display:flex;justify-content:space-between;align-items:center;gap:6px;margin:7px 0}#mozer-layout-editor input[type=number]{width:83px}#mozer-layout-editor input[type=range]{width:100%;accent-color:#ac8b37;padding:0}#mozer-layout-editor fieldset{border:0;border-top:1px solid #dfd9cd;padding:10px 0 2px;margin:12px 0 0}#mozer-layout-editor legend{font-weight:bold;padding:0 4px}#mozer-layout-editor .row{display:flex;gap:6px;flex-wrap:wrap;margin:8px 0}#mozer-layout-editor .note{color:#777061;font-size:11px}#mozer-layout-editor textarea{width:100%;height:110px;resize:vertical}#mozer-layout-editor [hidden]{display:none!important}
  </style><header><strong>MOZER · Адаптив</strong><button type="button" data-action="collapse">Свернуть</button></header><div class="body"><div class="note" data-info></div><label>Настраивать экран<select data-screen><option value="auto">Текущий автоматически</option>${api.screens.map(s=>`<option value="${s}">${s} px</option>`).join('')}</select></label><p class="note">X — от левого края, Y — от верхнего, в % холста. Масштаб равномерный. Выбор другого экрана не меняет ширину страницы.</p><div data-fields></div><div class="row"><button data-action="copy">Копировать JSON</button><button data-action="download">Скачать JSON</button></div><details><summary>Импорт / просмотр JSON</summary><textarea data-json aria-label="JSON настроек адаптива" spellcheck="false"></textarea><div class="row"><button data-action="import">Применить JSON</button></div></details><div class="row"><button data-action="reset">Сбросить этот экран</button><button data-action="erase">Забыть локальный черновик</button></div><div class="note" role="status" data-status>Черновик сохраняется в этом браузере. Посетители не увидят панель без режима настройки.</div></div>`;
  document.body.append(root);
  const compactStyle=document.createElement('style');compactStyle.textContent=`
  #mozer-layout-editor{display:flex;flex-direction:column;max-height:min(var(--panel-height,300px),calc(100dvh - 24px))}
  #mozer-layout-editor header,#mozer-layout-editor .tools{flex-shrink:0}
  #mozer-layout-editor .body{min-height:0;max-height:none;overflow:auto}
  #mozer-layout-editor .tools{padding:0 12px 6px;display:flex;align-items:center;gap:8px}
  #mozer-layout-editor .tools label{margin:0}#mozer-layout-editor .tools input{width:60px}
  #mozer-layout-editor .export{padding:0 12px;flex-shrink:0}
  @media(pointer:coarse){#mozer-layout-editor input[type=number],#mozer-layout-editor input[type=text],#mozer-layout-editor select,#mozer-layout-editor textarea{font-size:16px}}
  `;root.append(compactStyle);
  const tools=document.createElement('div');tools.className='tools';tools.innerHTML='<label>Высота <input type="number" aria-label="Высота панели, px" min="180" max="900" step="10" value="300"></label><button type="button" data-action="corner">Перенести</button>';
  root.insertBefore(tools,root.querySelector('.body'));
  const modeRow=document.createElement('label');modeRow.innerHTML='Раздел <select aria-label="Раздел настроек"><option value="layout">Адаптив</option>'+window.MOZER_MATERIAL_CONTROLS.map((g,i)=>'<option value="'+i+'">'+g[0]+'</option>').join('')+'</select>';
  modeRow.style.cssText='margin:0 12px 6px;flex-shrink:0';root.insertBefore(modeRow,root.querySelector('.body'));
  const modeSelect=modeRow.querySelector('select');
  const exportRow=root.querySelector('.row');exportRow.classList.add('export');root.insertBefore(exportRow,root.querySelector('.body'));
  tools.querySelector('input').addEventListener('input',e=>{const n=Number(e.target.value);if(n>=180&&n<=900)root.style.setProperty('--panel-height',n+'px');});
  let corner=0;
  const q=s=>root.querySelector(s), fields=q('[data-fields]'),select=q('[data-screen]'),status=q('[data-status]');
  const definitions=[['molecule','Молекула',[['x','Положение X, %',-100,200,.1],['y','Положение Y, %',-100,200,.1],['scale','Общий масштаб',.05,5,.01],['coreSize','Центральный шар',.2,3,.01],['sphereSizes.0','Шар слева сверху',.2,3,.01],['sphereSizes.1','Шар справа сверху',.2,3,.01],['sphereSizes.2','Шар слева снизу',.2,3,.01],['sphereSizes.3','Шар справа снизу',.2,3,.01]]],['wave','Фоновый поток',[['x','Положение X, %',-100,200,.1],['y','Положение Y, %',-100,200,.1],['scale','Общий масштаб',.05,5,.01],['rotationX','Наклон X, °',-180,180,1],['rotationY','Наклон Y, °',-180,180,1],['rotationZ','Поворот Z, °',-180,180,1]]]];
  for(let i=0;i<4;i++){const title=['Слева сверху','Справа сверху','Слева снизу','Справа снизу'][i];definitions[0][2].push(['sphereOffsets.'+i+'.0',title+' · сдвиг X',-100,100,1],['sphereOffsets.'+i+'.1',title+' · сдвиг Y',-100,100,1]);}
  definitions[0][2].push(['scrollTiltX','Скролл · наклон X, °',-20,20,.5],['scrollTiltY','Скролл · разворот Y, °',-20,20,.5],['scrollTiltZ','Скролл · поворот Z, °',-20,20,.5]);
  const offsetNote=document.createElement('p');offsetNote.className='note';offsetNote.textContent='Сдвиги шаров — % от условной ширины молекулы: X вправо, Y вниз. Они сохраняют связи и масштабируются вместе с ней.';root.querySelector('[data-fields]').after(offsetNote);
  const viewport=()=>document.documentElement.clientWidth,activeScreen=()=>api.screens.includes(window.MOZER_ACTIVE_SCREEN)?window.MOZER_ACTIVE_SCREEN:api.screen(viewport()), current=()=>select.value==='auto'?activeScreen():Number(select.value);
  const get=(obj,path)=>path.split('.').reduce((a,k)=>a[k],obj);
  function change(group,path,value){
    const config=api.get(),parts=path.split('.');let target=config.breakpoints[current()][group];for(const p of parts.slice(0,-1))target=target[p];target[parts.at(-1)]=value;
    try{api.set(config);try{localStorage.setItem(storageKey,JSON.stringify(config));}catch{}status.textContent='Применено · экран '+current()+' px';q('[data-json]').value=JSON.stringify(api.get(),null,2);}catch(e){status.textContent=e.message;}
  }
  function materialChange(key,value){
    const config=api.get();config.sceneSettings[key]=value;
    try{api.set(config);try{localStorage.setItem(storageKey,JSON.stringify(config));}catch{}status.textContent='Применено ко всем разрешениям';q('[data-json]').value=JSON.stringify(api.get(),null,2);}catch(e){status.textContent=e.message;}
  }
  function renderMaterials(){
    const group=window.MOZER_MATERIAL_CONTROLS[Number(modeSelect.value)],values=api.get().sceneSettings,set=document.createElement('fieldset'),legend=document.createElement('legend');legend.textContent=group[0]+' · общие';set.append(legend);
    for(const [key,title,min,max,step] of group[1]){
      const label=document.createElement('label'),name=document.createElement('span'),input=document.createElement('input');name.textContent=title;
      input.setAttribute('aria-label',title);input.value=values[key];
      if(min===undefined){
        input.type='color';input.value=values[key];const hex=document.createElement('input');hex.type='text';hex.value=values[key];hex.maxLength=7;hex.setAttribute('aria-label',title+' HEX');hex.style.width='90px';
        input.addEventListener('input',()=>{hex.value=input.value;materialChange(key,input.value);});hex.addEventListener('input',()=>{if(/^#[0-9a-fA-F]{6}$/.test(hex.value)){input.value=hex.value;materialChange(key,hex.value);}});
        label.append(name,input);set.append(label,hex);
      }else{
        input.type='number';Object.assign(input,{min,max,step,value:values[key]});const range=document.createElement('input');range.type='range';Object.assign(range,{min,max,step,value:values[key]});range.setAttribute('aria-label',title+' ползунок');
        input.addEventListener('input',()=>{if(input.value!==''&&input.validity.valid){range.value=input.value;materialChange(key,Number(input.value));}});range.addEventListener('input',()=>{input.value=range.value;materialChange(key,Number(range.value));});label.append(name,input);set.append(label,range);
      }
    }fields.append(set);
  }
  function render(){
    const active=activeScreen(),screen=current(),entry=api.get().breakpoints[screen];q('[data-info]').textContent=`Окно ${viewport()} px · активный экран ${active} px${screen!==active?' · редактируете неактивный экран':''}`;
    fields.replaceChildren();
    select.closest('label').hidden=modeSelect.value!=='layout';
    if(modeSelect.value!=='layout'){renderMaterials();q('[data-json]').value=JSON.stringify(api.get(),null,2);return;}
    for(const [group,title,defs] of definitions){
      const set=document.createElement('fieldset'),legend=document.createElement('legend');legend.textContent=title;set.append(legend);
      for(const [path,title,min,max,step] of defs){
        const label=document.createElement('label'),name=document.createElement('span'),number=document.createElement('input'),range=document.createElement('input');name.textContent=title;number.type='number';range.type='range';number.setAttribute('aria-label',title);range.setAttribute('aria-label',title+' ползунок');
        for(const input of [number,range]){Object.assign(input,{min,max,step,value:get(entry[group],path)});}
        range.addEventListener('input',()=>{number.value=range.value;change(group,path,Number(range.value));});
        number.addEventListener('input',()=>{if(number.value===''||!number.validity.valid)return;range.value=number.value;change(group,path,Number(number.value));});
        label.append(name,number);set.append(label,range);
      }fields.append(set);
    }q('[data-json]').value=JSON.stringify(api.get(),null,2);
  }
  root.addEventListener('click',async e=>{
    const action=e.target.dataset.action;if(!action)return;
    if(action==='corner'){corner=(corner+1)%4;Object.assign(root.style,{left:corner===0||corner===3?'12px':'auto',right:corner===1||corner===2?'12px':'auto',top:corner<2?'12px':'auto',bottom:corner>=2?'12px':'auto'});return;}
    if(action==='collapse'){q('.body').hidden=!q('.body').hidden;tools.hidden=exportRow.hidden=modeRow.hidden=q('.body').hidden;e.target.textContent=q('.body').hidden?'Развернуть':'Свернуть';return;}
    const text=JSON.stringify(api.get(),null,2);
    if(action==='copy'){try{await navigator.clipboard.writeText(text);status.textContent='JSON скопирован';}catch{q('details').open=true;q('[data-json]').focus();q('[data-json]').select();status.textContent='Выделен JSON: скопируйте Ctrl+C';}}
    if(action==='download'){const url=URL.createObjectURL(new Blob([text],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='mozer-responsive.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
    if(action==='import'){try{const raw=JSON.parse(q('[data-json]').value);api.set(raw);try{localStorage.setItem(storageKey,JSON.stringify(api.get()));}catch{}render();status.textContent='JSON применён';}catch(err){status.textContent='Ошибка: '+err.message;}}
    if(action==='reset'){const raw=api.get();if(modeSelect.value==='layout')raw.breakpoints[current()]=api.defaults().breakpoints[current()];else for(const [key] of window.MOZER_MATERIAL_CONTROLS[Number(modeSelect.value)][1])raw.sceneSettings[key]=api.defaults().sceneSettings[key];api.set(raw);try{localStorage.setItem(storageKey,JSON.stringify(raw));}catch{}render();}
    if(action==='erase'){try{localStorage.removeItem(storageKey);}catch{}api.set(api.defaults());render();status.textContent='Черновик удалён, исходные значения восстановлены';}
  });
  select.addEventListener('change',render);window.addEventListener('resize',render,{passive:true});
  window.addEventListener('mozer-screen',render);
  modeSelect.addEventListener('change',render);
  try{const saved=localStorage.getItem(storageKey);if(saved)api.set(JSON.parse(saved));}catch{}
  render();
})();
