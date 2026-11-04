(() => {
  'use strict';
  window.MOZER_RENDER_QUALITY={
    plan({width,height,scale=1,dpr=1,profile,renderScale=1.5,autoQuality=true}){
      const hostScale=Math.max(.25,Math.min(4,scale)),deviceDpr=Math.max(1,dpr||1);
      const dimensionLimit=Math.min(4096/Math.max(1,width),4096/Math.max(1,height));
      const target=profile.mobile?hostScale*Math.min(deviceDpr,profile.maxDpr):Math.min(renderScale,profile.maxDpr,Math.max(1,hostScale)*deviceDpr);
      const maximum=Math.max(.125,Math.min(target,dimensionLimit));
      const floor=profile.mobile?hostScale*Math.min(deviceDpr,profile.minDpr):1;
      const minimum=Math.min(1,floor/maximum);
      const budget=Math.sqrt(profile.pixelBudget/Math.max(1,width*height))/maximum;
      const initial=autoQuality?Math.max(minimum,Math.min(1,budget)):1;
      return {maximum,minimum,initial,hostScale,floor,mobile:profile.mobile};
    },
    sizes({width,height,plan,mainFactor=1,rearFactor=1}){
      const ratio=plan.maximum*Math.max(plan.minimum,Math.min(1,mainFactor));
      const rearRatio=plan.mobile?Math.min(ratio,Math.max(plan.floor,ratio*rearFactor)):Math.min(ratio,rearFactor);
      return {ratio,rearRatio,width:Math.max(1,Math.round(width*ratio)),height:Math.max(1,Math.round(height*ratio)),rearWidth:Math.max(1,Math.round(width*rearRatio)),rearHeight:Math.max(1,Math.round(height*rearRatio)),visibleDpr:ratio/plan.hostScale};
    }
  };
})();
