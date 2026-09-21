'use strict';
const $=id=>document.getElementById(id);
const fragment=new URLSearchParams(location.hash.slice(1));
const token=fragment.get('token')||sessionStorage.getItem('xt-token')||'';
if(token)sessionStorage.setItem('xt-token',token);
history.replaceState(null,'',location.pathname);
const client=Array.from(crypto.getRandomValues(new Uint8Array(24)),x=>x.toString(16).padStart(2,'0')).join(''), keys=new Set();
let arming=false, sendQueued=false;
let state=null, tick=0, seq=0, active=false, pending=false, generation=0, recording=false, pollAt=0, cameraURL=null, armSeq=Infinity, settingsDirty=false, cameraFreshAt=0, lidarFreshAt=0, boot=null;
function notice(text){$('notice').textContent=text;}
async function api(path,data){const ctrl=new AbortController(),timer=setTimeout(()=>ctrl.abort(),1000);try{const res=await fetch(path,{method:data?'POST':'GET',headers:{'X-Control-Token':token,...(data?{'Content-Type':'application/json'}:{})},body:data?JSON.stringify(data):undefined,signal:ctrl.signal,cache:'no-store'});const body=await res.json();if(res.status===403)$('access-form').hidden=false;if(!res.ok)throw Error(res.status===403?'访问令牌失效，请重新打开服务入口':body.error||res.status);return body;}finally{clearTimeout(timer);}}
function paintKeys(){document.querySelectorAll('[data-key]').forEach(b=>b.classList.toggle('pressed',keys.has(b.dataset.key)));}
function stop(reason='已停止并锁定'){active=false;generation++;sendQueued=false;keys.clear();physicalKeys.clear();paintKeys();notice(reason);api('/api/control',{op:'stop'}).catch(()=>{});}
async function send(force=false){if(!active)return;if(pending){if(force)sendQueued=true;return;}if(document.hidden||!document.hasFocus()||performance.now()-pollAt>250){stop('页面或连接失去活性，已请求停止');return;}pending=true;const own=generation;try{await api('/api/control',{op:'drive',client,boot,seq:++seq,tick,keys:[...keys]});}catch(e){if(own===generation)stop(e.message);}finally{pending=false;if(sendQueued){sendQueued=false;send();}}}
$('stop').onclick=()=>stop();
$('access-form').onsubmit=e=>{e.preventDefault();const value=$('access-token').value.trim();if(!/^[A-Za-z0-9_-]{8,128}$/.test(value)){notice('请输入8–128位访问码（字母、数字、- 或 _）');return;}location.hash='token='+value;location.reload();};
$('access-form').hidden=!!token;
$('arm').onclick=async()=>{if(arming)return;if(active){stop();return;}keys.clear();physicalKeys.clear();paintKeys();arming=true;const attempt=++generation;notice('正在解锁…');try{if(!state?.healthy)throw Error('相机、雷达或控制连接未就绪');if(state.owner)throw Error('已有页面占用控制，请先按停止并锁定，再解锁');const ack=await api('/api/control',{op:'arm',client,boot,seq:++seq,tick});if(attempt!==generation){stop('解锁已取消，请重新点击');return;}armSeq=ack.bridge_seq;active=true;generation++;notice('控制已解锁 · 按住方向键输出，松开回中');send();}catch(e){notice(e.message);}finally{arming=false;}};
const mappings={ArrowUp:'up',ArrowDown:'down',ArrowLeft:'left',ArrowRight:'right',w:'up',s:'down',a:'left',d:'right'};
const physicalKeys=new Map();
window.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();physicalKeys.clear();stop();return;}if(['INPUT','TEXTAREA'].includes(e.target.tagName))return;const key=mappings[e.key]||mappings[e.key.toLowerCase()];if(!key)return;e.preventDefault();if(!active||e.repeat)return;if(key==='down'&&!state?.reverse_enabled){notice('倒车尚未标定，当前禁用');return;}physicalKeys.set(e.code,key);keys.add(key);paintKeys();send(true);});
window.addEventListener('keyup',e=>{const key=physicalKeys.get(e.code);physicalKeys.delete(e.code);if(!key)return;e.preventDefault();if(![...physicalKeys.values()].includes(key))keys.delete(key);paintKeys();send(true);});
window.addEventListener('blur',()=>{physicalKeys.clear();if(active||arming)stop('窗口失焦，已请求停止并锁定');});
document.addEventListener('visibilitychange',()=>{if(document.hidden){physicalKeys.clear();if(active||arming)stop('页面隐藏，已请求停止并锁定');}});
window.addEventListener('pagehide',()=>{if(!active&&!arming)return;active=false;fetch('/api/control',{method:'POST',headers:{'X-Control-Token':token,'Content-Type':'application/json'},body:JSON.stringify({op:'stop'}),keepalive:true}).catch(()=>{});});
document.querySelectorAll('[data-key]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();if(!active)return;if(b.dataset.key==='down'&&!state?.reverse_enabled)return;b.setPointerCapture(e.pointerId);keys.add(b.dataset.key);paintKeys();send();};const release=()=>{keys.delete(b.dataset.key);paintKeys();send(true);};b.onpointerup=release;b.onpointercancel=()=>{if(active||arming)stop('触控中断');};b.onlostpointercapture=release;});
$('settings').addEventListener('input',()=>{settingsDirty=true;});
$('settings').onsubmit=async e=>{e.preventDefault();try{const data=Object.fromEntries(['forward','reverse','left','right'].map(k=>[k,Number($(k).value)]));await api('/api/settings',data);settingsDirty=false;notice('PWM参数已应用');}catch(e){notice(e.message);}};
$('snapshot').onclick=async()=>{try{const x=await api('/api/snapshot',{});notice('快照已保存：'+x.file);}catch(e){notice(e.message);}};
$('record').onclick=async()=>{try{const x=await api(recording?'/api/record/stop':'/api/record/start',{});notice(x.file?'录制已打包：'+x.file:'开始录制');}catch(e){notice(e.message);}};
const reasons={browser_timeout:'浏览器心跳超时',sensor_stale:'传感器数据过期',bridge_heartbeat_timeout:'底盘心跳超时',bridge_control_gap:'底盘控制循环超时',bridge_stale_request:'控制请求延迟或乱序',bridge_status_backpressure:'底盘状态通道阻塞',heartbeat_timeout:'心跳超时',control_gap:'控制循环中断',stale_request:'旧指令已拒绝',operator_stop:'已停止',locked:'已锁定',not_armed_or_invalid_request:'需要重新解锁',status_backpressure:'状态通道阻塞'};
async function poll(){try{const s=await api('/api/state');state=s;boot=s.boot;tick=s.status.control?.tick||0;pollAt=performance.now();const c=s.status.control||{};if(active&&!c.armed&&c.seq>=armSeq){const reason=s.last_stop?.reason||c.reason;stop((reasons[reason]||reason||'控制已锁定')+'；请重新解锁');}
if(s.healthy&&$('notice').textContent==='等待画面与雷达就绪，默认锁定。')notice('实时画面已连接，控制保持锁定。');$('connection').textContent=s.healthy?'● 传感器在线':'● 检查连接';$('connection').classList.toggle('good',s.healthy);$('armed').textContent=c.armed?'已解锁':'已锁定';$('arm').textContent=active?'锁定控制':'解锁键盘控制';$('arm').disabled=false;$('arm').title=!s.healthy?'传感器或控制连接未就绪':s.owner&&s.owner!==client?'其他页面占用控制，可先停止再解锁':'';$('motor').innerHTML=(c.motor||1500)+'<span>µs</span>';$('servo').textContent=c.servo||1500;$('camera-age').textContent=s.ages.camera<5?Math.round(s.ages.camera*1000)+' ms':'无新画面';$('mode').textContent=s.demo?'模拟模式 · 无实车输出':'实时画面';$('reverse-note').textContent=s.reverse_enabled?'倒车已启用：默认1450，下限1350；先松开前进回中，再按后退。':'倒车待标定，当前锁定。↓不会自动执行电调倒车序列。';document.querySelector('[data-key="down"]').disabled=!s.reverse_enabled;
for(const k of ['forward','reverse','left','right']){$(k).disabled=!!c.armed;if(!settingsDirty&&document.activeElement!==$(k))$(k).value=s.settings[k];}recording=s.recording;$('record').textContent=recording?'■ 结束录制':'● 开始录制';$('record').disabled=s.saving;$('record-state').textContent=s.saving?'打包中':recording?'● 正在录制':'未录制';if(s.record_error)notice('保存异常：'+s.record_error);if(Object.keys(s.errors).length)notice(Object.values(s.errors).join('；'));
const signature=s.files.join('|');if($('files').dataset.signature!==signature){$('files').dataset.signature=signature;$('files').replaceChildren();for(const name of [...s.files].reverse()){const a=document.createElement('a');a.textContent='↓ '+name;a.href='/download/'+encodeURIComponent(name)+'?token='+encodeURIComponent(token);a.download=name;$('files').append(a);}}
}catch(e){if(active)stop('连接断开，已请求停止');notice(e.message==='访问令牌失效，请重新打开服务入口'?e.message:'连接已断开，旧画面已停止；控制已锁定。');$('arm').disabled=true;$('connection').textContent='离线';$('connection').classList.remove('good');}finally{setTimeout(poll,80);}}
async function camera(){try{const r=await fetch('/camera.jpg',{headers:{'X-Control-Token':token},cache:'no-store',signal:AbortSignal.timeout(1500)});if(r.ok){const u=URL.createObjectURL(await r.blob());$('camera').src=u;if(cameraURL)URL.revokeObjectURL(cameraURL);cameraURL=u;cameraFreshAt=performance.now();}}catch(_){}setTimeout(camera,100);}
const canvas=$('lidar'),ctx=canvas.getContext('2d');
function draw(scan,age){const w=640,h=480,cx=320,cy=255,scale=18;ctx.fillStyle='#0d1c24';ctx.fillRect(0,0,w,h);ctx.strokeStyle='#29414d';ctx.lineWidth=1;ctx.fillStyle='#78929e';ctx.font='11px sans-serif';for(let m=2;m<=12;m+=2){ctx.beginPath();ctx.arc(cx,cy,m*scale,0,Math.PI*2);ctx.stroke();ctx.fillText(m+'m',cx+5,cy-m*scale+13);}ctx.beginPath();ctx.moveTo(cx,20);ctx.lineTo(cx,h-15);ctx.moveTo(30,cy);ctx.lineTo(w-30,cy);ctx.stroke();ctx.fillStyle='#72e2bc';ctx.beginPath();ctx.moveTo(cx,cy-9);ctx.lineTo(cx-6,cy+7);ctx.lineTo(cx+6,cy+7);ctx.closePath();ctx.fill();ctx.fillText('前方',cx-12,16);if(scan){ctx.fillStyle=age<1?'#72e2bc':'#6a7a7e';scan.ranges.forEach((r,i)=>{if(r===null)return;const a=i*Math.PI/180;ctx.fillRect(cx+Math.sin(a)*r*scale-1.5,cy-Math.cos(a)*r*scale-1.5,3,3);});$('lidar-quality').textContent=(age<1?'':'过期 · ')+'有效回波 '+Math.round(scan.valid_fraction*100)+'%';}}
async function lidar(){try{const x=await api('/api/lidar');draw(x.scan,x.age_s);if(x.scan&&x.age_s<1)lidarFreshAt=performance.now();}catch(_){}setTimeout(lidar,100);}
if(!token)notice('缺少访问令牌，请使用服务生成的完整入口链接。');else notice('等待画面与雷达就绪，默认锁定。');setInterval(()=>send(),80);poll();camera();lidar();draw(null,Infinity);

setInterval(()=>{const fresh=performance.now()-cameraFreshAt<1200&&performance.now()-pollAt<1200;document.querySelector('.camera-stage').classList.toggle('stale',!fresh);document.querySelector('.live').textContent=fresh?'● LIVE':'画面已停止';if(performance.now()-lidarFreshAt>1200)$('lidar-quality').textContent='雷达画面已停止';},250);
async function savePhoto(kind){
 try {const result=await api('/api/photo',{kind});const a=document.createElement('a');a.href='/download/'+encodeURIComponent(result.file)+'?token='+encodeURIComponent(token);a.download=result.file;a.textContent='↓ '+result.file;$('photo-files').prepend(a);a.click();notice('单张图片已保存：'+result.file+'；下方链接可重新下载。');}
 catch(e){notice(e.message);}
}
$('save-camera').onclick=()=>savePhoto('camera');
$('save-lidar').onclick=()=>savePhoto('lidar');
$('save-combined').onclick=()=>savePhoto('combined');

// Separate, slow diagnostic polling never enters the control request chain.
let visionSequence = -1, visionURL = null;
async function visionPoll(){
 try {
  const s=await api('/api/vision'), r=s.result;
  $('vision-status').textContent=!s.enabled?'未启用':s.error?'推理已停止':r&&r.age_ms<1500?'● 只读推理':'等待新结果';
  if(!s.enabled){$('vision-info').textContent='未配置模型；原始相机和遥控照常使用。';}
  else if(s.error){$('vision-info').textContent=s.error;$('vision-image').style.opacity='.4';}
  else if(r){
   const d=r.diagnostics,p=r.performance;
   $('vision-info').textContent=`帧龄 ${Math.round(r.age_ms)} ms · 推理 ${d.inference_ms.toFixed(1)} ms · 预处理 ${d.preprocess_ms.toFixed(1)} ms · 几何 ${d.geometry_ms.toFixed(1)} ms · 丢弃 ${s.dropped} 帧 · P95 ${p.p95_ms===null?'预热中':p.p95_ms.toFixed(1)+' ms'} · ${d.simulation_only?'模拟标定，米制结果不可用于实车导航':d.calibration_status}`;
   $('vision-image').style.opacity=r.age_ms<1500?'1':'.4';
   if(r.sequence!==visionSequence){
    visionSequence=r.sequence;
    const bytes=Uint8Array.from(atob(r.jpeg_base64),c=>c.charCodeAt(0));
    const imageURL=URL.createObjectURL(new Blob([bytes],{type:'image/jpeg'}));
    $('vision-image').src=imageURL;if(visionURL)URL.revokeObjectURL(visionURL);visionURL=imageURL;
    $('vision-image').hidden=false;$('save-vision').hidden=false;
    $('vision-elements').textContent=JSON.stringify({frame:r.sequence,captured_at_ms:r.captured_at_ms,light:r.road.observation.light,elements:r.road.elements?.observations||[]},null,2);
   }
  }
 }catch(_){$('vision-status').textContent='诊断连接中断';$('vision-image').style.opacity='.4';}
 setTimeout(visionPoll,500);
}
$('save-vision').onclick=()=>savePhoto('vision');
visionPoll();
