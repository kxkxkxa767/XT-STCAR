// DOM/network doubles only: no browser or physical vehicle access.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const path=require('node:path');
function fixture(){
 const events={}, nodes=new Map();
 const node=()=>({textContent:'',value:'',dataset:{},classList:{toggle(){},remove(){}},addEventListener(){},getContext(){return new Proxy({}, {get:()=>()=>{},set:()=>true});},replaceChildren(){},append(){}});
 const ctx=vm.createContext({URLSearchParams,Uint8Array,crypto:require('node:crypto').webcrypto,location:{hash:'',pathname:'/'},sessionStorage:{getItem(){return ''},setItem(){}},history:{replaceState(){}},performance:{now:()=>10000},AbortController,AbortSignal,fetch:()=>new Promise(()=>{}),setInterval(){},setTimeout(){},clearTimeout(){},document:{hidden:false,hasFocus:()=>true,getElementById(id){if(!nodes.has(id))nodes.set(id,node());return nodes.get(id)},querySelectorAll(){return []},querySelector(){return node()},addEventListener(name,fn){events[name]=fn}},window:{addEventListener(name,fn){events[name]=fn}}});
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../app.js'),'utf8'),ctx);
 return {ctx,events,run:code=>vm.runInContext(code,ctx)};
}
test('viewer blur, hide and unload do not stop another controller',()=>{
 const f=fixture();f.run('var calls=0,fetchCalls=0;api=async()=>{calls++;return {}};fetch=async()=>{fetchCalls++;return {}}');
 f.events.blur();f.run('document.hidden=true');f.events.visibilitychange();f.events.pagehide();
 assert.equal(f.run('calls'),0);assert.equal(f.run('fetchCalls'),0);
 f.run('active=true');f.events.blur();assert.equal(f.run('calls'),1);assert.equal(f.run('active'),false);
});
test('forced key release queues latest state without parallel drive requests',async()=>{
 const f=fixture();f.run('var requests=[],resolvers=[];api=(p,data)=>{requests.push(data);return new Promise(r=>resolvers.push(r))};active=true;pollAt=performance.now();keys.add("up");send();keys.clear();send(true)');
 assert.equal(f.run('requests.length'),1);
 f.run('resolvers.shift()({})');await new Promise(setImmediate);
 assert.equal(f.run('requests.length'),2);assert.equal(f.run('requests[1].keys.length'),0);
 f.run('resolvers.shift()({})');await new Promise(setImmediate);
 assert.equal(f.run('pending'),false);
});
test('explicit stop bypasses in-flight drive and cancels queued sends',async()=>{
 const f=fixture();f.run('var requests=[],resolvers=[];api=(p,data)=>{requests.push(data);return new Promise(r=>resolvers.push(r))};active=true;pollAt=performance.now();send();send(true);stop()');
 assert.equal(f.run('requests[1].op'),'stop');
 f.run('resolvers.shift()({})');await new Promise(setImmediate);
 assert.equal(f.run('requests.length'),2);assert.equal(f.run('active'),false);
});
test('heartbeat due during a request sends latest keys immediately after acknowledgement',async()=>{
 const f=fixture();f.run('var requests=[],resolvers=[];api=(p,data)=>{requests.push(data);return new Promise(r=>resolvers.push(r))};active=true;pollAt=performance.now();keys.add("up");send();keys.clear();send()');
 assert.equal(f.run('requests.length'),1);
 f.run('resolvers.shift()({})');await new Promise(setImmediate);
 assert.equal(f.run('requests.length'),2);
 assert.equal(f.run('requests[1].keys.length'),0);
 f.run('resolvers.shift()({})');await new Promise(setImmediate);
});
test('fresh commands advance the sampled bridge clock but pending commands keep their timestamp',()=>{
 const f=fixture();f.run('var requests=[];api=(p,data)=>{requests.push(data);return new Promise(()=>{})};active=true;tick=1000;pollAt=performance.now()-140;send()');
 assert.equal(f.run('requests[0].tick'),1140);
 const sent=f.run('requests[0].tick');f.run('pollAt-=50;send()');
 assert.equal(f.run('requests[0].tick'),sent);
 assert.equal(f.run('requests.length'),1);
});
test('old clock sample stops rather than extrapolating indefinitely',()=>{
 const f=fixture();f.run('var requests=[];api=async(p,data)=>{requests.push(data);return {}};active=true;tick=1000;pollAt=performance.now()-251;send()');
 assert.equal(f.run('requests.length'),1);assert.equal(f.run('requests[0].op'),'stop');
 assert.equal(f.run('active'),false);
});
test('unlock samples fresh state and cancellation during sampling cannot arm',async()=>{
 const f=fixture();f.run('var requests=[],resolvers=[];api=(p,data)=>{requests.push({p,data});return new Promise(r=>resolvers.push(r))};$("arm").onclick()');
 assert.equal(f.run('requests[0].p'),'/api/state');
 f.run('resolvers.shift()({healthy:true,owner:null,boot:"new",status:{control:{tick:900}}})');await new Promise(setImmediate);
 assert.equal(f.run('requests[1].data.op'),'arm');assert.equal(f.run('requests[1].data.tick'),900);
 const g=fixture();g.run('var requests=[],resolvers=[];api=(p,data)=>{requests.push({p,data});return new Promise(r=>resolvers.push(r))};$("arm").onclick();stop();resolvers.shift()({healthy:true,owner:null,status:{control:{tick:900}}})');await new Promise(setImmediate);
 assert.equal(g.run('requests.length'),2);assert.equal(g.run('requests[1].data.op'),'stop');
});
test('manual takeover stops automatic output without silently unlocking again',async()=>{
 for(const mode of ['auto_probe','auto_goal','auto_turn_trial']){
  const f=fixture();f.ctx.takeoverState={...stateSample('takeover-boot',900),healthy:mode!=='auto_turn_trial',owner:'auto-session',autonomy:{mode}};
  f.run('var requests=[];api=async(p,data)=>{requests.push({p,data});return p==="/api/state"?takeoverState:{ok:true}};$("arm").onclick()');
  await new Promise(setImmediate);
  assert.equal(f.run('requests.length'),2);
  assert.equal(f.run('requests[1].data.op'),'takeover');
  assert.equal(f.run('active'),false);
 }
});

function clockFixture(){
 const f=fixture();
 f.run('var now=1000,requests=[],resolvers=[],rejectors=[];performance.now=()=>now;api=(p,data)=>{requests.push({p,data});return new Promise((r,j)=>{resolvers.push(r);rejectors.push(j)})}');
 return f;
}
function stateSample(boot,tick){
 return {boot,healthy:true,owner:null,status:{control:{tick,seq:0,armed:false,motor:1500,servo:1500}},
  settings:{forward:1550,reverse:1450,left:1650,right:1350},ages:{camera:0,lidar:0,control:0},
  errors:{},files:[],autonomy:{mode:'locked'}};
}
async function respond(f,index,at,value){
 f.ctx.now=at;f.ctx.resolvers[index](value);await new Promise(setImmediate);
}
test('older overlapping poll cannot rewind the fresh unlock clock or first heartbeat',async()=>{
 const f=clockFixture();f.run('poll();$("arm").onclick()');
 assert.equal(f.ctx.requests[0].p,'/api/state');assert.equal(f.ctx.requests[1].p,'/api/state');
 await respond(f,1,1030,stateSample('same-boot',1000));
 assert.equal(f.ctx.requests[2].data.op,'arm');
 await respond(f,0,1110,stateSample('same-boot',920));
 assert.equal(f.run('tick'),1000);assert.equal(f.run('pollAt'),1030);
 await respond(f,2,1130,{bridge_seq:1});
 assert.equal(f.ctx.requests[3].data.op,'drive');assert.equal(f.ctx.requests[3].data.tick,1100);
});
test('duplicate and lower bridge ticks do not renew sample freshness',async()=>{
 const f=clockFixture();f.run('poll()');await respond(f,0,1030,stateSample('same-boot',1000));
 f.run('poll()');await respond(f,1,1100,stateSample('same-boot',1000));
 assert.equal(f.run('pollAt'),1030);
 f.run('poll()');await respond(f,2,1200,stateSample('same-boot',999));
 assert.equal(f.run('tick'),1000);assert.equal(f.run('pollAt'),1030);
 f.run('now=1281;active=true;send()');
 assert.equal(f.ctx.requests[3].data.op,'stop');assert.equal(f.run('active'),false);
});
test('late pre-unlock response cannot revive a boot superseded by polling',async()=>{
 const f=clockFixture();f.run('poll()');await respond(f,0,1010,stateSample('old-boot',1000));
 f.run('$("arm").onclick();poll()');
 await respond(f,2,1050,stateSample('new-boot',20));
 await respond(f,1,1080,stateSample('old-boot',1050));
 assert.equal(f.run('boot'),'new-boot');assert.equal(f.run('tick'),20);
 assert.equal(f.ctx.requests.some(r=>r.data?.op==='arm'),false);
 assert.equal(f.run('active'),false);
});
test('slow state response cannot arm or turn an old sample into a fresh clock',async()=>{
 const f=clockFixture();f.run('$("arm").onclick()');
 await respond(f,0,1251,stateSample('same-boot',1000));
 assert.equal(f.ctx.requests.some(r=>r.data?.op==='arm'),false);
 assert.equal(f.run('pollAt'),0);assert.equal(f.run('active'),false);
});
test('request time counts toward the existing sample lifetime without advancing command time',async()=>{
 const f=clockFixture();f.run('poll()');await respond(f,0,1200,stateSample('same-boot',1100));
 f.run('now=1230;active=true;send()');
 assert.equal(f.ctx.requests[1].data.op,'drive');assert.equal(f.ctx.requests[1].data.tick,1130);
 await respond(f,1,1251,{});f.run('send()');
 assert.equal(f.ctx.requests[2].data.op,'stop');assert.equal(f.run('active'),false);
 assert.equal(f.ctx.requests[1].data.tick,1130);
});
test('server restart during pending unlock acknowledgement cancels the first drive',async()=>{
 const f=clockFixture();f.run('$("arm").onclick()');
 await respond(f,0,1010,stateSample('old-boot',1000));
 assert.equal(f.ctx.requests[1].data.op,'arm');
 f.run('poll()');await respond(f,2,1050,stateSample('new-boot',20));
 await respond(f,1,1080,{bridge_seq:1});
 assert.equal(f.run('boot'),'new-boot');assert.equal(f.run('active'),false);
 assert.equal(f.ctx.requests.filter(r=>r.data?.op==='arm').length,1);
 assert.equal(f.ctx.requests.some(r=>r.data?.op==='drive'),false);
 assert.equal(f.ctx.requests.some(r=>r.data?.op==='stop'),true);
});
test('server restart locks an active page and cannot inherit its queued keys',async()=>{
 const f=clockFixture();f.run('poll()');await respond(f,0,1010,stateSample('old-boot',1000));
 f.run('active=true;keys.add("up");poll()');
 await respond(f,1,1050,stateSample('new-boot',20));f.run('send()');
 assert.equal(f.run('active'),false);assert.equal(f.run('keys.size'),0);
 assert.equal(f.ctx.requests.filter(r=>r.data).length,1);
 assert.equal(f.ctx.requests[2].data.op,'stop');
});
test('healthy locked recovery clears only the generic disconnect notice without unlocking',async()=>{
 const f=clockFixture();f.run('poll()');f.ctx.rejectors[0](Error('network error'));await new Promise(setImmediate);
 assert.equal(f.run('$("notice").textContent'),'连接已断开，旧画面已停止；控制已锁定。');
 f.run('now=1100;poll()');await respond(f,1,1120,stateSample('same-boot',1100));
 assert.equal(f.run('$("notice").textContent'),'连接已恢复，控制保持锁定。');
 assert.equal(f.run('active'),false);assert.equal(f.ctx.requests.some(r=>r.data),false);
});
test('healthy state preserves a specific lock cause',async()=>{
 const f=clockFixture();f.run('notice("控制请求延迟或乱序；请重新解锁");poll()');
 await respond(f,0,1030,stateSample('same-boot',1000));
 assert.equal(f.run('$("notice").textContent'),'控制请求延迟或乱序；请重新解锁');
 assert.equal(f.run('active'),false);
});
test('late failed poll cannot overwrite a newer unlock sample or its notice',async()=>{
 const f=clockFixture();f.run('poll();$("arm").onclick()');
 await respond(f,1,1030,stateSample('same-boot',1000));
 f.ctx.rejectors[0](Error('late network error'));await new Promise(setImmediate);
 assert.equal(f.run('$("notice").textContent'),'正在解锁…');
 assert.equal(f.run('tick'),1000);assert.equal(f.run('pollAt'),1030);
 assert.equal(f.ctx.requests.some(r=>r.data?.op==='stop'),false);
});
test('late unlock rejection cannot overwrite an explicit cancellation notice',async()=>{
 const f=clockFixture();f.run('$("arm").onclick()');
 await respond(f,0,1030,stateSample('same-boot',1000));
 f.run('stop("已取消本次解锁")');f.ctx.rejectors[1](Error('late arm rejection'));await new Promise(setImmediate);
 assert.equal(f.run('$("notice").textContent'),'已取消本次解锁');assert.equal(f.run('active'),false);
 assert.equal(f.ctx.requests.some(r=>r.data?.op==='drive'),false);
});
