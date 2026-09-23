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
