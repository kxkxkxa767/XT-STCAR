// DOM/network doubles only: no browser or physical vehicle access.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const path=require('node:path');
function fixture(){
 const events={}, nodes=new Map();
 const node=()=>({textContent:'',value:'',dataset:{},classList:{toggle(){},remove(){}},addEventListener(){},getContext(){return new Proxy({}, {get:()=>()=>{},set:()=>true});},replaceChildren(){},append(){}});
 const ctx=vm.createContext({URLSearchParams,Uint8Array,crypto:require('node:crypto').webcrypto,location:{hash:'',pathname:'/'},sessionStorage:{getItem(){return ''},setItem(){}},history:{replaceState(){}},performance,AbortController,AbortSignal,fetch:()=>new Promise(()=>{}),setInterval(){},setTimeout(){},clearTimeout(){},document:{hidden:false,hasFocus:()=>true,getElementById(id){if(!nodes.has(id))nodes.set(id,node());return nodes.get(id)},querySelectorAll(){return []},querySelector(){return node()},addEventListener(name,fn){events[name]=fn}},window:{addEventListener(name,fn){events[name]=fn}}});
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
