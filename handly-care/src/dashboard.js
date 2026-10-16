import { live, POSES } from './live.js';

const ROOMS = [
  {n:'10', pt:'M.K.', age:'74 F', pod:1, proc:'Lap. cholecystectomy', flags:['Fall risk']},
  {n:'11', pt:'J.R.', age:'81 M', pod:1, proc:'Inguinal hernia repair', flags:['Sleep apnea','Delirium risk']},
  {n:'12', pt:'A.T.', age:'68 F', pod:2, proc:'Lap. appendectomy', flags:[]}
];
const SEED_LOG = [
  ['01:52','12','Arm','Handed water cup to A.T.'],
  ['01:31','11','Nurse','Pain reassessed, 3/10. Documented.'],
  ['00:48','11','Arm','Handed tissues to J.R.'],
  ['23:58','10','Arm','Handed water bottle to M.K.']
];
const SEED_LAST = {'10':['Water bottle','23:58'],'11':['Tissues','00:48'],'12':['Water cup','01:52']};
const START_SEC = 2*3600 + 10*60;
const CLOCK_RATIO = 30;   // unit clock: 1 real second = 30 s
const RESP_RATIO = 10;    // simulated arm response time scale
const STEP_GAP = 9000;
const WHO_COLOR = {Arm:'var(--teal)', Nurse:'var(--green)', Patient:'var(--indigo)', Alert:'var(--red)', System:'var(--text2)'};
const SEV_COLOR = {critical:'var(--red)', approve:'var(--blue)', review:'var(--orange)'};
const SEV_ORDER = {critical:0, approve:1, review:2};
const SEV_LABEL = {critical:'Critical', approve:'Approval', review:'Review'};

const SETTINGS_KEY = 'bedside-arm-settings';
const settings = (() => { try { return Object.assign({room:'12', hold:3}, JSON.parse(localStorage.getItem(SETTINGS_KEY)||'{}')); } catch { return {room:'12', hold:3}; } })();
const saveSettings = () => { try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings)); } catch {} };

let S;
function fresh(){
  return {
    sim:0, running:false, speed:1, jobs:[], step:0, nextAt:1500,
    rooms:Object.fromEntries(ROOMS.map(r=>[r.n,{...r, arm:'idle', phase:'', start:0, end:0, token:null, live:false, progress:0, last:SEED_LAST[r.n]}])),
    queue:[], log:SEED_LOG.map((e,i)=>({id:'s'+i, t:e[0], room:e[1], who:e[2], text:e[3]})),
    handled:4, saved:16, resp:[52,61,58,61], uid:0
  };
}
S = fresh();
let view = 'overview';

const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"]/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
function clock(){ const s=Math.floor(START_SEC + S.sim/1000*CLOCK_RATIO)%86400; const h=Math.floor(s/3600), m=Math.floor(s%3600/60); return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0'); }
function after(ms, fn){ S.jobs.push({at:S.sim+ms, fn}); }
function log(room, who, text){ S.log.unshift({id:'l'+(S.uid++), t:clock(), room, who, text}); }
const cap = s => s.charAt(0).toUpperCase()+s.slice(1);
const fmt = v => (v<0?'−':'')+Math.abs(Math.round(v));

let actx=null, soundOn=false;
function chime(){ if(!soundOn||!actx) return; try{ [880,660].forEach((f,i)=>{ const o=actx.createOscillator(), g=actx.createGain(); o.frequency.value=f; o.connect(g); g.connect(actx.destination); const t=actx.currentTime+i*.22; g.gain.setValueAtTime(.0001,t); g.gain.exponentialRampToValueAtTime(.25,t+.02); g.gain.exponentialRampToValueAtTime(.0001,t+.2); o.start(t); o.stop(t+.22); }); }catch(e){} }

/* Arm tasks */
function finishTask(n, item, opts, secs){
  const r = S.rooms[n];
  r.arm='idle'; r.phase=''; r.token=null; r.live=false; r.progress=0;
  r.last=[opts.lastLabel||cap(item), clock()];
  if(opts.count!==false){ S.handled++; S.saved+=4; S.resp.push(secs); }
  log(n,'Arm', (opts.doneText||('Handed '+item+' to '+r.pt)) + ' · ' + secs + ' s' + (opts.isLive?' · real arm':''));
  if(opts.done) opts.done(secs);
  renderAll();
}
function armTask(n, item, opts){
  opts = opts || {};
  const r = S.rooms[n];
  if (n === settings.room && live.connected && !opts.noLive) {
    const task = live.taskFor(item, opts.kind);
    if (!task) log(n,'System','No real-arm routine for '+item+' yet. Simulating it.');
    else if (live.busy) log(n,'System','Real arm busy. Simulating this hand-over.');
    else if (!confirm('Real arm: '+item+' for Room '+n+'.\n\nCheck: '+live.checklist(task)+'\n\nStart?')) { log(n,'Nurse','Real-arm '+item+' cancelled before start.'); renderAll(); return; }
    else return liveTask(n, item, opts);
  }
  const token = {}; r.token = token; r.arm='working'; r.live=false;
  const requested = opts.requestedAt != null ? opts.requestedAt : S.sim;
  const phases = opts.phases || [['Picking up '+item, 2500], ["Finding patient's hand", 2000], ['Handing over', 2000]];
  const total = phases.reduce((a,p)=>a+p[1],0);
  r.start=S.sim; r.end=S.sim+total; r.phase=phases[0][0];
  let acc=0;
  phases.forEach(([label,d])=>{ after(acc, ()=>{ if(r.token!==token) return; r.phase=label; renderArms(); }); acc+=d; });
  after(total, ()=>{ if(r.token!==token) return; finishTask(n, item, opts, Math.round((S.sim-requested)/1000*RESP_RATIO)); });
  renderAll();
}
function liveTask(n, item, opts){
  const r = S.rooms[n]; const t0 = Date.now();
  r.arm='working'; r.live=true; r.token=null; r.phase='Starting'; r.progress=0;
  log(n,'System', opts.kind==='temp' ? 'Real arm started temperature check.' : 'Real arm started hand-over of '+item+'.');
  renderAll();
  const onStep = (label, p) => { r.phase = label; r.progress = p; renderArms(); };
  (opts.kind==='temp' ? live.measure({ readSeconds: 3, onStep }) : live.handover({ item, onStep })).then(() => {
    finishTask(n, item, {...opts, isLive:true}, Math.round((Date.now()-t0)/1000));
  }).catch(err => {
    r.arm='stopped'; r.live=false; r.phase=''; r.progress=0;
    log(n,'System','Real arm stopped: ' + (err && err.message ? err.message : err));
    renderAll();
  });
}

function addItem(it){ it.id='q'+(S.uid++); it.t=clock(); it.state=''; S.queue.push(it); if(it.sev==='critical') chime(); }
function removeItem(id){ S.queue = S.queue.filter(q=>q.id!==id); }

const SCRIPT = [
  // 1. Pill box: nurse-prepared, nurse releases, arm hands it over
  ()=>{ log('12','System','03:00 medication box is due.');
        addItem({sev:'approve', room:'12', title:'03:00 meds ready to hand over', detail:'Pill box prepared and verified by R. Alvarez, RN at 7:15pm · Acetaminophen, docusate', acts:[['Release pill box','release','primary'],['Hold','hold','']]}); renderAll(); },
  // 2. Water
  ()=>{ log('12','Patient','Voice request: "Could I have a tissue?"');
        log('12','System','Routine request. Sent to arm.');
        armTask('12','tissue',{requestedAt:S.sim-600}); },
  // Risk: respiratory depression in another room, nurse only
  ()=>{ log('11','Alert','SpO₂ 88% and falling. Arm actions locked for this room.');
        addItem({sev:'critical', room:'11', title:'SpO₂ 88% and falling', detail:'Sleep apnea · opioid at 01:15 · RR 8 · hard to wake · was 95% at 01:40', acts:[['On my way','enroute','danger']]}); renderAll(); },
  // 3. Temperature: arm measures without contact, nurse reviews
  ()=>{ log('12','System','Scheduled temperature check. Arm announces it before moving.');
        armTask('12','IR thermometer',{kind:'temp', count:false, lastLabel:'Temp check',
          phases:[['Announcing temperature check',1500],['Moving slowly near forehead',2500],['Reading temperature',2000],['Returning home',1500]],
          doneText:'Temperature 38.4 °C, flagged for nurse review',
          done:()=>{ addItem({sev:'review', room:'12', title:'Temperature 38.4 °C', detail:'Non-contact reading by arm · 37.4 °C at 11pm · POD 2 appendectomy', acts:[["I'll assess",'assess','primary'],['Acknowledge','ack','']]}); }}); },
  // Risk: fall, routed to staff
  ()=>{ log('10','Patient','Voice request: "I need to get to the bathroom."');
        log('10','System','Not an arm task. Fall risk, routed to staff.');
        addItem({sev:'review', room:'10', title:'Needs help to the bathroom', detail:'74 F · POD 1 cholecystectomy · Fall risk · Bed alarm on', acts:[["I'll go",'go','primary'],['Send J. Ortiz, CNA','aide','']]}); renderAll(); },
  ()=>{ log('', 'System','End of demo script.'); renderAll(); }
];
function fireStep(){ if(S.step>=SCRIPT.length) return; SCRIPT[S.step++](); S.nextAt = S.sim + STEP_GAP; renderControls(); }

function act(id, a){
  const it = S.queue.find(q=>q.id===id); if(!it) return;
  const r = S.rooms[it.room];
  switch(a){
    case 'enroute': it.state='Frank en route'; it.acts=[['Mark attended','attended','primary']]; log(it.room,'Nurse','Frank Sinatra, RN on the way.'); break;
    case 'attended': removeItem(id); log(it.room,'Nurse','Attended. Patient roused, O₂ applied, SpO₂ back to 94%. Provider notified.'); break;
    case 'release': removeItem(id); log(it.room,'Nurse','Pill box released for hand-over.');
      armTask(it.room,'pill box',{lastLabel:'Pill box', doneText:'Handed nurse-prepared pill box to '+r.pt+'. Logged to MAR'}); return;
    case 'hold': removeItem(id); log(it.room,'Nurse','Medication hand-over held.'); break;
    case 'assess': it.state='Assessment pending'; it.acts=[['Mark assessed','assessed','primary']]; log(it.room,'Nurse','Will assess in person.'); break;
    case 'assessed': removeItem(id); log(it.room,'Nurse','Assessed. Provider notified, cultures ordered.'); break;
    case 'ack': removeItem(id); log(it.room,'Nurse','Reading acknowledged. Recheck in 1 hour.'); break;
    case 'go': it.state='Frank en route'; it.acts=[['Mark done','done','primary']]; log(it.room,'Nurse','Frank Sinatra, RN on the way.'); break;
    case 'aide': removeItem(id); log(it.room,'Nurse','Assigned to J. Ortiz, CNA.'); break;
    case 'done': removeItem(id); log(it.room,'Nurse','Done.'); break;
  }
  renderAll();
}
function roomAct(n, a){
  const r = S.rooms[n];
  if(a==='stop'){
    if (r.live) { live.stop(); log(n,'Nurse','Real arm stopped by nurse.'); renderAll(); return; }
    r.token=null; r.arm='stopped'; r.phase=''; log(n,'Nurse','Arm stopped by nurse. Item returned to tray.');
  }
  if(a==='resume'){ r.arm='idle'; log(n,'Nurse','Arm back in service.'); }
  if(a==='handover'){ log(n,'Nurse','Hand-over started from the console.'); armTask(n,'tissue',{lastLabel:'Tissue', doneText:'Handed a tissue to '+r.pt}); return; }
  renderAll();
}

/* Rendering */
function renderQueue(){
  const q = [...S.queue].sort((a,b)=>SEV_ORDER[a.sev]-SEV_ORDER[b.sev]);
  $('queue').innerHTML = q.length ? q.map(it=>`
    <article class="item ${it.sev}">
      <span class="stripe" style="background:${SEV_COLOR[it.sev]}"></span>
      <div class="body">
        <div class="meta"><span class="b ${it.sev}">${SEV_LABEL[it.sev]}</span><span>Room ${esc(it.room)}</span><span class="t">${esc(it.t)}</span></div>
        <div class="title">${esc(it.title)}</div>
        <div class="detail">${esc(it.detail)}</div>
        ${it.state?`<div class="state">${esc(it.state)}</div>`:''}
        <div class="acts">${it.acts.map(([l,a,c])=>`<button class="btn sm ${c}" data-item="${it.id}" data-act="${a}">${esc(l)}</button>`).join('')}</div>
      </div>
    </article>`).join('')
    : `<div class="empty"><span class="ok"><svg viewBox="0 0 24 24"><path d="M5 12l4 4 10-10"/></svg></span><div><strong>All clear</strong>Arms are covering routine requests. Alerts and approvals land here.</div></div>`;
  const crit = q.filter(i=>i.sev==='critical').length;
  $('needCount').textContent = q.length ? (q.length+' waiting'+(crit?' · '+crit+' critical':'')) : 'Nothing waiting';
  $('navQueue').textContent = q.length;
  $('navQueue').classList.toggle('hot', crit>0);
}
function renderArms(){
  const hotRooms = new Set(S.queue.filter(q=>q.sev==='critical').map(q=>q.room));
  const openBy = {}; S.queue.forEach(q=>{ if(!openBy[q.room]||SEV_ORDER[q.sev]<SEV_ORDER[openBy[q.room].sev]) openBy[q.room]=q; });
  $('armRows').innerHTML = ROOMS.map(({n})=>{
    const r = S.rooms[n]; const open = openBy[n]; const hot = hotRooms.has(n);
    const isLiveRoom = n === settings.room && live.connected;
    const state = r.arm==='working' ? '<span class="b arm">Working</span>'
               : r.arm==='stopped' ? '<span class="b stopped">Stopped</span>'
               : hot ? '<span class="b critical">Locked</span>'
               : '<span class="b">Idle</span>';
    let btn = '';
    if (r.arm==='working') btn = `<button class="btn sm danger" data-room-act="stop" data-rn="${n}">Emergency Stop</button>`;
    else if (r.arm==='stopped') btn = `<button class="btn sm" data-room-act="resume" data-rn="${n}">Resume</button>`;
    else if (isLiveRoom && !hot) btn = `<button class="btn sm" data-room-act="handover" data-rn="${n}" ${live.ready && !live.busy?'':'disabled'}>Hand Over</button>`;
    const lastLine = r.last ? `Last handed: ${esc(r.last[0])} <span class="t">· ${esc(r.last[1])}</span>` : '<span class="t">No items handed since 7pm</span>';
    const task = r.arm==='working'
      ? `<div class="phase">${esc(r.phase)}</div><div class="bar"><i data-bar="${n}"></i></div>`
      : open ? `<div class="waiting">${esc(open.title)}</div><div class="last">${lastLine}</div>`
      : `<div class="last">${lastLine}</div>`;
    const pose = isLiveRoom && live.pose ? `<div class="liveline">x ${fmt(live.pose.x)} · y ${fmt(live.pose.y)} · z ${fmt(live.pose.z)} mm${live.moving?' · moving':''}</div>` : '';
    return `<tr class="${hot?'hot':''}">
      <td><span class="roomno">${esc(n)}</span></td>
      <td><div class="pt">${esc(r.pt)} <span class="age">${esc(r.age)}</span>${r.flags.map(f=>`<span class="b flag">${esc(f)}</span>`).join('')}</div><div class="sub">POD ${r.pod} · ${esc(r.proc)}</div></td>
      <td><div class="badges">${state}${isLiveRoom?'<span class="b live">Real arm</span>':''}${open?`<span class="b ${open.sev}">${SEV_LABEL[open.sev]}</span>`:''}</div></td>
      <td class="task">${task}${pose}</td>
      <td class="r">${btn}</td>
    </tr>`;
  }).join('');
  const working = ROOMS.filter(r=>S.rooms[r.n].arm==='working').length;
  $('armCount').textContent = '3 online' + (working? ' · '+working+' working':'');
  tickBars();
}
function renderLog(){
  if (!$('log')) return;
  $('log').innerHTML = S.log.map(e=>`
    <div class="ent"><span class="tm">${esc(e.t)}</span><span class="who" style="color:${WHO_COLOR[e.who]}">${esc(e.who)}</span><span class="rm">${e.room?'Room '+esc(e.room):''}</span><span class="txt">${esc(e.text)}</span></div>`).join('');
}
function renderStats(){
  const crit = S.queue.some(q=>q.sev==='critical');
  $('vNeed').textContent = S.queue.length; $('statNeed').classList.toggle('alert', crit);
  $('vNeedNote').textContent = crit ? 'Critical alert waiting' : 'Alerts and approvals';
  $('vHandled').textContent = S.handled; $('vSaved').textContent = S.saved;
  $('vResp').textContent = Math.round(S.resp.reduce((a,b)=>a+b,0)/S.resp.length);
}
function renderControls(){
  $('btnRun').textContent = S.running ? 'Pause' : (S.sim>0 ? 'Resume' : 'Start demo');
  $('btnRun').className = S.running ? 'btn' : 'btn primary';
  $('btnNext').disabled = S.step>=SCRIPT.length;
  document.querySelectorAll('[data-speed]').forEach(b=>b.setAttribute('aria-pressed', String(+b.dataset.speed===S.speed)));
  $('topbar').classList.toggle('running', S.running);
}
function tickBars(){
  ROOMS.forEach(({n})=>{
    const r=S.rooms[n]; const el=document.querySelector(`[data-bar="${n}"]`); if(!el) return;
    const pct = r.live ? r.progress*100 : (r.end>r.start ? (S.sim-r.start)/(r.end-r.start)*100 : 0);
    el.style.width = Math.min(100, Math.max(0, pct))+'%';
  });
}
function renderAll(){ renderQueue(); renderArms(); renderLog(); renderStats(); renderControls(); renderLive(); $('clock').textContent = clock(); }

/* Views */
function setView(v, target){
  view = v;
  $('viewOverview').hidden = v!=='overview';
  $('viewSetup').hidden = v!=='setup';
  $('pageTitle').textContent = v==='setup' ? 'Arm Setup' : 'Overview';
  $('pageSub').textContent = v==='setup' ? 'Connect and teach the real arm' : 'Night shift · 3 patients · avg age 74';
  document.querySelectorAll('.navitem').forEach(b=>{
    const on = b.dataset.view===v && (v==='setup' || (b.dataset.target||'top')===(target||'top'));
    if (on) b.setAttribute('aria-current','page'); else b.removeAttribute('aria-current');
  });
  if (v==='overview' && target && target!=='top') { const el=$(target); if(el) el.scrollIntoView({behavior:'smooth', block:'start'}); }
  else window.scrollTo({top:0});
}

/* Arm Setup */
const STATUS_TEXT = {disconnected:'Not connected', connecting:'Connecting…', connected:'Connected', error:'Connection problem'};
function renderLive(){
  const st = live.status;
  $('liveDot').className = 'ldot ' + st; $('navDot').className = 'ldot ' + st;
  $('btnLiveText').textContent = st==='connected' ? 'Arm connected' : (st==='connecting' ? 'Connecting…' : 'Connect Arm');
  $('liveStatus').innerHTML = `<span class="ldot ${st}"></span>` + esc(STATUS_TEXT[st] + (st==='error' && live.error ? ': ' + live.error : ''));
  $('liveStatus').className = 'lstatus ' + st;
  $('btnConnect').textContent = st==='connected' ? 'Disconnect' : (st==='connecting' ? 'Connecting…' : 'Connect');
  $('btnConnect').className = st==='connected' ? 'btn' : 'btn primary';
  $('btnConnect').disabled = st==='connecting';
  if (st==='connected' && live.sim) $('liveStatus').innerHTML += ' · <b>SIMULATED arm</b>';

  const res = live.result;
  $('poseList').innerHTML = POSES.map(p => {
    const now = live.busy && live.task===p.key;
    const mine = res && res.task===p.key;
    const last = mine ? (res.ok ? `<span class="b ok">Done · ${res.seconds} s</span>` : '<span class="b stopped">Failed</span>') : '<span class="b">—</span>';
    return `<tr>
      <td><div class="pname">${esc(p.label)}</div><div class="phint">${esc(p.hint)}</div></td>
      <td class="angles">${now ? esc(live.step)+' · '+Math.round(live.progress*100)+'%' : (mine && !res.ok ? esc(res.message) : '')}</td>
      <td>${now?'<span class="b arm">Running</span>':last}</td>
      <td class="r"><button class="btn sm" data-capture="${p.key}" ${live.connected && !live.busy ? '' : 'disabled'}>Run</button></td>
    </tr>`;
  }).join('');
  $('teachNote').textContent = !live.connected ? 'Start the bridge on this laptop (python bridge.py), then Connect.'
    : 'Room ' + settings.room + ' runs on the real arm. Medicine and tissue requests use these routines.';
  $('btnTest').disabled = !live.ready || live.busy;
  $('btnLiveStop').disabled = !live.connected;
  $('movingState').innerHTML = live.connected ? (live.busy ? '<span style="color:var(--teal);font-weight:500">Running '+esc(live.task)+'</span>' : 'Idle') : 'No data';
  $('readout').innerHTML = live.connected && live.lines.length
    ? `<pre class="armout">${esc(live.lines.join('\n'))}</pre>`
    : `<div class="nodata">${live.connected ? 'No runs yet.' : 'Connect to see the arm output.'}</div>`;
}

// A routine with no hand-over (start pose): runs from Arm Setup only, after the nurse confirms.
function runSetup(task){
  const n = settings.room;
  if (!confirm('Real arm: '+task+'.\n\nCheck: '+live.checklist(task)+'\n\nStart?')) return;
  log(n,'Nurse','Arm Setup: '+task+' started.'); renderAll();
  live.run(task).then(r => { log(n,'System','Arm Setup: '+task+' done · '+r.seconds+' s.'); renderAll(); })
    .catch(err => { log(n,'System','Arm Setup: '+task+' failed: '+(err.message||err)); renderAll(); });
}

function initLive(){
  $('liveRoom').innerHTML = ROOMS.map(r => `<option value="${r.n}">Room ${r.n} · ${r.pt}</option>`).join('');
  $('liveRoom').value = settings.room;
  $('hold').value = settings.hold;

  $('btnLive').addEventListener('click', () => setView('setup'));
  $('liveForm').addEventListener('submit', e => {
    e.preventDefault();
    if (live.connected) { live.disconnect(); return; }
    live.connect().then(() => { if (live.connected) { log(settings.room,'System','Real arm connected in Room '+settings.room+(live.sim?' (simulated)':'')+'.'); renderAll(); } });
  });
  $('liveRoom').addEventListener('change', e => { settings.room = e.target.value; saveSettings(); renderAll(); });
  $('hold').addEventListener('change', e => { settings.hold = Math.max(1, Math.min(15, +e.target.value || 3)); e.target.value = settings.hold; saveSettings(); });
  $('poseList').addEventListener('click', async e => {
    const b = e.target.closest('[data-capture]'); if (!b) return;
    const task = b.dataset.capture, n = settings.room;
    if (task === 'medicine') return armTask(n,'pill box',{lastLabel:'Pill box', doneText:'Handed pill box to '+S.rooms[n].pt+' (from Arm Setup)'});
    if (task === 'tissue') return armTask(n,'tissue',{lastLabel:'Tissue', doneText:'Handed a tissue to '+S.rooms[n].pt+' (from Arm Setup)'});
    runSetup(task);
  });
  $('btnTest').addEventListener('click', () => runSetup('to_high'));
  $('btnLiveStop').addEventListener('click', () => { live.stop(); log(settings.room,'Nurse','Real arm stopped from Arm Setup.'); renderAll(); });

  let pending = false;
  live.on(() => { if (pending) return; pending = true; requestAnimationFrame(() => { pending = false; renderLive(); renderArms(); }); });
}

/* Main loop */
setInterval(()=>{
  if(!S.running) return;
  S.sim += 250*S.speed;
  const due = S.jobs.filter(j=>j.at<=S.sim).sort((a,b)=>a.at-b.at);
  S.jobs = S.jobs.filter(j=>j.at>S.sim);
  due.forEach(j=>j.fn());
  if(S.step<SCRIPT.length && S.sim>=S.nextAt) fireStep();
  $('clock').textContent = clock();
  tickBars();
}, 250);

document.addEventListener('click', e=>{
  const b = e.target.closest('button'); if(!b) return;
  if(b.dataset.item) return act(b.dataset.item, b.dataset.act);
  if(b.dataset.roomAct) return roomAct(b.dataset.rn, b.dataset.roomAct);
  if(b.dataset.speed){ S.speed = +b.dataset.speed; renderControls(); return; }
  if(b.dataset.view) return setView(b.dataset.view, b.dataset.target);
});
$('btnRun').addEventListener('click', ()=>{ S.running=!S.running; renderControls(); });
$('btnNext').addEventListener('click', ()=>{ S.running=true; S.nextAt=S.sim; renderControls(); });
$('btnSound').addEventListener('click', ()=>{
  soundOn=!soundOn;
  if(soundOn && !actx){ try{ actx = new (window.AudioContext||window.webkitAudioContext)(); }catch(e){ soundOn=false; } }
  $('btnSound').textContent = soundOn?'Sound on':'Sound off'; $('btnSound').setAttribute('aria-pressed', String(soundOn));
});
$('btnReset').addEventListener('click', ()=>{ if (live.busy) live.stop(); S = fresh(); renderAll(); });
document.addEventListener('keydown', e=>{ if((e.key==='n'||e.key==='N') && !e.target.closest('input,textarea,select')) $('btnNext').click(); });

initLive();
renderAll();
