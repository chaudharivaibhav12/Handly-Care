// Live arm = Sidekick's Python, through bridge.py on this laptop (it also serves this page).
// The page asks for a tested routine by name; Python runs exactly `python app.py <routine>`.
// No Viam keys in the browser. The teammate's direct-Viam version is kept in live-viam-browser.js.orig.

export const POSES = [   // the real arm's routines (shown in Arm Setup)
  { key: 'medicine', label: 'Medicine',        hint: 'Open the drawer, hand over the pill box, close it (~60 s). Drawer SHUT before.' },
  { key: 'tissue',   label: 'Tissue',          hint: 'Find the tissue box, hand over one tissue, end at the start pose (~75 s).' },
  { key: 'to_high',  label: 'Start pose',      hint: "Bring the arm to 'high' from anywhere, one joint at a time." },
];

// Before every run the nurse confirms the setup (same checklist as the terminal runs).
const CHECKS = {
  medicine: 'Drawer SHUT, pill box on its spot, arm at the start pose, hand on the E-stop.',
  tissue:   'Tissue box TAPED down with a tissue sticking up, arm at the start pose, hand on the E-stop.',
  to_high:  'Path clear around the arm, hand on the E-stop.',
};

const sleep = ms => new Promise(r => setTimeout(r, ms));
async function api(path, body){
  const res = await fetch('/api/' + path, body === undefined ? { cache: 'no-store' }
    : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || ('bridge error ' + res.status));
  return data;
}

export const live = {
  status: 'disconnected',   // disconnected | connecting | connected | error
  error: '',
  sim: false, busy: false, task: '', step: '', progress: 0, lines: [], result: null,
  moving: false, joints: null, pose: null,
  poses: Object.fromEntries(POSES.map(p => [p.key, true])),
  preview: true,             // no Viam fields to fill in
  _listeners: new Set(), _poll: null, _pollErrors: 0,

  on(fn){ this._listeners.add(fn); },
  emit(){ this._listeners.forEach(fn => { try { fn(this); } catch (e) { console.error(e); } }); },

  get connected(){ return this.status === 'connected'; },
  get missingPoses(){ return []; },
  get ready(){ return this.connected; },
  savedConnection(){ return null; },

  // Which routine does a dashboard request run on the real arm? null = not a real-arm task (stays simulated).
  taskFor(item, kind){
    if (kind === 'temp') return null;                    // thermometer: not built yet
    const s = String(item || '').toLowerCase();
    if (s.includes('pill') || s.includes('medic')) return 'medicine';
    if (s.includes('tissue')) return 'tissue';
    return null;
  },
  checklist(task){ return CHECKS[task] || ''; },

  async connect(){
    if (this.status === 'connecting') return;
    this.status = 'connecting'; this.error = ''; this.emit();
    try { this._apply(await api('status')); this.status = 'connected'; this._startPolling(); }
    catch (e) { this.status = 'error'; this.error = 'Bridge not running (python bridge.py): ' + (e.message || e); }
    this.emit();
  },
  async disconnect(){ this._stopPolling(); this.status = 'disconnected'; this.emit(); },

  _apply(s){
    this.sim = !!s.sim; this.busy = !!s.busy; this.task = s.task || ''; this.step = s.step || '';
    this.progress = s.progress || 0; this.lines = s.lines || []; this.result = s.result || null;
    this.moving = this.busy;
  },
  _startPolling(){
    this._stopPolling();
    this._poll = setInterval(async () => {
      try { this._apply(await api('status')); this._pollErrors = 0; }
      catch (e) { if (++this._pollErrors >= 4) { this.status = 'error'; this.error = 'Lost the bridge: ' + (e.message || e); this._stopPolling(); } }
      this.emit();
    }, 500);
  },
  _stopPolling(){ if (this._poll) clearInterval(this._poll); this._poll = null; },

  // Run one routine and wait for it. Resolves when it worked; throws with Python's reason when it didn't.
  async run(task, { onStep = () => {} } = {}){
    if (!this.connected) throw new Error('Bridge not connected.');
    const { run } = await api('run', { task });
    this.busy = true; this.emit();
    for (;;) {
      await sleep(400);
      let s;
      try { s = await api('status'); } catch { continue; }   // a dropped poll isn't a failed run
      this._apply(s); this.emit();
      if (s.result && s.result.id === run) {
        if (!s.result.ok) throw new Error(s.result.message);
        onStep('Done', 1);
        return s.result;
      }
      if (s.busy) onStep(s.step || 'Working', s.progress || 0);
    }
  },
  handover({ item, onStep } = {}){
    const task = this.taskFor(item);
    if (!task) return Promise.reject(new Error('No real-arm routine for ' + item));
    return this.run(task, { onStep });
  },
  measure(){ return Promise.reject(new Error('Thermometer is not built yet.')); },

  async stop(){
    try { await api('stop', {}); } catch (e) { this.error = 'Stop failed: ' + (e.message || e) + '. USE THE HARDWARE E-STOP.'; }
    this.emit();
  },
};
