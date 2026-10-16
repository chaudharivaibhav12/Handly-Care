// Preview stand-in for live.js: same interface, simulated arm, no network.
const POSES = [
  { key: 'home',    label: 'Home',             hint: 'Folded, clear of the patient' },
  { key: 'above',   label: 'Above the item',   hint: '10 cm over the tray spot' },
  { key: 'pick',    label: 'At the item',      hint: 'Gripper around the item' },
  { key: 'handoff', label: "Patient's hand",   hint: 'Where the patient takes it' },
  { key: 'forehead', label: 'Near forehead',   hint: 'Thermometer about 4 cm from the forehead' },
];
const MOCK_JOINTS = {
  home:    [0, -60, 75, 0, 15, 0],
  above:   [-32, -18, 42, 0, 62, -32],
  pick:    [-32, -4, 38, 0, 52, -32],
  handoff: [48, -10, 30, 0, 70, 48],
  forehead: [62, -24, 18, 0, 84, 62],
};
const MOCK_POSE = { home:{x:205,y:0,z:310}, above:{x:340,y:-210,z:190}, pick:{x:352,y:-218,z:92}, handoff:{x:300,y:330,z:250}, forehead:{x:250,y:360,z:420} };
const sleep = ms => new Promise(r => setTimeout(r, ms));

const live = {
  preview: true,
  status: 'disconnected', error: '',
  poses: {}, moving: false, joints: null, pose: null,
  busy: false, abort: false, _at: 'home', _cycle: 0,
  _listeners: new Set(),
  on(fn){ this._listeners.add(fn); },
  emit(){ this._listeners.forEach(fn => { try { fn(this); } catch (e) {} }); },
  get connected(){ return this.status === 'connected'; },
  get missingPoses(){ return POSES.filter(p => !this.poses[p.key]).map(p => p.label); },
  get ready(){ return this.connected && this.missingPoses.length === 0; },
  savedConnection(){ return null; },

  async connect(){
    this.status = 'connecting'; this.error = ''; this.emit();
    await sleep(1100);
    this.status = 'connected'; this._goto('home'); this.emit();
  },
  async disconnect(){ this.status = 'disconnected'; this.moving = false; this.joints = this.pose = null; this.emit(); },
  _goto(k){ this._at = k; this.joints = [...MOCK_JOINTS[k]]; this.pose = { ...MOCK_POSE[k] }; },

  // In preview, "Save current" walks the arm through the four spots so the panel fills in.
  async capture(key){
    this.moving = true; this.emit(); await sleep(500);
    this._goto(key); this.moving = false;
    this.poses[key] = [...MOCK_JOINTS[key]];
    this.emit();
  },
  clearPoses(){ this.poses = {}; this.emit(); },

  async _move(k){ this.moving = true; this.emit(); await this._wait(1300); this._goto(k); this.moving = false; this.emit(); },
  async _wait(ms){ const end = Date.now() + ms; while (Date.now() < end) { if (this.abort) throw new Error('Stopped by nurse'); await sleep(100); } },

  async handover({ holdSeconds = 3, onStep = () => {} } = {}){
    if (!this.ready) throw new Error('Teach all four positions first.');
    this.busy = true; this.abort = false; this.emit();
    const steps = [
      ['Opening gripper',          () => this._wait(400)],
      ['Moving above the item',    () => this._move('above')],
      ['Reaching the item',        () => this._move('pick')],
      ['Gripping',                 () => this._wait(600)],
      ['Lifting',                  () => this._move('above')],
      ["Moving to patient's hand", () => this._move('handoff')],
      ['Holding for patient',      () => this._wait(holdSeconds * 1000)],
      ['Releasing',                () => this._wait(500)],
      ['Returning home',           () => this._move('home')],
    ];
    try {
      for (let i = 0; i < steps.length; i++) {
        if (this.abort) throw new Error('Stopped by nurse');
        onStep(steps[i][0], i / steps.length);
        await steps[i][1]();
      }
      onStep('Done', 1);
    } finally { this.busy = false; this.moving = false; this.emit(); }
  },
  async measure({ readSeconds = 3, onStep = () => {} } = {}){
    if (!this.ready) throw new Error('Teach all positions first.');
    this.busy = true; this.abort = false; this.emit();
    const steps = [
      ['Announcing temperature check', () => this._wait(1500)],
      ['Moving slowly near forehead',  () => this._move('forehead')],
      ['Reading temperature',          () => this._wait(readSeconds * 1000)],
      ['Returning home',               () => this._move('home')],
    ];
    try {
      for (let i = 0; i < steps.length; i++) {
        if (this.abort) throw new Error('Stopped by nurse');
        onStep(steps[i][0], i / steps.length);
        await steps[i][1]();
      }
      onStep('Done', 1);
    } finally { this.busy = false; this.moving = false; this.emit(); }
  },
  async stop(){ this.abort = true; this.moving = false; this.emit(); },
};
