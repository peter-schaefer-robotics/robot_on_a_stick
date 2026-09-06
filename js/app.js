/*
 * Anwendungslogik: Simulationsschleife, Bedienelemente, Stoerungen,
 * Sprach- und Themenumschaltung.
 *
 * Zeitliche Struktur:
 *   - Simulation: feste Schrittweite h = Ts/8, Runge-Kutta 4
 *   - Regler:     alle Ts (also jeden 8. Simulationsschritt)
 *   - Anzeige:    requestAnimationFrame, entkoppelt von der Simulation
 */
(function (root) {
  'use strict';
  var IPM = root.IPM;
  var M = IPM.model;
  var R = IPM.render;

  var SUB = 8;                 // Simulationsschritte je Regeltakt
  var SPAN = 12;               // Zeitfenster des Scopes [s]
  var MOUSE_RADIUS = 0.32;     // Wirkradius der Maus [m]
  var LS = 'ipm-mpc:';

  // --------------------------------------------------------------- Zustand --
  var sim = {
    s: [0, 0, 0.08, 0],
    t: 0,
    u: 0,
    fd: 0,
    fdMouse: 0,
    fdKick: 0,
    kickTimer: 0,
    running: true,
    xref: [0, 0, 0, 0]
  };

  var opts = {
    noise: 0,          // Standardabweichung des Messrauschens (p in m, theta in rad/5)
    mismatch: 0,       // relativer Fehler der Pendelmasse im Reglermodell
    distStrength: 1,
    showPred: true,
    showForces: true,
    showTrace: false
  };

  var plant = Object.assign({}, M.DEFAULT_PLANT);
  var ctrl = new IPM.MpcController({ plant: plant });
  var lastRes = null;
  // Browser runden performance.now() typischerweise auf 100 us; der gleitende
  // Mittelwert macht die tatsaechliche Rechenzeit trotzdem ablesbar.
  var msAvg = 0;

  var hist = { t: [], th: [], p: [], u: [] };
  var scopeScales = { th: 15, p: 1, u: 15 };
  var trace = [];
  var camX = 0;
  var lastTf = null;
  var mouse = { x: 0, y: 0, vx: 0, active: false, radius: MOUSE_RADIUS, inside: false };

  var els = {};
  var lang = 'de';

  function $(id) { return document.getElementById(id); }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function wrapPi(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }
  function store(k, v) { try { localStorage.setItem(LS + k, v); } catch (e) { /* ignoriert */ } }
  function load(k) { try { return localStorage.getItem(LS + k); } catch (e) { return null; } }

  // ------------------------------------------------------------------ i18n --
  var PLACEHOLDERS = {};

  function applyI18n(next) {
    lang = next;
    var dict = IPM.i18n[lang] || IPM.i18n.de;
    document.documentElement.lang = lang;
    var nodes = document.querySelectorAll('[data-i18n]');
    for (var i = 0; i < nodes.length; i++) {
      var key = nodes[i].getAttribute('data-i18n');
      var val = dict[key];
      if (val === undefined) continue;
      for (var ph in PLACEHOLDERS) {
        if (val.indexOf(ph) >= 0) val = val.split(ph).join(PLACEHOLDERS[ph]);
      }
      if (nodes[i].tagName === 'TITLE') nodes[i].textContent = val.replace(/&[a-z]+;/g, '');
      else nodes[i].innerHTML = val;
    }
    var titled = document.querySelectorAll('[data-i18n-title]');
    for (i = 0; i < titled.length; i++) {
      var tv = dict[titled[i].getAttribute('data-i18n-title')];
      if (tv) titled[i].title = tv.replace(/<[^>]+>/g, '');
    }
    $('symbolList').innerHTML = dict['formula.legend'];
    buildPresets();
    updatePlayLabel();
    updateReadouts(true);
    store('lang', lang);
  }

  // --------------------------------------------------------------- Presets --
  var PRESETS = [
    { key: 'preset.balanced',     q: [10, 1, 100, 10],     R: 0.5,   N: 40, terminal: true },
    { key: 'preset.tight',        q: [80, 6, 140, 12],     R: 0.25,  N: 40, terminal: true },
    { key: 'preset.gentle',       q: [4, 1, 60, 6],        R: 12,    N: 40, terminal: true },
    { key: 'preset.angleOnly',    q: [0.05, 0.05, 200, 8], R: 0.5,   N: 40, terminal: true },
    { key: 'preset.aggressive',   q: [200, 10, 500, 25],   R: 0.002, N: 40, terminal: true },
    { key: 'preset.shortsighted', q: [10, 1, 100, 10],     R: 0.5,   N: 8,  terminal: false }
  ];
  var activePreset = 0;

  function buildPresets() {
    var dict = IPM.i18n[lang];
    var box = $('presets');
    box.innerHTML = '';
    PRESETS.forEach(function (p, idx) {
      var b = document.createElement('button');
      b.className = 'btn' + (idx === activePreset ? ' active' : '');
      b.innerHTML = dict[p.key];
      b.addEventListener('click', function () { applyPreset(idx); });
      box.appendChild(b);
    });
  }

  function applyPreset(idx) {
    var p = PRESETS[idx];
    activePreset = idx;
    ctrl.configure({ q: p.q.slice(), R: p.R, N: p.N, terminal: p.terminal });
    syncControlsFromCtrl();
    buildPresets();
  }

  // ----------------------------------------------------------- UI-Anbindung --
  function fmt(v, d) { return v.toFixed(d === undefined ? 2 : d); }

  function syncControlsFromCtrl() {
    var c = ctrl.cfg;
    for (var i = 0; i < 4; i++) $('q' + i).value = String(c.q[i]);
    $('rin').value = String(c.R);
    $('Nin').value = String(c.N);
    $('NVal').textContent = c.N + '  (' + fmt(c.N * c.Ts, 2) + ' s)';
    $('Tsin').value = String(Math.round(c.Ts * 1000));
    $('TsVal').textContent = Math.round(c.Ts * 1000) + ' ms  (' + fmt(1 / c.Ts, 0) + ' Hz)';
    $('umaxin').value = String(c.umax);
    $('umaxVal').textContent = fmt(c.umax, 1) + ' N';
    $('termChk').checked = !!c.terminal;
    $('modeSel').value = c.mode;
    scopeScales.u = Math.max(5, c.umax);
  }

  function syncPlantControls() {
    $('mpin').value = String(plant.mp);   $('mpVal').textContent = fmt(plant.mp, 2) + ' kg';
    $('Mcin').value = String(plant.Mc);   $('McVal').textContent = fmt(plant.Mc, 2) + ' kg';
    $('lin').value = String(plant.l);     $('lVal').textContent = fmt(plant.l, 2) + ' m';
    $('bin').value = String(plant.b);     $('bVal').textContent = fmt(plant.b, 2) + ' Ns/m';
    $('distin').value = String(opts.distStrength);
    $('distVal').textContent = fmt(opts.distStrength, 2) + '×';
    $('noisein').value = String(Math.round(opts.noise * 1000));
    $('noiseVal').textContent = opts.noise === 0 ? '0' : fmt(opts.noise * 1000, 1) + ' mm / m°';
    $('mmin').value = String(Math.round(opts.mismatch * 100));
    $('mmVal').textContent = (opts.mismatch >= 0 ? '+' : '') + Math.round(opts.mismatch * 100) + ' %';
    $('spin').value = String(Math.round(sim.xref[0] * 100));
    $('spVal').textContent = fmt(sim.xref[0], 2) + ' m';
  }

  /** Reglermodell aktualisieren (inkl. absichtlichem Modellfehler). */
  function pushPlantToCtrl() {
    ctrl.configure({
      plant: {
        Mc: plant.Mc,
        mp: plant.mp * (1 + opts.mismatch),
        l: plant.l,
        g: plant.g,
        b: plant.b
      }
    });
  }

  function readQR() {
    var q = [], ok = true;
    for (var i = 0; i < 4; i++) {
      var el = $('q' + i);
      var v = parseFloat(el.value.replace(',', '.'));
      var bad = !isFinite(v) || v < 0;
      el.classList.toggle('invalid', bad);
      if (bad) { ok = false; q.push(ctrl.cfg.q[i]); } else q.push(v);
    }
    var rel = $('rin');
    var r = parseFloat(rel.value.replace(',', '.'));
    var rbad = !isFinite(r) || r <= 0;
    rel.classList.toggle('invalid', rbad);
    if (rbad) { r = ctrl.cfg.R; ok = false; }
    ctrl.configure({ q: q, R: r });
    activePreset = -1;
    buildPresets();
    return ok;
  }

  function bindUI() {
    for (var i = 0; i < 4; i++) $('q' + i).addEventListener('input', readQR);
    $('rin').addEventListener('input', readQR);

    $('Nin').addEventListener('input', function () {
      ctrl.configure({ N: parseInt(this.value, 10) });
      syncControlsFromCtrl();
    });
    $('Tsin').addEventListener('input', function () {
      ctrl.configure({ Ts: parseInt(this.value, 10) / 1000 });
      syncControlsFromCtrl();
    });
    $('umaxin').addEventListener('input', function () {
      ctrl.configure({ umax: parseFloat(this.value) });
      syncControlsFromCtrl();
    });
    $('termChk').addEventListener('change', function () {
      ctrl.configure({ terminal: this.checked });
    });
    $('modeSel').addEventListener('change', function () {
      ctrl.configure({ mode: this.value });
    });

    $('mpin').addEventListener('input', function () {
      plant.mp = parseFloat(this.value); pushPlantToCtrl(); syncPlantControls();
    });
    $('Mcin').addEventListener('input', function () {
      plant.Mc = parseFloat(this.value); pushPlantToCtrl(); syncPlantControls();
    });
    $('lin').addEventListener('input', function () {
      plant.l = parseFloat(this.value); pushPlantToCtrl(); syncPlantControls();
    });
    $('bin').addEventListener('input', function () {
      plant.b = parseFloat(this.value); pushPlantToCtrl(); syncPlantControls();
    });
    $('distin').addEventListener('input', function () {
      opts.distStrength = parseFloat(this.value); syncPlantControls();
    });
    $('noisein').addEventListener('input', function () {
      opts.noise = parseFloat(this.value) / 1000; syncPlantControls();
    });
    $('mmin').addEventListener('input', function () {
      opts.mismatch = parseFloat(this.value) / 100; pushPlantToCtrl(); syncPlantControls();
    });
    $('spin').addEventListener('input', function () {
      sim.xref[0] = parseFloat(this.value) / 100; syncPlantControls();
    });

    $('showPred').addEventListener('change', function () { opts.showPred = this.checked; });
    $('showForces').addEventListener('change', function () { opts.showForces = this.checked; });
    $('showTrace').addEventListener('change', function () {
      opts.showTrace = this.checked; if (!this.checked) trace.length = 0;
    });

    $('playBtn').addEventListener('click', function () { sim.running = !sim.running; updatePlayLabel(); });
    $('resetBtn').addEventListener('click', function () { reset(0.08); });
    $('dropBtn').addEventListener('click', function () { reset(0.49 * (Math.random() < 0.5 ? -1 : 1)); });
    $('kickLeftBtn').addEventListener('click', function () { kick(-1); });
    $('kickRightBtn').addEventListener('click', function () { kick(1); });

    $('langBtn').addEventListener('click', function () { applyI18n(lang === 'de' ? 'en' : 'de'); });
    $('themeBtn').addEventListener('click', function () {
      var next = document.documentElement.getAttribute('data-theme') === 'light' ? 'dark' : 'light';
      setTheme(next);
    });

    document.addEventListener('keydown', function (e) {
      if (/^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;
      if (e.code === 'Space') { e.preventDefault(); sim.running = !sim.running; updatePlayLabel(); }
      else if (e.key === 'r' || e.key === 'R') reset(0.08);
      else if (e.key === 'ArrowLeft') { e.preventDefault(); kick(-1); }
      else if (e.key === 'ArrowRight') { e.preventDefault(); kick(1); }
    });

    var scene = $('scene');
    var leave = function () { mouse.inside = false; mouse.active = false; mouse.vx = 0; };
    scene.addEventListener('pointermove', onPointerMove);
    scene.addEventListener('pointerleave', leave);
    scene.addEventListener('pointerup', function (e) { if (e.pointerType === 'touch') leave(); });
    scene.addEventListener('pointercancel', leave);
    scene.addEventListener('pointerdown', function (e) {
      if (!lastTf) return;
      // Ohne Hover (Touch/Stift) ist das Ziehen die Stoerung; die Sollposition
      // wird dort nur ueber den Schieberegler gesetzt.
      if (e.pointerType === 'touch' || e.pointerType === 'pen') { onPointerMove(e); return; }
      var pt = toWorld(e);
      sim.xref[0] = clamp(pt.x, -1.5, 1.5);
      syncPlantControls();
    });
  }

  function updatePlayLabel() {
    var dict = IPM.i18n[lang];
    $('playBtn').innerHTML = sim.running ? dict['ctrl.pause'] : dict['ctrl.start'];
  }

  function setTheme(t) {
    document.documentElement.setAttribute('data-theme', t);
    store('theme', t);
    R.readColors();
  }

  function reset(theta0) {
    sim.s = [0, 0, theta0, 0];
    sim.t = 0; sim.u = 0; sim.fd = 0; sim.fdKick = 0; sim.kickTimer = 0;
    ctrl.reset();
    hist.t.length = hist.th.length = hist.p.length = hist.u.length = 0;
    trace.length = 0;
    camX = 0;
    lastRes = null;
  }

  function kick(dir) {
    // Kurzer Impuls, ebenfalls an der Gewichtskraft der Kugel bemessen.
    sim.fdKick = dir * 2.5 * plant.mp * plant.g;
    sim.kickTimer = 0.08;
  }

  // ------------------------------------------------------------ Mausstoerung --
  function toWorld(e) {
    var rect = $('scene').getBoundingClientRect();
    var px = e.clientX - rect.left, py = e.clientY - rect.top;
    var tf = lastTf;
    return {
      x: tf.camX + (px - rect.width / 2) / tf.scale,
      y: (tf.groundY - tf.cartOffset - py) / tf.scale
    };
  }

  var lastPointerT = 0;
  function onPointerMove(e) {
    if (!lastTf) return;
    var now = performance.now();
    var pt = toWorld(e);
    var dt = (now - lastPointerT) / 1000;
    if (dt > 0 && dt < 0.2) {
      var v = (pt.x - mouse.x) / dt;
      mouse.vx = clamp(0.6 * mouse.vx + 0.4 * v, -12, 12);
    }
    lastPointerT = now;
    mouse.x = pt.x; mouse.y = pt.y; mouse.inside = true;
  }

  /** Horizontale Kraft der Maus auf die Pendelmasse bestimmen. */
  function updateMouseForce(dt) {
    mouse.vx *= Math.exp(-dt / 0.09);
    if (!mouse.inside || opts.distStrength <= 0) { mouse.active = false; sim.fdMouse = 0; return; }
    var bx = sim.s[0] + plant.l * Math.sin(sim.s[2]);
    var by = plant.l * Math.cos(sim.s[2]);
    var d = Math.hypot(mouse.x - bx, mouse.y - by);
    if (d > MOUSE_RADIUS) { mouse.active = false; sim.fdMouse = 0; return; }
    mouse.active = true;
    // Abstossung vom Zeiger: Richtung bleibt auch dicht an der Kugel erhalten,
    // damit ruhiges Danebenhalten wirkt und nicht nur schnelles Wischen.
    // Bezugsgroesse ist die Gewichtskraft der Kugel - eine Dauerkraft von rund
    // der Haelfte davon ist spuerbar, laesst aber noch eine Ruhelage zu.
    var W = plant.mp * plant.g;
    var dx = bx - mouse.x;
    var wgt = 1 - d / MOUSE_RADIUS;
    var push = 0.55 * W * dx / Math.max(0.05, Math.abs(dx));
    var drag = 0.35 * W * mouse.vx;                  // Mitnahme durch die Bewegung
    sim.fdMouse = clamp(opts.distStrength * wgt * (push + drag), -4 * W, 4 * W);
  }

  // ---------------------------------------------------------------- Regelung --
  function gauss() {
    var u1 = Math.random() || 1e-9, u2 = Math.random();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  }

  function runController() {
    var meas = sim.s.slice();
    meas[2] = wrapPi(meas[2]);
    if (opts.noise > 0) {
      meas[0] += opts.noise * gauss();
      meas[2] += opts.noise * 0.35 * gauss();
    }
    lastRes = ctrl.step(meas, sim.xref);
    sim.u = lastRes.u;
    msAvg += (lastRes.ms - msAvg) * 0.1;
  }

  // ----------------------------------------------------------------- Schleife --
  var acc = 0, subCount = 0, lastFrame = 0, frameCount = 0;

  function stepSim(dt) {
    var Ts = ctrl.cfg.Ts;
    var h = Ts / SUB;
    acc += dt;
    var guard = 0;
    while (acc >= h && guard++ < 600) {
      if (subCount % SUB === 0) runController();
      subCount++;

      updateMouseForce(h);
      if (sim.kickTimer > 0) { sim.kickTimer -= h; if (sim.kickTimer <= 0) sim.fdKick = 0; }
      sim.fd = sim.fdMouse + sim.fdKick;

      sim.s = M.rk4(sim.s, sim.u, sim.fd, plant, h);
      sim.s[2] = wrapPi(sim.s[2]);
      sim.t += h;
      acc -= h;

      if (subCount % SUB === 0) {
        hist.t.push(sim.t);
        hist.th.push(sim.s[2] * 180 / Math.PI);
        hist.p.push(sim.s[0]);
        hist.u.push(sim.u);
        if (hist.t.length > 1400) {
          hist.t.splice(0, 400); hist.th.splice(0, 400);
          hist.p.splice(0, 400); hist.u.splice(0, 400);
        }
        if (opts.showTrace) {
          trace.push({ x: sim.s[0] + plant.l * Math.sin(sim.s[2]), y: plant.l * Math.cos(sim.s[2]) });
          if (trace.length > 260) trace.shift();
        }
      }
    }
    if (guard >= 600) acc = 0;   // nach langer Pause nicht aufholen
  }

  function updateScopeScales() {
    var i, maxTh = 8, maxP = 0.4;
    for (i = Math.max(0, hist.t.length - 700); i < hist.t.length; i++) {
      var a = Math.abs(hist.th[i]); if (a > maxTh) maxTh = a;
      var b = Math.abs(hist.p[i]); if (b > maxP) maxP = b;
    }
    scopeScales.th += (Math.min(180, maxTh * 1.15) - scopeScales.th) * 0.06;
    scopeScales.p += (maxP * 1.15 - scopeScales.p) * 0.06;
    scopeScales.u = Math.max(1, ctrl.cfg.umax);
  }

  function updateReadouts(force) {
    if (!force && (frameCount % 4) !== 0) return;
    var dict = IPM.i18n[lang];
    $('hudTheta').textContent = (sim.s[2] * 180 / Math.PI).toFixed(1) + '°';
    $('hudPos').textContent = sim.s[0].toFixed(3) + ' m';
    $('hudU').textContent = sim.u.toFixed(2) + ' N';
    $('hudFd').textContent = sim.fd.toFixed(1) + ' N';
    if (lastRes) {
      $('hudJ').textContent = lastRes.cost < 1e5 ? lastRes.cost.toFixed(1) : lastRes.cost.toExponential(2);
      $('hudMs').textContent = msAvg.toFixed(3) + ' ms';
      $('hudIter').textContent = String(lastRes.sweeps);
      var st = $('hudStatus');
      if (lastRes.saturated) { st.textContent = '⚠ ' + strip(dict['hud.sat']); st.className = 'hud-row status sat'; }
      else if (lastRes.exact) { st.textContent = '✓ ' + strip(dict['hud.exact']); st.className = 'hud-row status exact'; }
      else { st.textContent = ''; st.className = 'hud-row status'; }
    }
  }
  function strip(s) { return String(s).replace(/<[^>]+>/g, ''); }

  function draw() {
    // Kamera folgt dem Wagen mit Totzone
    var dead = 0.55;
    var target = 0;
    if (sim.s[0] > camX + dead) target = sim.s[0] - dead;
    else if (sim.s[0] < camX - dead) target = sim.s[0] + dead;
    else target = camX;
    camX += (target - camX) * 0.08;

    lastTf = R.drawScene($('scene'), {
      state: sim.s, plant: plant, u: sim.u, fd: sim.fd, xref: sim.xref,
      umax: ctrl.cfg.umax, camX: camX, mouse: mouse,
      pred: lastRes ? lastRes.xpred : null,
      showPred: opts.showPred, showForces: opts.showForces,
      trace: opts.showTrace ? trace : null
    });

    updateScopeScales();
    R.drawScope($('scope'), hist, sim.t, SPAN, scopeScales);
    R.drawUPlan($('uplan'), lastRes ? lastRes.useq : null, ctrl.cfg.umax, ctrl.cfg.Ts);
    updateReadouts(false);
  }

  function frame(now) {
    root.requestAnimationFrame(frame);
    var dt = (now - lastFrame) / 1000;
    lastFrame = now;
    if (!isFinite(dt) || dt < 0) dt = 0;
    if (dt > 0.25) dt = 0.25;
    frameCount++;
    if (sim.running) stepSim(dt);
    else updateMouseForce(dt);
    draw();
  }

  // -------------------------------------------------------------------- Start --
  function init() {
    els.scene = $('scene');
    PLACEHOLDERS.MODEL_EQ = $('eq-model').innerHTML;
    PLACEHOLDERS.QP_EQ = $('eq-qp').innerHTML;

    // URL-Parameter haben Vorrang - dadurch laesst sich die Seite eingebettet
    // (z. B. als iframe) gezielt konfigurieren: ?lang=en&theme=light&embed=1
    var qs = {};
    try {
      new URLSearchParams(root.location.search).forEach(function (v, k) { qs[k] = v; });
    } catch (e) { /* aeltere Browser: Parameter werden ignoriert */ }

    if (qs.embed === '1' || qs.embed === 'true') document.body.classList.add('embed');

    var savedTheme = qs.theme || load('theme');
    if (savedTheme !== 'light' && savedTheme !== 'dark') {
      savedTheme = (root.matchMedia && root.matchMedia('(prefers-color-scheme: light)').matches)
        ? 'light' : 'dark';
    }
    setTheme(savedTheme);

    var savedLang = qs.lang || load('lang');
    if (savedLang !== 'de' && savedLang !== 'en') {
      savedLang = (navigator.language || 'de').slice(0, 2) === 'de' ? 'de' : 'en';
    }

    bindUI();
    applyPreset(0);
    pushPlantToCtrl();
    syncPlantControls();
    applyI18n(savedLang);

    root.addEventListener('resize', function () { R.readColors(); });

    // Zugriffspunkt fuer die Konsole und fuer eine Einbettung in andere Seiten.
    IPM.app = {
      sim: sim, ctrl: ctrl, opts: opts, plant: plant, hist: hist, mouse: mouse,
      reset: reset, kick: kick, applyPreset: applyPreset, setLang: applyI18n,
      setTheme: setTheme, result: function () { return lastRes; }
    };

    lastFrame = performance.now();
    root.requestAnimationFrame(frame);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})(typeof window !== 'undefined' ? window : globalThis);
