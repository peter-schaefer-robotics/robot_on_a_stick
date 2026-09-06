/*
 * Application logic: simulation loop, controls, disturbances, mode switching.
 *
 * Timing:
 *   - simulation: fixed step h = Ts/8, Runge-Kutta 4
 *   - controller: every Ts (that is, every 8th simulation step)
 *   - display:    requestAnimationFrame, decoupled from the simulation
 *
 * Two modes. Simple mode hides everything that is not needed to understand the
 * idea, and resets those settings to their defaults. Because it also drops the
 * terminal weight P, it compensates with a longer horizon (N = 100, i.e. two
 * seconds of look-ahead) - without either of the two, the cart drifts.
 */
(function (root) {
  'use strict';
  var IPM = root.IPM;
  var M = IPM.model;
  var R = IPM.render;

  var SUB = 8;                 // simulation steps per control step
  var SPAN = 12;               // scope window [s]
  var MOUSE_RADIUS = 0.32;     // radius of influence of the pointer [m]
  var LS = 'ipm-mpc:';

  var MODE_DEFAULTS = {
    simple: { N: 100, terminal: false, showForces: false },
    expert: { N: 40, terminal: true, showForces: true }
  };
  var COMMON_DEFAULTS = { Ts: 0.02, umax: 15, mode: 'linear' };

  // ------------------------------------------------------------------ state --
  var sim = {
    s: [0, 0, 0.08, 0],
    t: 0, u: 0, fd: 0, fdMouse: 0, fdKick: 0, kickTimer: 0,
    running: true,
    xref: [0, 0, 0, 0]
  };

  var opts = {
    noise: 0, mismatch: 0, distStrength: 1,
    showPred: true, showForces: false, showTrace: false
  };

  var plant = Object.assign({}, M.DEFAULT_PLANT);
  var ctrl = new IPM.MpcController({ plant: plant });
  var lastRes = null;
  // Browsers round performance.now() to about 100 us; the running average keeps
  // the displayed solve time meaningful anyway.
  var msAvg = 0;

  var hist = { t: [], th: [], p: [], u: [] };
  var scopeScales = { th: 15, p: 1, u: 15 };
  var trace = [];
  var camX = 0;
  var lastTf = null;
  var mouse = { x: 0, y: 0, vx: 0, active: false, radius: MOUSE_RADIUS, inside: false };

  var uiMode = 'simple';
  var pAuto = true;
  var pCells = [];

  function $(id) { return document.getElementById(id); }
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function wrapPi(a) {
    while (a > Math.PI) a -= 2 * Math.PI;
    while (a < -Math.PI) a += 2 * Math.PI;
    return a;
  }
  function store(k, v) { try { localStorage.setItem(LS + k, v); } catch (e) { /* ignored */ } }
  function load(k) { try { return localStorage.getItem(LS + k); } catch (e) { return null; } }
  function fmt(v, d) { return v.toFixed(d === undefined ? 2 : d); }

  /** Compact formatting for the terminal-weight cells. */
  function fmtP(v) {
    var a = Math.abs(v);
    if (a < 5e-4) return '0';
    if (a >= 100) return v.toFixed(0);
    if (a >= 10) return v.toFixed(1);
    if (a >= 1) return v.toFixed(2);
    if (a >= 0.01) return v.toFixed(3);
    return v.toExponential(1);
  }

  // ------------------------------------------------------------ mode switch --
  function setMode(m) {
    uiMode = m;
    document.body.setAttribute('data-mode', m);
    var d = MODE_DEFAULTS[m];

    // Everything hidden in this mode goes back to its default value.
    pAuto = true;
    ctrl.configure({
      N: d.N, terminal: d.terminal, Ts: COMMON_DEFAULTS.Ts,
      umax: COMMON_DEFAULTS.umax, mode: COMMON_DEFAULTS.mode, Pdiag: null
    });
    Object.assign(plant, M.DEFAULT_PLANT);
    opts.noise = 0;
    opts.mismatch = 0;
    opts.distStrength = 1;
    opts.showForces = d.showForces;
    opts.showTrace = false;
    sim.xref[0] = 0;
    trace.length = 0;

    pushPlantToCtrl();
    syncControls();
    buildPGrid();
    updateModeButton();

    $('modeBtn').blur();
  }

  function updateModeButton() {
    var expert = uiMode === 'expert';
    $('modeBtn').querySelector('.mode-btn-label').textContent =
      expert ? 'Simple mode' : 'Expert mode';
    $('modeBtn').querySelector('.mode-btn-sub').textContent = expert
      ? 'back to the essentials — hides the details and restores the defaults'
      : 'show the model, all tuning parameters and diagnostics';
  }

  function updatePTermVisibility() {
    document.body.classList.toggle('no-pterm', !ctrl.cfg.terminal);
  }

  // ------------------------------------------------------- terminal weight --
  /** Rebuild the 4x4 grid of P: read-only Riccati values or editable diagonal. */
  function buildPGrid() {
    var grid = $('pGrid');
    grid.innerHTML = '';
    pCells = [];
    for (var i = 0; i < 4; i++) {
      for (var j = 0; j < 4; j++) {
        var el;
        if (pAuto) {
          el = document.createElement('span');
          el.className = 'cell ro';
          el.textContent = '–';
        } else if (i === j) {
          el = document.createElement('input');
          el.className = 'cell in';
          el.type = 'number';
          el.step = 'any';
          el.inputMode = 'decimal';
          el.value = String(currentPDiag()[i]);
          el.addEventListener('input', readPDiag);
        } else {
          el = document.createElement('span');
          el.className = 'cell z';
          el.textContent = '0';
        }
        grid.appendChild(el);
        pCells.push(el);
      }
    }
    if (pAuto && ctrl.cfg.terminal && !ctrl.Pterm) ctrl.rebuildTerminal();
    updatePCells(true);
    updatePTermVisibility();
  }

  function currentPDiag() {
    if (ctrl.cfg.Pdiag) return ctrl.cfg.Pdiag.slice();
    // P is normally computed on the next control step; force it now so the
    // fields show the Riccati values right away.
    if (!ctrl.Pterm && ctrl.cfg.terminal) ctrl.rebuildTerminal();
    var P = ctrl.Pterm;
    if (P) return [P[0][0], P[1][1], P[2][2], P[3][3]];
    return ctrl.cfg.q.slice();
  }

  function readPDiag() {
    var d = [], ok = true;
    for (var i = 0; i < 4; i++) {
      var el = pCells[i * 4 + i];
      var v = parseFloat(String(el.value).replace(',', '.'));
      var bad = !isFinite(v) || v < 0;
      el.classList.toggle('invalid', bad);
      if (bad) { ok = false; d.push(ctrl.cfg.Pdiag ? ctrl.cfg.Pdiag[i] : 0); } else d.push(v);
    }
    ctrl.configure({ Pdiag: d });
    return ok;
  }

  /** Refresh the displayed Riccati values (they change with Q, R, Ts, plant). */
  function updatePCells(force) {
    if (!pAuto || !ctrl.cfg.terminal) return;
    var P = ctrl.Pterm;
    if (!P) {
      if (force) for (var k = 0; k < 16; k++) pCells[k].textContent = '–';
      return;
    }
    for (var i = 0; i < 4; i++) {
      for (var j = 0; j < 4; j++) pCells[i * 4 + j].textContent = fmtP(P[i][j]);
    }
  }

  // ----------------------------------------------------------------- presets --
  function bindPresets() {
    var btns = document.querySelectorAll('.preset');
    Array.prototype.forEach.call(btns, function (b) {
      b.addEventListener('click', function () {
        var q = b.getAttribute('data-q').split(',').map(Number);
        var r = parseFloat(b.getAttribute('data-r'));
        var cfg = { q: q, R: r };
        if (b.hasAttribute('data-n')) cfg.N = parseInt(b.getAttribute('data-n'), 10);
        if (b.hasAttribute('data-terminal')) cfg.terminal = b.getAttribute('data-terminal') === '1';
        ctrl.configure(cfg);
        markPreset(b);
        syncControls();
      });
    });
  }

  function markPreset(active) {
    var btns = document.querySelectorAll('.preset');
    Array.prototype.forEach.call(btns, function (b) { b.classList.toggle('active', b === active); });
  }

  function matchPreset() {
    var btns = document.querySelectorAll('.preset');
    var found = null;
    Array.prototype.forEach.call(btns, function (b) {
      if (found) return;
      var q = b.getAttribute('data-q').split(',').map(Number);
      var r = parseFloat(b.getAttribute('data-r'));
      var same = Math.abs(r - ctrl.cfg.R) < 1e-12;
      for (var i = 0; i < 4 && same; i++) same = Math.abs(q[i] - ctrl.cfg.q[i]) < 1e-12;
      if (same) found = b;
    });
    markPreset(found);
  }

  // ------------------------------------------------------------- UI binding --
  function syncControls() {
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
    $('pAuto').checked = pAuto;
    scopeScales.u = Math.max(5, c.umax);

    $('mpin').value = String(plant.mp);   $('mpVal').textContent = fmt(plant.mp, 2) + ' kg';
    $('Mcin').value = String(plant.Mc);   $('McVal').textContent = fmt(plant.Mc, 2) + ' kg';
    $('lin').value = String(plant.l);     $('lVal').textContent = fmt(plant.l, 2) + ' m';
    $('bin').value = String(plant.b);     $('bVal').textContent = fmt(plant.b, 2) + ' Ns/m';
    $('distin').value = String(opts.distStrength);
    $('distVal').textContent = fmt(opts.distStrength, 2) + '×';
    $('noisein').value = String(Math.round(opts.noise * 1000));
    $('noiseVal').textContent = opts.noise === 0 ? 'off' : fmt(opts.noise * 1000, 1) + ' mm';
    $('mmin').value = String(Math.round(opts.mismatch * 100));
    $('mmVal').textContent = (opts.mismatch >= 0 ? '+' : '') + Math.round(opts.mismatch * 100) + ' %';
    $('spin').value = String(Math.round(sim.xref[0] * 100));
    $('spVal').textContent = fmt(sim.xref[0], 2) + ' m';
    $('showPred').checked = opts.showPred;
    $('showForces').checked = opts.showForces;
    $('showTrace').checked = opts.showTrace;

    updatePTermVisibility();
    matchPreset();
  }

  /** Push the plant parameters to the controller, including deliberate error. */
  function pushPlantToCtrl() {
    ctrl.configure({
      plant: {
        Mc: plant.Mc, mp: plant.mp * (1 + opts.mismatch),
        l: plant.l, g: plant.g, b: plant.b
      }
    });
  }

  function readQR() {
    var q = [], ok = true, i;
    for (i = 0; i < 4; i++) {
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
    matchPreset();
    return ok;
  }

  function bindUI() {
    for (var i = 0; i < 4; i++) $('q' + i).addEventListener('input', readQR);
    $('rin').addEventListener('input', readQR);

    $('pAuto').addEventListener('change', function () {
      pAuto = this.checked;
      ctrl.configure({ Pdiag: pAuto ? null : currentPDiag() });
      buildPGrid();
    });

    $('Nin').addEventListener('input', function () {
      ctrl.configure({ N: parseInt(this.value, 10) }); syncControls();
    });
    $('Tsin').addEventListener('input', function () {
      ctrl.configure({ Ts: parseInt(this.value, 10) / 1000 }); syncControls();
    });
    $('umaxin').addEventListener('input', function () {
      ctrl.configure({ umax: parseFloat(this.value) }); syncControls();
    });
    $('termChk').addEventListener('change', function () {
      ctrl.configure({ terminal: this.checked });
      updatePTermVisibility();
      updatePCells(true);
    });
    $('modeSel').addEventListener('change', function () { ctrl.configure({ mode: this.value }); });

    $('mpin').addEventListener('input', function () {
      plant.mp = parseFloat(this.value); pushPlantToCtrl(); syncControls();
    });
    $('Mcin').addEventListener('input', function () {
      plant.Mc = parseFloat(this.value); pushPlantToCtrl(); syncControls();
    });
    $('lin').addEventListener('input', function () {
      plant.l = parseFloat(this.value); pushPlantToCtrl(); syncControls();
    });
    $('bin').addEventListener('input', function () {
      plant.b = parseFloat(this.value); pushPlantToCtrl(); syncControls();
    });
    $('distin').addEventListener('input', function () {
      opts.distStrength = parseFloat(this.value); syncControls();
    });
    $('noisein').addEventListener('input', function () {
      opts.noise = parseFloat(this.value) / 1000; syncControls();
    });
    $('mmin').addEventListener('input', function () {
      opts.mismatch = parseFloat(this.value) / 100; pushPlantToCtrl(); syncControls();
    });
    $('spin').addEventListener('input', function () {
      sim.xref[0] = parseFloat(this.value) / 100; syncControls();
    });

    $('showPred').addEventListener('change', function () { opts.showPred = this.checked; });
    $('showForces').addEventListener('change', function () { opts.showForces = this.checked; });
    $('showTrace').addEventListener('change', function () {
      opts.showTrace = this.checked; if (!this.checked) trace.length = 0;
    });

    $('playBtn').addEventListener('click', function () { sim.running = !sim.running; updatePlayLabel(); });
    $('resetBtn').addEventListener('click', function () { reset(0.08); });
    $('kickLeftBtn').addEventListener('click', function () { kick(-1); });
    $('kickRightBtn').addEventListener('click', function () { kick(1); });
    $('modeBtn').addEventListener('click', function () {
      setMode(uiMode === 'simple' ? 'expert' : 'simple');
    });

    $('themeBtn').addEventListener('click', function () {
      setTheme(document.documentElement.getAttribute('data-theme') === 'light' ? 'dark' : 'light');
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
      // Without hover (touch, pen) dragging is the disturbance; the target
      // position is set with the slider there.
      if (e.pointerType === 'touch' || e.pointerType === 'pen') { onPointerMove(e); return; }
      sim.xref[0] = clamp(toWorld(e).x, -1.5, 1.5);
      syncControls();
    });
  }

  function updatePlayLabel() { $('playBtn').textContent = sim.running ? 'Pause' : 'Start'; }

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
    msAvg = 0;
  }

  function kick(dir) {
    // Short impulse, scaled by the weight of the ball.
    sim.fdKick = dir * 2.5 * plant.mp * plant.g;
    sim.kickTimer = 0.08;
  }

  // ------------------------------------------------------ mouse disturbance --
  function toWorld(e) {
    var rect = $('scene').getBoundingClientRect();
    var px = e.clientX - rect.left, py = e.clientY - rect.top;
    return {
      x: lastTf.camX + (px - rect.width / 2) / lastTf.scale,
      y: (lastTf.groundY - lastTf.cartOffset - py) / lastTf.scale
    };
  }

  var lastPointerT = 0;
  function onPointerMove(e) {
    if (!lastTf) return;
    var now = performance.now();
    var pt = toWorld(e);
    var dt = (now - lastPointerT) / 1000;
    if (dt > 0 && dt < 0.2) {
      mouse.vx = clamp(0.6 * mouse.vx + 0.4 * ((pt.x - mouse.x) / dt), -12, 12);
    }
    lastPointerT = now;
    mouse.x = pt.x; mouse.y = pt.y; mouse.inside = true;
  }

  function updateMouseForce(dt) {
    mouse.vx *= Math.exp(-dt / 0.09);
    if (!mouse.inside || opts.distStrength <= 0) { mouse.active = false; sim.fdMouse = 0; return; }
    var bx = sim.s[0] + plant.l * Math.sin(sim.s[2]);
    var by = plant.l * Math.cos(sim.s[2]);
    var d = Math.hypot(mouse.x - bx, mouse.y - by);
    if (d > MOUSE_RADIUS) { mouse.active = false; sim.fdMouse = 0; return; }
    mouse.active = true;
    // Repulsion from the pointer. The direction survives right at the ball, so
    // holding the pointer still also has an effect, not just fast swipes.
    // Reference is the weight of the ball: a steady push of about half of it is
    // clearly felt but still leaves an equilibrium to settle into.
    var W = plant.mp * plant.g;
    var dx = bx - mouse.x;
    var wgt = 1 - d / MOUSE_RADIUS;
    var push = 0.55 * W * dx / Math.max(0.05, Math.abs(dx));
    var drag = 0.35 * W * mouse.vx;
    sim.fdMouse = clamp(opts.distStrength * wgt * (push + drag), -4 * W, 4 * W);
  }

  // ------------------------------------------------------------- controller --
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

  // ------------------------------------------------------------------- loop --
  var acc = 0, subCount = 0, lastFrame = 0, frameCount = 0;

  function stepSim(dt) {
    var h = ctrl.cfg.Ts / SUB;
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
    if (guard >= 600) acc = 0;   // do not try to catch up after a long pause
  }

  function updateScopeScales() {
    var maxTh = 8, maxP = 0.4;
    for (var i = Math.max(0, hist.t.length - 700); i < hist.t.length; i++) {
      var a = Math.abs(hist.th[i]); if (a > maxTh) maxTh = a;
      var b = Math.abs(hist.p[i]); if (b > maxP) maxP = b;
    }
    scopeScales.th += (Math.min(180, maxTh * 1.15) - scopeScales.th) * 0.06;
    scopeScales.p += (maxP * 1.15 - scopeScales.p) * 0.06;
    scopeScales.u = Math.max(1, ctrl.cfg.umax);
  }

  function updateReadouts(force) {
    if (!force && (frameCount % 4) !== 0) return;
    $('hudTheta').textContent = (sim.s[2] * 180 / Math.PI).toFixed(1) + '°';
    $('hudU').textContent = sim.u.toFixed(2) + ' N';
    $('hudFd').textContent = sim.fd.toFixed(2) + ' N';
    if (uiMode !== 'expert') return;
    $('hudPos').textContent = sim.s[0].toFixed(3) + ' m';
    if (lastRes) {
      $('hudJ').textContent = lastRes.cost < 1e5 ? lastRes.cost.toFixed(1) : lastRes.cost.toExponential(2);
      $('hudMs').textContent = msAvg.toFixed(3) + ' ms';
      $('hudIter').textContent = String(lastRes.sweeps);
      var st = $('hudStatus');
      if (lastRes.saturated) { st.textContent = '⚠ input bound active'; st.className = 'hud-row status sat'; }
      else if (lastRes.exact) { st.textContent = '✓ exact (bounds inactive)'; st.className = 'hud-row status exact'; }
      else { st.textContent = ''; st.className = 'hud-row status'; }
    }
    if (frameCount % 10 === 0) updatePCells(false);
  }

  function draw() {
    // camera follows the cart with a dead zone
    var dead = 0.55, target = camX;
    if (sim.s[0] > camX + dead) target = sim.s[0] - dead;
    else if (sim.s[0] < camX - dead) target = sim.s[0] + dead;
    camX += (target - camX) * 0.08;

    lastTf = R.drawScene($('scene'), {
      state: sim.s, plant: plant, u: sim.u, fd: sim.fd, xref: sim.xref,
      umax: ctrl.cfg.umax, camX: camX, mouse: mouse,
      pred: lastRes ? lastRes.xpred : null,
      showPred: opts.showPred, showForces: opts.showForces,
      trace: opts.showTrace ? trace : null
    });

    if (uiMode === 'expert') {
      updateScopeScales();
      R.drawScope($('scope'), hist, sim.t, SPAN, scopeScales);
      R.drawUPlan($('uplan'), lastRes ? lastRes.useq : null, ctrl.cfg.umax, ctrl.cfg.Ts);
    }
    updateReadouts(false);
  }

  function frame(now) {
    root.requestAnimationFrame(frame);
    var dt = (now - lastFrame) / 1000;
    lastFrame = now;
    if (!isFinite(dt) || dt < 0) dt = 0;
    if (dt > 0.25) dt = 0.25;
    frameCount++;
    if (sim.running) stepSim(dt); else updateMouseForce(dt);
    draw();
  }

  // ------------------------------------------------------------------ start --
  function init() {
    // URL parameters allow an embedded page to be configured directly:
    // ?mode=expert&theme=light&embed=1
    var qs = {};
    try {
      new URLSearchParams(root.location.search).forEach(function (v, k) { qs[k] = v; });
    } catch (e) { /* older browsers: parameters ignored */ }

    if (qs.embed === '1' || qs.embed === 'true') document.body.classList.add('embed');

    var theme = qs.theme || load('theme');
    if (theme !== 'light' && theme !== 'dark') {
      theme = (root.matchMedia && root.matchMedia('(prefers-color-scheme: light)').matches)
        ? 'light' : 'dark';
    }
    setTheme(theme);

    bindUI();
    bindPresets();
    ctrl.configure({ q: [10, 1, 100, 10], R: 0.5 });
    setMode(qs.mode === 'expert' ? 'expert' : 'simple');
    updatePlayLabel();

    root.addEventListener('resize', function () { R.readColors(); });

    // Entry point for the console and for embedding in other pages.
    IPM.app = {
      sim: sim, ctrl: ctrl, opts: opts, plant: plant, hist: hist, mouse: mouse,
      reset: reset, kick: kick, setMode: setMode, setTheme: setTheme,
      result: function () { return lastRes; }
    };

    lastFrame = performance.now();
    root.requestAnimationFrame(frame);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})(typeof window !== 'undefined' ? window : globalThis);
