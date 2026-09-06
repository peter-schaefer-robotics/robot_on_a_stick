# Inverted Pendulum on a Cart — an interactive MPC playground

A browser demo of **model predictive control** on the classic cart–pole system. The weight
matrices **Q** and **R** sit inside the cost function itself and can be edited there; the
change takes effect on the running controller immediately. Move the mouse near the ball to
push the pendulum around and watch the controller argue with you.

No build step, no dependencies, no framework — plain HTML, CSS and JavaScript.

## Quick start

```bash
git clone https://github.com/peter-schaefer-robotics/inv_pendulum_mpc.git
```

Then just double-click `index.html`. The page deliberately uses classic `<script>` tags
rather than ES modules so that it also runs straight from `file://`, with no server and no
CORS errors.

If you prefer a local server (handy to avoid browser caching while editing):

```bash
python3 -m http.server 8000
```

Tested in current Chrome, Firefox, Safari and Edge, on desktop and mobile.

## Beat the controller

A second button next to the mode switch turns the MPC off and hands the cart to you: click
anywhere in the scene and drag to pull it left or right — or hold the arrow keys — through a
spring-damper that is clipped to the same 15 N the controller gets. The rod starts perfectly upright, cannot be
pushed any more, and stays up until you move. A clock counts how long you keep it within 60°,
and remembers your best run.

It runs at **half speed**, deliberately. A 0.5 m pendulum has a time constant of about
0.19 s; measured against a human reaction time of ~150 ms, even optimal play survives roughly
3.5 seconds in real time — too short to learn anything from. At half speed the same strategy
lasts around 14 s. The physics is untouched, only playback is slowed, the clock counts
simulated seconds, and the page says so. The controller, meanwhile, does this in real time
and indefinitely.

## Two modes

The page starts in **simple mode**: the cost function, the state vector, three presets and
the animation. Everything else — plots, tuning parameters, plant parameters, diagnostics —
is hidden behind the large **Expert mode** button underneath the animation. Switching modes
also resets every hidden setting back to its default, so you always return to a known state.

One design decision is worth spelling out. Simple mode drops the terminal weight **P** to keep
the formula short, and compensates with a longer horizon: **N = 100** instead of 40, i.e. two
full seconds of look-ahead. Both are ways to approximate an infinite horizon, and without
either of them the cart holds the rod upright but slowly drifts away from its target — a
horizon of 0.8 s simply does not reach far enough to see the cost of that drift. Expert mode
uses the more common combination: a shorter horizon plus the Riccati terminal weight.

| | Simple | Expert |
| --- | --- | --- |
| Horizon N | 100 (2.0 s) | 40 (0.8 s), adjustable |
| Terminal weight P | off | on, from the Riccati equation — or type your own diagonal |
| Presets | Balanced, Gentle, Aggressive | plus Tight position, Angle only, Short-sighted |
| Readout | angle, control force, disturbance | plus position, cost, solve time, iterations, bound status |
| Plots | – | time histories and the planned input sequence |
| Settings | – | horizon, sample time, input bound, prediction model, plant, noise, model error |

## Controls

| Input | Effect |
| --- | --- |
| Mouse near the robot head | pushes the pendulum (horizontal force on the pendulum mass) |
| Click on the scene | sets the cart's target position |
| Click and drag (Beat the controller) | pulls the cart yourself |
| Arrow keys (Beat the controller) | move the cart with the keyboard |
| Q and R fields | change the weights — takes effect immediately |
| Space | pause / resume |
| `R` | reset |
| Arrow keys ← → | reproducible disturbance impulse |

## What the demo shows

The cost function minimised at every control step:

$$J=\sum_{k=0}^{N-1}\Big(e_k^\top Q\,e_k + R\,u_k^2\Big) + e_N^\top P\,e_N,
\qquad e_k = x_k - x_\text{ref}$$

subject to $x_{k+1}=A_d x_k + B_d u_k$, $|u_k|\le u_\text{max}$ and $x_0 = x(t)$.

Effects that are deliberately easy to reproduce:

* **Q against R.** Large `R` gives smooth, economical, lazy control; large `q_θ` prioritises
  the angle; large `q_p` keeps the cart in place at the price of bigger swings. Only the ratio
  matters — scale Q and R by the same factor and nothing changes.
* **Horizon and terminal weight.** The *Short-sighted* preset (N = 8, no P) makes the pendulum
  fall. Tick the terminal weight back on, change nothing else, and the same horizon
  stabilises.
* **Constraints.** Lower `u_max` and push hard: the controller plans *with* the bound instead
  of clipping afterwards. That is what separates MPC from an LQR.
* **Non-minimum-phase behaviour.** To move right, the cart must first duck left — visible at
  every target change.
* **Steady-state offset.** Hold the pointer against the ball and the rod settles at
  tan θ = −F_d/(m·g) while the cart drifts off target: the cost function has no integral term.
* **Computation.** The displayed QP solve time is genuinely measured — typically under 0.1 ms
  per step in linear mode, roughly an order of magnitude more in nonlinear mode.

## How it works

**Plant.** Cart of mass `M` with viscous friction `b`, massless rod of length `l` with a point
mass `m` at the tip. State `x = [p, ṗ, θ, θ̇]`, input: horizontal force on the cart. The full
nonlinear dynamics are simulated with 4th-order Runge–Kutta at a step size of `Ts/8`:

```
p̈ = [u − b·ṗ + F_d·sin²θ + m·l·θ̇²·sinθ − m·g·sinθ·cosθ] / (M + m·sin²θ)
θ̈ = [g·sinθ − p̈·cosθ + (F_d/m)·cosθ] / l
```

`F_d` is the mouse disturbance acting on the pendulum mass. The controller is **not** told
about it — it only sees the consequences in the measured state.

**Controller.** Two prediction models:

* *linear* — one linearisation about the upright equilibrium, exact zero-order-hold
  discretisation via the matrix exponential. `A_d`, `B_d` and the QP Hessian are constant and
  rebuilt only when a parameter changes.
* *nonlinear* — real-time iteration (one SQP iteration per step): the previously planned input
  sequence is simulated forward through the nonlinear model and the system is re-linearised at
  every point along that trajectory. The resulting time-varying model recovers the pendulum
  from deflections where the fixed linearisation is long since wrong.

The affine term of the linearisation is embedded via an augmented state `z = [x; 1]` so the
prediction stays linear.

**QP.** Substituting the state equations into the cost (*condensed form*) gives
`X = Φz₀ + ΓU` and leaves a box-constrained QP in N variables:

```
min_U  ½ Uᵀ H U + gᵀ U      with  H = 2(Γᵀ Q̄ Γ + R·I),   −u_max ≤ U ≤ u_max
```

It is solved in two stages: first the unconstrained solution exactly, from a Cholesky
factorisation of H. If it lies inside the bounds it already is the optimum (the readout says
*exact*, 0 iterations — the usual case). Otherwise it is clipped and used to start a projected
coordinate descent that identifies the active set. Every solution warm-starts the next step.

The terminal weight P is the solution of the discrete algebraic Riccati equation by value
iteration — with a single input, `R + BᵀPB` is a scalar, so no linear solver is needed. In
expert mode you can untick the automatic solution and type your own diagonal instead.

## Project layout

```
index.html          page structure, formulas as MathML (no external library)
styles.css          layout and colours; light/dark via CSS variables
js/robot.js         the robot head that rides on the rod, inlined as an SVG string
js/linalg.js        minimal matrix library including the matrix exponential
js/model.js         nonlinear plant dynamics, RK4, linearisation
js/mpc.js           prediction matrices, Riccati, QP solver, controller class
js/render.js        canvas drawing: scene, time histories, input sequence
js/app.js           simulation loop, controls, disturbances, mode switching
assets/             the original robot-head.svg the inlined artwork comes from
astro/              example component for embedding in an Astro site
TEXT.md             every visible string on the page, for editing the copy in one file
tools_extract_text.py   regenerates TEXT.md from index.html
```

`window.IPM.app` exposes the simulation state, the controller and the parameters in the
browser console — useful for experimenting:

```js
IPM.app.ctrl.configure({ q: [10, 1, 300, 10], R: 0.05 });
IPM.app.kick(1);
IPM.app.setMode('expert');
IPM.app.setPlayMode('manual');   // controller off, you drive
IPM.app.result();     // last QP solution including solve time
IPM.app.step(0.02);   // advance the simulation by hand, independent of the animation
```

The robot head is inlined as an SVG string in `js/robot.js` rather than loaded from
`assets/`: a separate file would be blocked as a cross-origin request when the page is opened
straight from the file system. Edit `assets/robot-head.svg` and copy the shapes over if you
want a different one.

## URL parameters

| Parameter | Values | Meaning |
| --- | --- | --- |
| `mode` | `simple`, `expert` | starting mode (default: simple) |
| `theme` | `dark`, `light` | colour scheme |
| `embed` | `1` | hides heading, explanations and footer |

Example: `index.html?mode=expert&theme=light&embed=1`

## Embedding in an Astro site

**Option A — iframe (recommended).** Keeps the demo's styles separate from the site's own and
is immune to updates on either side.

1. Copy the contents of this repo into `public/pendulum/` of the Astro site
   (`index.html`, `styles.css`, `js/`).
2. Copy `astro/InvertedPendulumMpc.astro` into `src/components/`.
3. Use it in a page:

```astro
---
import InvertedPendulumMpc from '../components/InvertedPendulumMpc.astro';
---
<InvertedPendulumMpc mode="simple" embed={true} height="900px" />
```

Since GitHub Pages usually serves a site from a sub-path, the component uses
`import.meta.env.BASE_URL`, so the path stays correct with a `base` set in `astro.config.mjs`.

**Option B — inline in the page.** Copy the contents of `<body>` (without the `<script>`
lines) into an `.astro` component, import `styles.css`, and load the scripts at the end with
`<script is:inline src={...}>` in the order `linalg, model, mpc, render, app`. `is:inline`
matters: otherwise Astro bundles them as ES modules and the global names no longer line up.
With this option, check the demo's selectors (`.card`, `.btn`, `.field`, …) against your own
site styles.

## Limitations

* The full state is assumed measurable; a real rig would need an observer (Kalman filter).
* No swing-up from hanging — that is no longer a QP.
* The actuator is ideal: no dead time, no dynamics, no quantisation.
* No state constraints, so the rail is infinitely long.

## License

MIT — see [LICENSE](LICENSE).
