# Visible text — Inverted Pendulum MPC playground

Every piece of text the page shows, in the order it appears. Edit the text under each
key and send the file back; the keys are what I use to put it in the right place.

**Conventions**

- Keep the `key:` lines exactly as they are — only change the text below them.
- `**bold**` and `*italic*` are kept as emphasis.
- `x_sub` marks a subscript: `u_max`, `T_s`, `q_θ`, `e_N` render as u<sub>max</sub> etc.
- `⟨formula⟩` is a MathML formula, not editable here. Tell me in plain words if one
  should change.
- `[expert]` = shown in expert mode only, `[simple]` = simple mode only, no marker =
  both. Whole sections marked `[expert]` are hidden in simple mode.
- `[beat]` = only while "Beat the Controller" is on, `[controller]` = only while the
  MPC is driving.
- Deleting a whole entry is fine — say so and I will remove the element too.

---

## 1 · Page metadata

Not visible on the page itself — browser tab and search results.

`meta.title:`

Inverted Pendulum — MPC Playground

`meta.description:`

Interactive browser demo of model predictive control on an inverted pendulum.

## 2 · Header

`header.h1:`

Robot on a stick!

`header.intro:`  (the loose 2–3 sentence opener)

This is a fun control theory example made accessible in a playful manner.

## 3 · Cost function card

`eq.cardtitle:`

The optimization problem — solved every 20 ms

The formula itself is built from MathML plus the editable matrix fields:

> min over u_0 … u_N−1 of  J = Σ ( e_k^⊤ **Q** e_k + R u_k² ) + e_N^⊤ **P** e_N

The little column headers above each matrix are the state names: `p`, `ṗ`, `θ`, `θ̇`,
and the letters below the brackets are `Q`, `R`, `P`.

`eq.pauto.label:`  [expert]

P from the Riccati equation (uncheck to type your own diagonal)

`eq.constraints.label:`  [expert]  (followed by three MathML constraints)

subject to, for k = 0 … N−1:

## 4 · "What the controller is looking at"

`state.cardtitle:`

What the controller looks at, and what it penalises

`state.vector.rows:`  (the four lines next to the state vector ⟨p, ṗ, θ, θ̇⟩)

- **cart position** [m]
- **cart velocity** [m/s]
- **rod angle**, 0 = upright [rad]
- **angular rate** [rad/s]

`state.symbols:`  (one entry per row; the formula on the left is fixed)

- ⟨formula⟩ the state error the controller *expects* k steps from now [simple:.] [expert: — not a measurement, a prediction from its model. Weighting it with **Q** is what makes one kind of error more expensive than another.]
- ⟨formula⟩ penalises state error: how expensive each kind of deviation is. Its four diagonal entries line up with the four states above.
- ⟨formula⟩ the horizontal force on the cart [N], k steps from now. [expert: It is the only thing the controller can actually do — nothing pushes the rod directly.]
- ⟨formula⟩ penalises expensive actuation: how much that force costs. A large R buys smoothness and pays for it with speed.
- ⟨formula⟩ how many steps it looks ahead. [simple: Here that is two seconds of future.] [expert: Look-ahead is N · T_s; both are adjustable below.]
- [expert] ⟨formula⟩ the discrete-time model used for the prediction (zero-order hold, sample time T_s), obtained by linearising the equations of motion.

## 5 · Presets

`presets.cardtitle:`

Starting points

`presets.buttons:`  (name — subtitle)

- **Balanced** — a sensible compromise
- **Gentle** — large R — smooth and lazy
- **Aggressive** — tiny R — snappy and twitchy
- [expert] **Tight position** — holds the cart, swings the rod
- [expert] **Angle only** — rod upright, cart drifts away
- [expert] **Short-sighted** — N = 8 without P — it falls over

## 6 · Mode buttons

Two buttons side by side, each with a label and a subtitle. The labels live in
`js/app.js`, the note below them in `index.html`.

`mode.button.toExpert:`  (shown while in simple mode)

**Expert mode** — show the model, all tuning parameters and diagnostics

`mode.button.toSimple:`  (shown while in expert mode)

**Simple mode** — back to the essentials

`mode.button.toBeat:`  (second button, shown while the controller is running)

**Beat the Controller** — switch the controller off and balance the rod yourself

`mode.button.toController:`  (same button, shown while you are in control)

**Give me the controller back** — hand the rod back to the MPC

`mode.beatnote:`  [beat]  (the box under the buttons — this is what tells the user to
drag the cart)

**You are the controller now — click anywhere in the scene and drag left or right to pull the cart.** That is your only handle, and you get the same 15 N limit the MPC has. The rod starts perfectly upright and can no longer be pushed; it stays up until you move. Everything runs at **half speed**, because at full speed human reaction time loses this game in about three seconds. The clock still counts simulated seconds.

## 7 · Animation

`stage.readout:`  (labels of the readout box; the values are numbers)

- Angle θ
- Control force u
- [controller] Your disturbance
- [beat] Balancing time
- [beat] Your best
- [expert] Position p
- [expert] Cost J
- [expert] QP solve time
- [expert] Iterations

`stage.status:`  [expert]  (status line under the readout, one of two)

- ⚠ input bound active
- ✓ exact (bounds inactive)

`stage.buttons:`

- Pause
- Reset
- Kick ←
- Kick →
- Start / Pause toggle on the first button (set from `js/app.js`)

`stage.checkboxes:`

- Prediction
- [expert] Forces
- [expert] Bob trace

`stage.hint:`

Move the mouse close to the ball to push it around. Click the rail to set a new target position. Space = pause. R = reset. Arrow keys = kick.

## 8 · Plots  [expert]

`plots.scope.title:`

Time histories (last 12 s)

`plots.scope.legend:`  (the three trace labels)

- Angle θ [°]
- Position p [m]
- Cart force u [N]

`plots.uplan.title:`

Planned input sequence over the horizon

`plots.uplan.hint:`

The controller commits only to the first bar. Everything to the right of it is a plan it will throw away and recompute at the next sample.

## 9 · Settings  [expert]

`settings.controller.title:`

Controller

`settings.controller.labels:`

- Prediction model
- Horizon N
- Sample time T_s
- Input bound u_max
- Terminal weight P in the cost function

`settings.controller.options:`  (dropdown entries)

- linear (about upright)
- nonlinear (SQP, 1 iteration per step)

`settings.plant.title:`

Plant & reality

`settings.plant.labels:`

- Pendulum mass m
- Cart mass M
- Rod length l
- Cart friction b
- Mouse disturbance strength
- Measurement noise σ
- Controller model error
- Target position p_ref

`settings.plant.hint:`

The controller assumes a pendulum mass off by this much.

## 10 · Explanations

Sections marked `[expert]` do not appear in simple mode at all. Inside the shared
sections, single paragraphs or fragments marked `[expert]` are dropped in simple mode.

`explain.title:`

In a bit more detail

### 1. The setup

`explain.the-setup.title:`

The setup

`explain.the-setup.body:`

A cart slides along a horizontal rail. On it sits a freely pivoting rod with a mass at its tip. The only thing the system can actively do is push the cart left or right — there is no motor at the joint. Two things can move (cart and rod), only one thing can be actuated (cart).

Upright is an **unstable equilibrium**. Lean the rod a little and it leans faster and faster. The controller has to keep sliding the cart back underneath the centre of mass.

There is a second goal — the cart should also end up at its target position. Short term the two goals fight each other: to get to a target on the right, the cart must first duck briefly to the *left* so the rod tips rightwards. That backward step is visible every time you click a new target, and it is not a flaw. [expert: It is a **non-minimum-phase** behaviour, a zero in the right half-plane.]

### 2. How the controller works

`explain.how-the-controller-works.title:`

How the controller works

`explain.how-the-controller-works.body:`

Every 20 milliseconds the same five steps run:

1. **Measure** where the pendulum is right now.
1. **Predict** what would happen over the next N steps for any sequence of cart forces, using a model of the plant.
1. **Optimise**: find the sequence with the lowest total cost J [expert: that also respects every bound].
1. **Apply** only the very first input step of that sequence.
1. **Repeat**, shifted one step into the future.

Throwing away most of the plan sounds wasteful, but it is what turns planning into **feedback**. Every cycle restarts from a fresh measurement, so a gust of wind, a mouse push or a slightly wrong model get absorbed automatically. Switch on *Prediction* above and the ghost trail shows you the plan being reconsidered several times per second.

### 3. What Q and R actually do

`explain.what-q-and-r-actually-do.title:`

What Q and R actually do

`explain.what-q-and-r-actually-do.body:`

The cost function is a bargain between two wishes: *keep the state error small* (that is Q) and *do not use much force* (that is R). Both terms are squared, which keeps the problem convex — there is exactly one best answer, and it can be found reliably in well under a millisecond.

- **Large R** → smooth, economical, lazy. Push it far enough and the planned force is no longer enough to beat gravity.
- **Large q_θ** → the angle becomes the priority; the cart is allowed to wander a long way to keep the rod vertical.
- **Large q_p** → the cart stays put, and pays for it with bigger swings of the rod.
- **q_ṗ and q_θ̇** penalise speeds rather than positions. They act like damping: less overshoot, less oscillation.

Two things are worth knowing. Only the **ratio** counts — scale Q and R by the same factor and nothing changes at all. And the entries carry **units**: q_p acts on m², q_θ on rad². An error of 0.1 rad (5.7°) is physically far more serious than 0.1 m of cart offset, which is why q_θ is usually the biggest number in the matrix.

[expert] When trying to set the matrix values of Q and R, a useful starting point is **Bryson's rule**: set q_i = 1 / (largest deviation you can live with)² and R = 1 / u_max². It rarely gives the final answer, but it gets the orders of magnitude right, which is most of the battle.

### 4. Pushing it around

`explain.pushing-it-around.title:`

Pushing it around

`explain.pushing-it-around.body:`

Near the ball, your mouse pointer acts like a finger: a horizontal force is applied to the pendulum mass, depending on how close you are and how fast you are moving. The controller is told nothing about it. It only sees the consequences in the measured state and reacts — which is exactly the situation in a real plant.

Park the pointer against the ball and hold it there. The system settles into a new equilibrium with a **permanent offset**: the rod leans just far enough for the lean to balance your push and the cart drifts away from its target. Nothing in the cost function accumulates past error, so nothing ever cancels a constant unknown force completely.

[expert] Two more realities are on the sliders. **Measurement noise** goes straight into the optimisation, because there is no state estimator here — turn it up and the control force gets visibly nervous. **Controller model error** makes the controller believe in a wrong pendulum mass.

### 5. Simulation versus controller model [expert]

`explain.simulation-versus-controller.title:`

Simulation versus controller model

`explain.simulation-versus-controller.body:`

What is simulated is the full nonlinear dynamics — Lagrange, Runge–Kutta 4th order, step size T_s/8:

⟨formula block — not editable here⟩

The controller never sees those equations. It only gets a **linearised, discrete-time** model.

In *linear* mode the linearisation happens once, about the upright equilibrium; A_d, B_d and the QP Hessian are then constant and are only rebuilt when you change a parameter.

In *nonlinear* mode the previously planned input sequence is simulated forward through the nonlinear model and the system is re-linearised at every point along that trajectory — a real-time iteration, one SQP step per sample. It costs about an order of magnitude more computation, and it earns that back by catching the pendulum from deflections where the fixed linearisation is long since nonsense. Swipe the ball hard with the mouse in both modes: the fixed linearisation gives up somewhere past 30–40°, while the nonlinear one keeps catching the pendulum well beyond that.

### 6. Horizon N and the terminal weight P [expert]

`explain.horizon-n-and-the-terminal-w.title:`

Horizon N and the terminal weight P

`explain.horizon-n-and-the-terminal-w.body:`

The horizon is finite: whatever happens after step N never enters the sum. That makes a short horizon genuinely dangerous — the controller does not *see* the fall coming.

P improves this. As the solution of the discrete algebraic Riccati equation, e_N^⊤P e_N is exactly the cost an LQR would still incur from step N to infinity. Adding it makes a short horizon behave like an infinite one, as long as the input bound stays inactive.

**Try it:** pick the *Short-sighted* preset (N = 8, P off) and the pendulum falls. Now tick the terminal weight back on and change nothing else — the same eight steps of look-ahead suddenly stabilise perfectly.

The other way to buy the same safety is simply to look further ahead, which is what simple mode does: no P, but N = 100 — a full two seconds of future. Cheaper to explain, more expensive to compute, and it stops working as soon as you shorten the horizon again.

Unticking *P from the Riccati equation* lets you type your own diagonal terminal weight. It is instructive to see how much worse a hand-picked P is than the one Riccati gives you — the off-diagonal couplings in the automatic solution are doing real work.

### 7. Constraints — the real reason to use MPC [expert]

`explain.constraints-the-real-reason-.title:`

Constraints — the real reason to use MPC

`explain.constraints-the-real-reason-.body:`

Without the bound |u_k| ≤ u_max this whole problem has a closed-form answer: the LQR, a constant state feedback u = −Kx. No optimisation, no solver, faster and simpler in every respect.

The difference shows up at the limit. An LQR computes as though the bound did not exist and then gets clipped — it plans with force it will never receive. The MPC puts the bound inside the optimisation, so it knows in advance that it will run out of force later and starts countering earlier. That is what the computation buys you.

Set u_max to about 3 N and push the ball hard: the readout reports *input bound active*, and the planned input sequence visibly sits on its limit.

Only input bounds are implemented here. State constraints — a rail of finite length, |p| ≤ p_max — have the same structure but need a QP solver for general inequalities, plus slack variables so the problem stays solvable when a disturbance pushes you outside anyway.

### 8. What happens numerically [expert]

`explain.what-happens-numerically.title:`

What happens numerically

`explain.what-happens-numerically.body:`

The state equations are substituted into the cost function (*condensed form*), X = Φx_0 + ΓU. What is left is a quadratic program in N variables, in the input sequence alone:

⟨formula block — not editable here⟩

Because the only constraints are box bounds, the solver stays compact. First the unconstrained solution is computed exactly via a Cholesky factorisation of H. If it happens to lie inside the bounds it already *is* the optimum — the readout then says *exact* with 0 iterations, which is the usual case. Otherwise it is clipped and used to start a projected coordinate descent that finds the active set. Each solution also warm-starts the next sample.

The displayed solve time is genuinely measured, averaged over recent samples because browsers round their clocks to 100 µs. Typically it stays well under 0.1 ms per sample with a fixed model, and around a millisecond in nonlinear mode where the model and the Hessian are rebuilt from scratch every single step.

### 9. Demo limitations [expert]

`explain.demo-limitations.title:`

Demo limitations

`explain.demo-limitations.body:`

- The full state is assumed measurable. In reality ṗ and θ̇ would be estimated from encoder readings, normally with a Kalman filter, and that estimator has dynamics of its own.
- No swing-up from hanging: that is not a QP any more and needs genuine nonlinear optimisation or a separate energy-based controller. Nonlinear mode with a long horizon does catch the pendulum from surprisingly large angles, though.
- The actuator is ideal — no dead time, no dynamics, no quantisation. Dead time can spoil an MPC design.
- No state constraints, so the rail is infinitely long and the cart can wander off happily.

## 11 · Footer

`footer.text:`

No build step, no dependencies — plain HTML, CSS and JavaScript.

## 12 · Tooltips

Shown on hover, so worth getting right too.

`tooltip.theme:`  Toggle light / dark

`tooltip.matrices:`

- Q — state weights: how expensive each kind of error is
- R — how expensive control force is
- P — terminal weight: the cost of everything beyond the horizon

`tooltip.qfields:`  (one per editable Q entry)

- weight on cart position error [1/m²]
- weight on cart velocity [1/(m/s)²]
- weight on rod angle [1/rad²]
- weight on angular rate [1/(rad/s)²]

## 13 · Units and number formats

Appended to the numeric readouts in `js/app.js`. Change only if you want different
wording — the units themselves have to match the physics.

- Horizon: `40  (0.80 s)` · Sample time: `20 ms  (50 Hz)` · Input bound: `15.0 N`
- Masses `kg`, length `m`, friction `Ns/m`, disturbance strength `1.00×`
- Measurement noise: `off` when zero, otherwise `5.0 mm`
- Controller model error: `+50 %`, target position: `0.00 m`
- Readout: angle `-0.3°`, force `0.31 N`, position `0.021 m`, solve time `0.084 ms`
