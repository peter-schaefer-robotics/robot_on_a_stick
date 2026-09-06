/*
 * Plant model: inverted pendulum on a cart (cart-pole).
 *
 * State    x = [p, p_dot, theta, theta_dot]
 *   p        cart position [m]        (positive to the right)
 *   theta    rod angle [rad]          (0 = upright, positive = tip to the right)
 * Input    u = force on the cart [N]
 * Disturb. fd = horizontal force on the pendulum mass [N] (the mouse "push")
 *
 * Assumed: a massless rod of length l carrying a point mass mp at its tip, a
 * cart of mass Mc, and viscous friction b in the cart bearing.
 *
 * Equations of motion (Lagrange, no small-angle approximation):
 *
 *   p_ddot     = [ u - b*p_dot + fd*sin^2(th) + mp*l*th_dot^2*sin(th)
 *                  - mp*g*sin(th)*cos(th) ] / (Mc + mp*sin^2(th))
 *   theta_ddot = [ g*sin(th) - p_ddot*cos(th) + (fd/mp)*cos(th) ] / l
 *
 * These nonlinear equations are what gets simulated. The controller only ever
 * sees the linearised model - exactly as in practice.
 */
(function (root) {
  'use strict';

  var IPM = root.IPM = root.IPM || {};

  var DEFAULT_PLANT = {
    Mc: 0.5,   // cart mass [kg]
    mp: 0.2,   // pendulum mass [kg]
    l: 0.5,    // rod length [m]
    g: 9.81,   // gravity [m/s^2]
    b: 0.1     // viscous cart friction [N s/m]
  };

  /** State derivative, f(x, u, fd). */
  function deriv(s, u, fd, p) {
    var dp = s[1], th = s[2], dth = s[3];
    var st = Math.sin(th), ct = Math.cos(th);
    var den = p.Mc + p.mp * st * st;
    var ddp = (u - p.b * dp + fd * st * st + p.mp * p.l * dth * dth * st
               - p.mp * p.g * st * ct) / den;
    var ddth = (p.g * st - ddp * ct + (fd / p.mp) * ct) / p.l;
    return [dp, ddp, dth, ddth];
  }

  function axpy(s, k, h) {
    return [s[0] + h * k[0], s[1] + h * k[1], s[2] + h * k[2], s[3] + h * k[3]];
  }

  /** One simulation step with classical Runge-Kutta 4. */
  function rk4(s, u, fd, p, h) {
    var k1 = deriv(s, u, fd, p);
    var k2 = deriv(axpy(s, k1, h / 2), u, fd, p);
    var k3 = deriv(axpy(s, k2, h / 2), u, fd, p);
    var k4 = deriv(axpy(s, k3, h), u, fd, p);
    return [
      s[0] + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
      s[1] + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
      s[2] + h / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]),
      s[3] + h / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
    ];
  }

  /**
   * Analytic linearisation about the upright equilibrium (theta = 0, u = 0).
   * Returns A, B as nested arrays and the offset c = 0.
   */
  function linearizeUpright(p) {
    var Mc = p.Mc, mp = p.mp, l = p.l, g = p.g, b = p.b;
    return {
      A: [
        [0, 1, 0, 0],
        [0, -b / Mc, -mp * g / Mc, 0],
        [0, 0, 0, 1],
        [0, b / (Mc * l), (Mc + mp) * g / (Mc * l), 0]
      ],
      B: [0, 1 / Mc, 0, -1 / (Mc * l)],
      c: [0, 0, 0, 0]
    };
  }

  /**
   * Numerical linearisation about an arbitrary operating point (s0, u0):
   *   x_dot ~= f(s0,u0) + A (x - s0) + B (u - u0)
   *          = A x + B u + c,   c = f(s0,u0) - A s0 - B u0
   * Central differences, accurate enough for this application.
   */
  function linearizeAt(s0, u0, p) {
    var eps = 1e-6, n = 4;
    var A = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    var B = [0, 0, 0, 0];
    var i, j, sp, sm, fp, fm;

    for (j = 0; j < n; j++) {
      sp = s0.slice(); sm = s0.slice();
      sp[j] += eps; sm[j] -= eps;
      fp = deriv(sp, u0, 0, p);
      fm = deriv(sm, u0, 0, p);
      for (i = 0; i < n; i++) A[i][j] = (fp[i] - fm[i]) / (2 * eps);
    }
    fp = deriv(s0, u0 + eps, 0, p);
    fm = deriv(s0, u0 - eps, 0, p);
    for (i = 0; i < n; i++) B[i] = (fp[i] - fm[i]) / (2 * eps);

    var f0 = deriv(s0, u0, 0, p);
    var c = [0, 0, 0, 0];
    for (i = 0; i < n; i++) {
      var v = f0[i] - B[i] * u0;
      for (j = 0; j < n; j++) v -= A[i][j] * s0[j];
      c[i] = v;
    }
    return { A: A, B: B, c: c };
  }

  /** Total energy (kinetic + potential), for display only. */
  function energy(s, p) {
    var dp = s[1], th = s[2], dth = s[3];
    var T = 0.5 * p.Mc * dp * dp
          + 0.5 * p.mp * (dp * dp + 2 * p.l * dp * dth * Math.cos(th) + p.l * p.l * dth * dth);
    var V = p.mp * p.g * p.l * Math.cos(th);
    return T + V;
  }

  IPM.model = {
    DEFAULT_PLANT: DEFAULT_PLANT,
    deriv: deriv,
    rk4: rk4,
    linearizeUpright: linearizeUpright,
    linearizeAt: linearizeAt,
    energy: energy
  };
})(typeof window !== 'undefined' ? window : globalThis);
