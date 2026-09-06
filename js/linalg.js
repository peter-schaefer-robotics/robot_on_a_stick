/*
 * Minimale Matrix-Bibliothek fuer die MPC-Demo.
 * Matrizen: { r, c, d } mit d als Float64Array in row-major Reihenfolge.
 * Bewusst klein gehalten - es werden nur die Operationen implementiert,
 * die der Regler wirklich braucht.
 */
(function (root) {
  'use strict';

  var IPM = root.IPM = root.IPM || {};

  function mat(r, c) {
    return { r: r, c: c, d: new Float64Array(r * c) };
  }

  function eye(n) {
    var A = mat(n, n);
    for (var i = 0; i < n; i++) A.d[i * n + i] = 1;
    return A;
  }

  function fromRows(rows) {
    var r = rows.length, c = rows[0].length, A = mat(r, c);
    for (var i = 0; i < r; i++) {
      for (var j = 0; j < c; j++) A.d[i * c + j] = rows[i][j];
    }
    return A;
  }

  function clone(A) {
    return { r: A.r, c: A.c, d: A.d.slice() };
  }

  function mul(A, B) {
    if (A.c !== B.r) throw new Error('mul: Dimensionsfehler ' + A.c + ' vs ' + B.r);
    var C = mat(A.r, B.c), i, k, j, a;
    for (i = 0; i < A.r; i++) {
      for (k = 0; k < A.c; k++) {
        a = A.d[i * A.c + k];
        if (a === 0) continue;
        for (j = 0; j < B.c; j++) C.d[i * B.c + j] += a * B.d[k * B.c + j];
      }
    }
    return C;
  }

  /** y = A * x, x als einfaches Array/Float64Array. */
  function mulVec(A, x) {
    var y = new Float64Array(A.r), i, j, s;
    for (i = 0; i < A.r; i++) {
      s = 0;
      for (j = 0; j < A.c; j++) s += A.d[i * A.c + j] * x[j];
      y[i] = s;
    }
    return y;
  }

  function add(A, B) {
    var C = mat(A.r, A.c);
    for (var i = 0; i < A.d.length; i++) C.d[i] = A.d[i] + B.d[i];
    return C;
  }

  function sub(A, B) {
    var C = mat(A.r, A.c);
    for (var i = 0; i < A.d.length; i++) C.d[i] = A.d[i] - B.d[i];
    return C;
  }

  function scale(A, s) {
    var C = mat(A.r, A.c);
    for (var i = 0; i < A.d.length; i++) C.d[i] = A.d[i] * s;
    return C;
  }

  function transpose(A) {
    var C = mat(A.c, A.r);
    for (var i = 0; i < A.r; i++) {
      for (var j = 0; j < A.c; j++) C.d[j * A.r + i] = A.d[i * A.c + j];
    }
    return C;
  }

  /** Groesste absolute Zeilensumme (Unendlich-Norm). */
  function normInf(A) {
    var best = 0, i, j, s;
    for (i = 0; i < A.r; i++) {
      s = 0;
      for (j = 0; j < A.c; j++) s += Math.abs(A.d[i * A.c + j]);
      if (s > best) best = s;
    }
    return best;
  }

  function maxAbsDiff(A, B) {
    var m = 0, v;
    for (var i = 0; i < A.d.length; i++) {
      v = Math.abs(A.d[i] - B.d[i]);
      if (v > m) m = v;
    }
    return m;
  }

  /**
   * Matrix-Exponential exp(A) ueber "scaling and squaring" mit Taylorreihe.
   * Genau genug fuer die kleinen (6x6) Systemmatrizen hier und ohne
   * externe Abhaengigkeit.
   */
  function expm(A) {
    var n = A.r;
    var nrm = normInf(A);
    var s = 0;
    if (nrm > 0.5) s = Math.max(0, Math.ceil(Math.log2(nrm / 0.5)));
    var As = scale(A, Math.pow(2, -s));
    var result = eye(n);
    var term = eye(n);
    for (var k = 1; k <= 24; k++) {
      term = scale(mul(term, As), 1 / k);
      result = add(result, term);
      if (normInf(term) < 1e-16 * (1 + normInf(result))) break;
    }
    for (var i = 0; i < s; i++) result = mul(result, result);
    return result;
  }

  /** Untermatrix [r0, r1) x [c0, c1). */
  function block(A, r0, r1, c0, c1) {
    var C = mat(r1 - r0, c1 - c0);
    for (var i = r0; i < r1; i++) {
      for (var j = c0; j < c1; j++) C.d[(i - r0) * C.c + (j - c0)] = A.d[i * A.c + j];
    }
    return C;
  }

  IPM.linalg = {
    mat: mat, eye: eye, fromRows: fromRows, clone: clone,
    mul: mul, mulVec: mulVec, add: add, sub: sub, scale: scale,
    transpose: transpose, normInf: normInf, maxAbsDiff: maxAbsDiff,
    expm: expm, block: block
  };
})(typeof window !== 'undefined' ? window : globalThis);
