# Inverses Pendel am Wagen — interaktive MPC-Demonstration

Eine spielerische, aber fachlich korrekte Browser-Demonstration einer **modellprädiktiven
Regelung (MPC)** am klassischen Regelungstechnik-Beispiel: ein inverses Pendel auf einem
Wagen. Die Gewichtungsmatrizen **Q** und **R** lassen sich live eintragen und wirken sofort
auf die Regelung; mit der Maus kann das Pendel angestoßen und gestört werden.

Kein Build-Schritt, keine Abhängigkeiten, kein Framework — reines HTML, CSS und JavaScript.
Die Oberfläche ist zweisprachig (Deutsch / Englisch).

> *English:* an interactive, dependency-free browser demo of model predictive control on the
> classic cart–pole system. Tune the Q and R weights live, disturb the pendulum with your
> mouse. Switch the interface language with the button in the top right, or open the page
> with `?lang=en`.

---

## Schnellstart

```bash
git clone https://github.com/peter-schaefer-robotics/inv_pendulum_mpc.git
```

Dann `index.html` per Doppelklick öffnen — das genügt. Die Anwendung verwendet bewusst
klassische `<script>`-Tags statt ES-Modulen, damit sie auch direkt über `file://` läuft,
ohne Server und ohne CORS-Fehler.

Wer lieber einen lokalen Server nutzt (z. B. um den Browser-Cache zu umgehen):

```bash
python3 -m http.server 8000
```

und dann `http://localhost:8000` aufrufen. Getestet in aktuellen Versionen von Chrome,
Firefox, Safari und Edge; Desktop und Mobil.

## Bedienung

| Eingabe | Wirkung |
| --- | --- |
| Maus in die Nähe der Kugel bewegen | stößt das Pendel an (horizontale Störkraft auf die Pendelmasse) |
| Klick auf die Szene | setzt die Sollposition des Wagens |
| Q- und R-Felder | Gewichte ändern — wirkt sofort auf die laufende Regelung |
| Leertaste | Pause / weiter |
| `R` | Reset |
| Pfeiltasten ← → | reproduzierbarer Störimpuls |
| Voreinstellungen | typische Tuning-Fälle, inklusive eines instabilen |

## Was die Demo zeigt

Die Kostenfunktion, die in jedem Regeltakt minimiert wird:

$$J=\sum_{k=0}^{N-1}\Big(e_k^\top Q\,e_k + R\,u_k^2\Big) + e_N^\top P\,e_N,
\qquad e_k = x_k - x_\text{soll}$$

unter den Nebenbedingungen $x_{k+1}=A_d x_k + B_d u_k$, $|u_k|\le u_\text{max}$ und
$x_0 = x(t)$.

Bewusst nachvollziehbare Effekte:

* **Q gegen R.** Großes `R` regelt sanft und sparsam, aber träge; große `q_θ` priorisieren
  den Winkel, große `q_p` die Wagenposition. Nur das Verhältnis zählt — skaliert man Q und R
  mit demselben Faktor, ändert sich die Lösung nicht.
* **Horizont und Terminalgewicht.** Die Voreinstellung *Kurzsichtig* (N = 8, ohne P) lässt
  das Pendel umfallen. Schaltet man nur das Terminalgewicht P ein — die Lösung der diskreten
  Riccati-Gleichung —, stabilisiert derselbe Horizont sauber.
* **Nebenbedingungen.** `u_max` klein stellen und kräftig stören: Der Regler plant *mit* der
  Stellgrenze, statt nachträglich abzuschneiden. Genau das unterscheidet MPC vom LQR.
* **Nichtminimalphasiges Verhalten.** Um nach rechts zu fahren, muss der Wagen zuerst kurz
  nach links — sichtbar bei jedem Sollpositionssprung.
* **Stationäre Abweichung.** Bei dauerhafter Störkraft bleibt ein Restfehler stehen: Die
  Kostenfunktion hat keinen Integralanteil. Abhilfe wäre ein Störgrößenbeobachter.
* **Rechenaufwand.** Die angezeigte QP-Rechenzeit ist echt gemessen (typisch < 0,1 ms je
  Takt im linearen Modus, rund eine Größenordnung mehr im nichtlinearen).

## Technische Umsetzung

**Strecke.** Wagen der Masse `M` mit viskoser Reibung `b`, masseloser Stab der Länge `l` mit
Punktmasse `m`. Zustand `x = [p, ṗ, θ, θ̇]`, Eingang: horizontale Kraft auf den Wagen.
Simuliert wird die vollständige nichtlineare Dynamik mit Runge-Kutta 4. Ordnung und der
Schrittweite `Ts/8`:

```
p̈ = [u − b·ṗ + F_d·sin²θ + m·l·θ̇²·sinθ − m·g·sinθ·cosθ] / (M + m·sin²θ)
θ̈ = [g·sinθ − p̈·cosθ + (F_d/m)·cosθ] / l
```

`F_d` ist die Störkraft der Maus, die an der Pendelmasse angreift. Sie ist dem Regler
**nicht** bekannt — er sieht nur ihre Folgen im gemessenen Zustand.

**Regler.** Zwei Prädiktionsmodelle stehen zur Wahl:

* *linear* — einmalige Linearisierung um die aufrechte Ruhelage, exakte Diskretisierung mit
  Halteglied nullter Ordnung über das Matrix-Exponential. `A_d`, `B_d` und die Hesse-Matrix
  des QP sind konstant und werden nur bei Parameteränderungen neu aufgebaut.
* *nichtlinear* — Real-Time-Iteration (eine SQP-Iteration pro Takt): Die zuletzt geplante
  Stellgrößenfolge wird durch das nichtlineare Modell vorwärts simuliert und entlang dieser
  Trajektorie an jedem Schritt neu linearisiert. Ergebnis ist ein zeitvariantes Modell, das
  das Pendel auch aus großen Auslenkungen wieder einfängt.

Der affine Term der Linearisierung wird über einen erweiterten Zustand `z = [x; 1]`
eingebettet, damit die Prädiktion linear bleibt.

**QP.** Die Zustandsgleichungen werden in die Kostenfunktion eingesetzt (*condensed form*),
`X = Φz₀ + ΓU`. Übrig bleibt ein box-beschränktes QP mit N Variablen:

```
min_U  ½ Uᵀ H U + gᵀ U      mit  H = 2(Γᵀ Q̄ Γ + R·I),   −u_max ≤ U ≤ u_max
```

Gelöst wird zweistufig: zuerst die unbeschränkte Lösung exakt über eine Cholesky-Zerlegung
von H. Liegt sie innerhalb der Grenzen, ist sie bereits das Optimum (die Anzeige meldet dann
*exakt*, 0 Iterationen). Andernfalls dient sie geklippt als Startpunkt für einen projizierten
Koordinatenabstieg, der die aktive Menge bestimmt. Jede Lösung wird zum Warmstart des
nächsten Takts weiterverwendet.

Das Terminalgewicht P ist die Lösung der diskreten algebraischen Riccati-Gleichung,
berechnet per Wertiteration — da es nur einen Eingang gibt, ist `R + BᵀPB` skalar und es
wird kein Gleichungslöser gebraucht.

## Projektstruktur

```
index.html          Aufbau der Seite, Formeln als MathML (ohne externe Bibliothek)
styles.css          Layout und Farben (Hell/Dunkel über CSS-Variablen)
js/linalg.js        minimale Matrixbibliothek inkl. Matrix-Exponential
js/model.js         nichtlineare Streckendynamik, RK4, Linearisierung
js/mpc.js           Prädiktionsmatrizen, Riccati, QP-Löser, Reglerklasse
js/render.js        Canvas-Darstellung: Szene, Zeitverläufe, Stellgrößenfolge
js/i18n.js          Texte in Deutsch und Englisch
js/app.js           Simulationsschleife, Bedienelemente, Störungen
astro/              Beispielkomponente für die Einbettung in eine Astro-Seite
```

Über `window.IPM.app` sind Simulationszustand, Regler und Parameter in der Browser-Konsole
zugänglich — praktisch zum Experimentieren:

```js
IPM.app.ctrl.configure({ q: [10, 1, 300, 10], R: 0.05 });
IPM.app.kick(1);
IPM.app.result();     // letzte QP-Lösung inkl. Rechenzeit
```

## URL-Parameter

| Parameter | Werte | Bedeutung |
| --- | --- | --- |
| `lang` | `de`, `en` | Sprache der Oberfläche |
| `theme` | `dark`, `light` | Farbschema |
| `embed` | `1` | blendet Kopfzeile, Erklärung und Fußzeile aus |

Beispiel: `index.html?lang=en&theme=light&embed=1`

## Einbettung in eine Astro-Website

**Variante A — als iframe (empfohlen).** Sie hält die Styles der Demo von denen der Website
getrennt und ist gegen Aktualisierungen unempfindlich.

1. Den Inhalt dieses Repos nach `public/pendulum/` der Astro-Seite kopieren
   (`index.html`, `styles.css`, `js/`).
2. `astro/InvertedPendulumMpc.astro` nach `src/components/` kopieren.
3. In einer Seite einbinden:

```astro
---
import InvertedPendulumMpc from '../components/InvertedPendulumMpc.astro';
---
<InvertedPendulumMpc lang="de" embed={true} height="820px" />
```

Da GitHub Pages die Seite meist unter einem Unterpfad ausliefert, verwendet die Komponente
`import.meta.env.BASE_URL` — der Pfad stimmt damit auch bei einem gesetzten `base` in
`astro.config.mjs`.

**Variante B — direkt in die Seite.** Den Inhalt von `<body>` (ohne die `<script>`-Zeilen)
in eine `.astro`-Komponente übernehmen, `styles.css` importieren und die Skripte am Ende mit
`<script is:inline src={...}>` in der Reihenfolge `linalg, model, mpc, i18n, render, app`
laden. `is:inline` ist wichtig, weil Astro Skripte sonst bündelt und als ES-Module ausliefert
— dann greifen die globalen Namen nicht mehr. Bei dieser Variante sollten die Selektoren der
Demo (`.card`, `.btn`, `.field` …) gegen die eigenen Website-Styles geprüft werden.

## Grenzen

* Der volle Zustand gilt als messbar; real käme ein Beobachter (Kalman-Filter) dazu.
* Kein Aufschwingen aus der Hängelage — das ist kein QP mehr.
* Der Aktor ist ideal: keine Totzeit, keine Dynamik, keine Quantisierung.
* Es gibt keine Zustandsbeschränkungen, die Schiene ist also unendlich lang.

## Lizenz

MIT — siehe [LICENSE](LICENSE).
