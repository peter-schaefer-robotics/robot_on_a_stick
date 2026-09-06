#!/usr/bin/env python3
"""Extrahiert allen sichtbaren Text aus index.html (+ den dynamischen Strings
aus js/app.js) in ein editierbares Markdown-File."""
import os
import re, html
from html.parser import HTMLParser

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, 'index.html')
OUT = os.path.join(ROOT, 'TEXT.md')
APP = os.path.join(ROOT, 'js', 'app.js')

VOID = {'br', 'img', 'input', 'meta', 'link', 'hr'}


class Node:
    def __init__(self, tag=None, attrs=None):
        self.tag = tag
        self.attrs = dict(attrs or [])
        self.kids = []
        self.parent = None

    def add(self, n):
        n.parent = self
        self.kids.append(n)


class Text(Node):
    def __init__(self, text):
        super().__init__('#text')
        self.text = text


class Builder(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.root = Node('#root')
        self.cur = self.root

    def handle_starttag(self, tag, attrs):
        n = Node(tag, attrs)
        self.cur.add(n)
        if tag not in VOID:
            self.cur = n

    def handle_startendtag(self, tag, attrs):
        self.cur.add(Node(tag, attrs))

    def handle_endtag(self, tag):
        if tag in VOID:
            return
        n = self.cur
        while n is not self.root and n.tag != tag:
            n = n.parent
        if n is not self.root:
            self.cur = n.parent

    def handle_data(self, data):
        self.cur.add(Text(data))


def find_all(node, tag=None, cls=None, id_=None, out=None):
    out = [] if out is None else out
    for k in node.kids:
        if isinstance(k, Text):
            continue
        ok = True
        if tag and k.tag != tag:
            ok = False
        if cls and cls not in k.attrs.get('class', '').split():
            ok = False
        if id_ and k.attrs.get('id') != id_:
            ok = False
        if ok:
            out.append(k)
        find_all(k, tag, cls, id_, out)
    return out


def find(node, **kw):
    r = find_all(node, **kw)
    return r[0] if r else None


def wrap(content, marker):
    """Auszeichnung setzen, ohne die umgebenden Leerzeichen zu verschieben."""
    lead = ' ' if content[:1].isspace() else ''
    trail = ' ' if content[-1:].isspace() else ''
    c = content.strip()
    if not c:
        return content
    return lead + marker + c + marker + trail


def inline(node, mark_modes=True):
    """Inline-Auszeichnung -> Markdown. Fragmente, die nur in einem Modus
    erscheinen, werden als [expert: ...] / [simple: ...] gekennzeichnet."""
    parts = []
    for k in node.kids:
        cls = '' if isinstance(k, Text) else k.attrs.get('class', '')
        only = '' if isinstance(k, Text) else k.attrs.get('data-only', '')
        if isinstance(k, Text):
            parts.append(k.text)
        elif mark_modes and ('expert-only' in cls or only == 'expert'):
            parts.append(' [expert: ' + inline(k, False).strip() + '] ')
        elif mark_modes and ('simple-only' in cls or only == 'simple'):
            parts.append(' [simple: ' + inline(k, False).strip() + '] ')
        elif mark_modes and 'manual-only' in cls:
            parts.append(' [beat: ' + inline(k, False).strip() + '] ')
        elif mark_modes and 'mpc-only' in cls:
            parts.append(' [controller: ' + inline(k, False).strip() + '] ')
        elif k.tag == 'math':
            parts.append('⟨formula⟩')
        elif k.tag in ('b', 'strong'):
            parts.append(wrap(inline(k, mark_modes), '**'))
        elif k.tag in ('i', 'em'):
            parts.append(wrap(inline(k, mark_modes), '*'))
        elif k.tag == 'sub':
            parts.append('_' + inline(k, mark_modes).strip())
        elif k.tag == 'sup':
            parts.append('^' + inline(k, mark_modes).strip())
        elif k.tag == 'br':
            parts.append(' ')
        else:
            parts.append(inline(k, mark_modes))
    s = ''.join(parts).replace('\u00a0', ' ')
    return re.sub(r'[ \t\n\r]+', ' ', s)


def txt(node):
    """inline() fuer die Ausgabe: aussen getrimmt, kein Leerzeichen vor
    Satzzeichen (entsteht durch die eingeschobenen Modus-Markierungen)."""
    return re.sub(r' +([,.;:])', r'\1', inline(node).strip())


def blocks(container):
    """p / ul / ol eines Containers als Markdown-Absätze."""
    res = []
    for k in container.kids:
        if isinstance(k, Text):
            continue
        only = k.attrs.get('data-only')
        mark = ''
        if only == 'expert' or 'expert-only' in k.attrs.get('class', ''):
            mark = '[expert] '
        elif only == 'simple' or 'simple-only' in k.attrs.get('class', ''):
            mark = '[simple] '
        if k.tag == 'p':
            t = txt(k)
            if t:
                res.append(mark + t)
        elif k.tag in ('ul', 'ol'):
            bullet = '-' if k.tag == 'ul' else '1.'
            items = []
            for li in [x for x in k.kids if not isinstance(x, Text) and x.tag == 'li']:
                lm = ''
                if li.attrs.get('data-only') == 'expert':
                    lm = '[expert] '
                items.append(bullet + ' ' + lm + txt(li))
            res.append('\n'.join(items))
        elif k.tag == 'div' and 'eq-block' in k.attrs.get('class', ''):
            res.append('⟨formula block — not editable here⟩')
    return res


src = open(SRC, encoding='utf-8').read()
b = Builder()
b.feed(src)
root = b.root

L = []
A = L.append

A('# Visible text — Inverted Pendulum MPC playground')
A('')
A('Every piece of text the page shows, in the order it appears. Edit the text under each')
A('key and send the file back; the keys are what I use to put it in the right place.')
A('')
A('**Conventions**')
A('')
A('- Keep the `key:` lines exactly as they are — only change the text below them.')
A('- `**bold**` and `*italic*` are kept as emphasis.')
A('- `x_sub` marks a subscript: `u_max`, `T_s`, `q_θ`, `e_N` render as u<sub>max</sub> etc.')
A('- `⟨formula⟩` is a MathML formula, not editable here. Tell me in plain words if one')
A('  should change.')
A('- `[expert]` = shown in expert mode only, `[simple]` = simple mode only, no marker =')
A('  both. Whole sections marked `[expert]` are hidden in simple mode.')
A('- `[beat]` = only while "Beat the Controller" is on, `[controller]` = only while the')
A('  MPC is driving.')
A('- Deleting a whole entry is fine — say so and I will remove the element too.')
A('')
A('---')
A('')

# ---------------------------------------------------------------- metadata --
title = re.search(r'<title>(.*?)</title>', src, re.S).group(1).strip()
desc = re.search(r'<meta name="description" content="(.*?)">', src, re.S).group(1).strip()
A('## 1 · Page metadata')
A('')
A('Not visible on the page itself — browser tab and search results.')
A('')
A('`meta.title:`')
A('')
A(html.unescape(title))
A('')
A('`meta.description:`')
A('')
A(html.unescape(desc))
A('')

# ------------------------------------------------------------------ header --
A('## 2 · Header')
A('')
h1 = find(root, tag='h1')
A('`header.h1:`')
A('')
A(txt(h1))
A('')
lead = find(root, cls='lead')
A('`header.intro:`  (the loose 2–3 sentence opener)')
A('')
A(txt(lead))
A('')

# ------------------------------------------------- cost function / formula --
A('## 3 · Cost function card')
A('')
eqcard = find(root, cls='eq-card')
A('`eq.cardtitle:`')
A('')
A(txt(find(eqcard, tag='h2')))
A('')
A('The formula itself is built from MathML plus the editable matrix fields:')
A('')
A('> min over u_0 … u_N−1 of  J = Σ ( e_k^⊤ **Q** e_k + R u_k² ) + e_N^⊤ **P** e_N')
A('')
A('The little column headers above each matrix are the state names: `p`, `ṗ`, `θ`, `θ̇`,')
A('and the letters below the brackets are `Q`, `R`, `P`.')
A('')
pauto = find(eqcard, id_='pAuto')
lbl = pauto.parent if pauto else None
A('`eq.pauto.label:`  [expert]')
A('')
A(txt(lbl.parent if lbl and lbl.tag == 'input' else lbl))
A('')
A('`eq.constraints.label:`  [expert]  (followed by three MathML constraints)')
A('')
A(txt(find(eqcard, cls='st-label')))
A('')

# --------------------------------------------------------- state vector ----
A('## 4 · "What the controller is looking at"')
A('')
defcard = find(root, cls='def-card')
A('`state.cardtitle:`')
A('')
A(txt(find(defcard, tag='h2')))
A('')
A('`state.vector.rows:`  (the four lines next to the state vector ⟨p, ṗ, θ, θ̇⟩)')
A('')
for li in find(defcard, cls='vd-desc').kids:
    if not isinstance(li, Text) and li.tag == 'li':
        A('- ' + txt(li))
A('')
A('`state.symbols:`  (one entry per row; the formula on the left is fixed)')
A('')
for li in [k for k in find(defcard, cls='sym-list').kids
           if not isinstance(k, Text) and k.tag == 'li']:
    mark = '[expert] ' if li.attrs.get('data-only') == 'expert' else ''
    A('- ' + mark + txt(li))
A('')

# --------------------------------------------------------------- presets ---
A('## 5 · Presets')
A('')
A('`presets.cardtitle:`')
A('')
A(txt(find_all(defcard, tag='h2')[1]))
A('')
A('`presets.buttons:`  (name — subtitle)')
A('')
for btn in find_all(root, cls='preset'):
    mark = '[expert] ' if btn.attrs.get('data-only') == 'expert' else ''
    name = txt(find(btn, tag='b'))
    subs = [k for k in btn.kids if not isinstance(k, Text) and k.tag == 'span']
    A('- ' + mark + '**' + name + '** — ' + (txt(subs[0]) if subs else ''))
A('')
_ph = find(defcard, cls='hint')
if _ph is not None:
    A('`presets.hint:`')
    A('')
    A(txt(_ph))
    A('')

# ----------------------------------------------------------- mode button ---
A('## 6 · Mode buttons')
A('')
A('Two buttons side by side, each with a label and a subtitle. The labels live in')
A('`js/app.js`, the note below them in `index.html`.')
A('')
appjs = open(APP, encoding='utf-8').read()
m = re.search(r"expert \? 'Simple mode' : 'Expert mode';", appjs)
A('`mode.button.toExpert:`  (shown while in simple mode)')
A('')
A('**Expert mode** — show the model, all tuning parameters and diagnostics')
A('')
A('`mode.button.toSimple:`  (shown while in expert mode)')
A('')
A('**Simple mode** — back to the essentials')
A('')
A('`mode.button.toBeat:`  (second button, shown while the controller is running)')
A('')
A('**Beat the Controller** — switch the controller off and balance the rod yourself')
A('')
A('`mode.button.toController:`  (same button, shown while you are in control)')
A('')
A('**Give me the controller back** — hand the rod back to the MPC')
A('')
A('`mode.beatnote:`  [beat]  (the box under the buttons — this is what tells the user to')
A('drag the cart)')
A('')
A(txt(find(root, cls='mode-note')))
A('')

# ----------------------------------------------------------------- stage ---
A('## 7 · Animation')
A('')
A('`stage.readout:`  (labels of the readout box; the values are numbers)')
A('')
hud = find(root, id_='hud')
for row in [k for k in hud.kids if not isinstance(k, Text) and 'hud-row' in k.attrs.get('class', '')]:
    sp = find(row, tag='span')
    if sp is None:
        continue
    rcls = row.attrs.get('class', '')
    mark = '[expert] ' if row.attrs.get('data-only') == 'expert' else ''
    if 'manual-only' in rcls:
        mark = '[beat] '
    elif 'mpc-only' in rcls:
        mark = '[controller] '
    A('- ' + mark + txt(sp))
A('')
A('`stage.status:`  [expert]  (status line under the readout, one of two)')
A('')
A('- ⚠ input bound active')
A('- ✓ exact (bounds inactive)')
A('')
A('`stage.buttons:`')
A('')
tb = find(root, cls='toolbar')
for btn in [k for k in tb.kids if not isinstance(k, Text) and k.tag == 'button']:
    A('- ' + txt(btn))
A('- Start / Pause toggle on the first button (set from `js/app.js`)')
A('')
A('`stage.checkboxes:`')
A('')
for lab in [k for k in tb.kids if not isinstance(k, Text) and 'chk' in k.attrs.get('class', '')]:
    mark = '[expert] ' if lab.attrs.get('data-only') == 'expert' else ''
    A('- ' + mark + txt(lab))
A('')
A('`stage.hint:`')
A('')
A(txt(find(find(root, cls='stage-card'), cls='hint')))
A('')

# ----------------------------------------------------------------- plots ---
A('## 8 · Plots  [expert]')
A('')
cards = find_all(root, tag='section', cls='card')
plot_cards = [c for c in cards if find(c, id_='scope') or find(c, id_='uplan')]
A('`plots.scope.title:`')
A('')
A(txt(find(plot_cards[0], tag='h2')))
A('')
A('`plots.scope.legend:`  (the three trace labels)')
A('')
for lg in find_all(plot_cards[0], cls='lg'):
    A('- ' + txt(lg))
A('')
A('`plots.uplan.title:`')
A('')
A(txt(find(plot_cards[1], tag='h2')))
A('')
A('`plots.uplan.hint:`')
A('')
A(txt(find(plot_cards[1], cls='hint')))
A('')

# -------------------------------------------------------------- settings ---
A('## 9 · Settings  [expert]')
A('')
sg = find(root, cls='settings-grid')
setting_cards = [k for k in sg.kids if not isinstance(k, Text) and k.tag == 'section']
for ci, card in enumerate(setting_cards):
    key = 'controller' if ci == 0 else 'plant'
    A('`settings.' + key + '.title:`')
    A('')
    A(txt(find(card, tag='h2')))
    A('')
    A('`settings.' + key + '.labels:`')
    A('')
    for f in find_all(card, cls='field'):
        lab = find(f, tag='label')
        if lab is None:
            continue
        sp = find(lab, tag='span')
        A('- ' + txt(sp if sp is not None else lab))
    for chk in find_all(card, cls='chk'):
        A('- ' + txt(chk))
    sel = find(card, tag='select')
    if sel:
        A('')
        A('`settings.' + key + '.options:`  (dropdown entries)')
        A('')
        for op in find_all(sel, tag='option'):
            A('- ' + txt(op))
    hints = find_all(card, cls='hint')
    if hints:
        A('')
        A('`settings.' + key + '.hint:`')
        A('')
        for h in hints:
            A(txt(h))
    A('')

# ---------------------------------------------------------- explanations ---
A('## 10 · Explanations')
A('')
A('Sections marked `[expert]` do not appear in simple mode at all. Inside the shared')
A('sections, single paragraphs or fragments marked `[expert]` are dropped in simple mode.')
A('')
ex = find(root, cls='explain')
A('`explain.title:`')
A('')
A(txt(find(ex, cls='explain-title')))
A('')
for i, det in enumerate(find_all(ex, tag='details'), 1):
    mark = ' [expert]' if det.attrs.get('data-only') == 'expert' else ''
    summ = find(det, tag='summary')
    slug = re.sub(r'[^a-z0-9]+', '-', txt(summ).lower()).strip('-')[:28]
    A('### ' + str(i) + '. ' + txt(summ) + mark)
    A('')
    A('`explain.' + slug + '.title:`')
    A('')
    A(txt(summ))
    A('')
    A('`explain.' + slug + '.body:`')
    A('')
    body = find(det, tag='div')
    for blk in blocks(body):
        A(blk)
        A('')

# ---------------------------------------------------------------- footer ---
A('## 11 · Footer')
A('')
A('`footer.text:`')
A('')
A(txt(find(root, cls='foot')))
A('')

# -------------------------------------------------------------- tooltips ---
A('## 12 · Tooltips')
A('')
A('Shown on hover, so worth getting right too.')
A('')
A('`tooltip.theme:`  Toggle light / dark')
A('')
A('`tooltip.matrices:`')
A('')
for mx in find_all(root, cls='mx'):
    t = mx.attrs.get('title')
    if t:
        A('- ' + t)
A('')
A('`tooltip.qfields:`  (one per editable Q entry)')
A('')
for i in range(4):
    inp = find(root, id_='q' + str(i))
    if inp is not None and inp.attrs.get('title'):
        A('- ' + inp.attrs['title'])
A('')

# ----------------------------------------------------------------- units ---
A('## 13 · Units and number formats')
A('')
A('Appended to the numeric readouts in `js/app.js`. Change only if you want different')
A('wording — the units themselves have to match the physics.')
A('')
A('- Horizon: `40  (0.80 s)` · Sample time: `20 ms  (50 Hz)` · Input bound: `15.0 N`')
A('- Masses `kg`, length `m`, friction `Ns/m`, disturbance strength `1.00×`')
A('- Measurement noise: `off` when zero, otherwise `5.0 mm`')
A('- Controller model error: `+50 %`, target position: `0.00 m`')
A('- Readout: angle `-0.3°`, force `0.31 N`, position `0.021 m`, solve time `0.084 ms`')
A('')

open(OUT, 'w', encoding='utf-8').write('\n'.join(L).rstrip() + '\n')
print('written:', OUT, len(L), 'lines')
