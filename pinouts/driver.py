"""Brochage du driver moteurs DBH-12V / DBH-1A (2 voies, 30 A) avec pinout.

Vue schématique : connecteur logique 2×6 (rangées A et B) et bornier de puissance.
Connexions reprises de wires/driver.yml (câble W1) et du firmware (src/board/mod.rs),
identiques pour toutes les cartes Nucleo-64.
"""

from pinout.core import Group, Rect, Text
from pinout.components.layout import Diagram_2Rows
from pinout.components.legend import Legend
from pinout.components.pinlabel import PinLabelGroup
from pinout.components.text import TextBlock

# Connecteur logique : une ligne par signal, colonnes B (gauche) et A (droite).
# Chaque entrée : [(fonction côté Nucleo, tag), (rôle firmware, tag), (fil W1, tag)]
SIGNALS = ["V+", "GND", "IN1", "IN2", "EN", "CT"]
NC = [("non relié", "nc")]
HEADER = {
    ("V+", "A"): [("tension batterie", "pwr")] + NC,
    ("V+", "B"): [("tension batterie", "pwr")] + NC,
    ("GND", "A"): [("CN10-20 · GND", "gnd"), ("masse", "gnd"), ("gris", "wire-gy")],
    ("GND", "B"): [("fil W0", "gnd"), ("masse", "gnd"), ("noir", "wire-bk")],
    ("IN1", "A"): [("CN10-23 · PA8", "motor"), ("TIM1_CH1", "motor"), ("violet", "wire-vt")],
    ("IN2", "A"): [("CN10-21 · PA9", "motor"), ("TIM1_CH2", "motor"), ("vert", "wire-gn")],
    ("EN", "A"): [("CN10-34 · PC4", "motor"), ("ENA (GPIO)", "motor"), ("rouge", "wire-rd")],
    ("IN1", "B"): [("CN10-33 · PA10", "motor"), ("TIM1_CH3", "motor"), ("bleu", "wire-bu")],
    ("IN2", "B"): [("CN10-14 · PA11", "motor"), ("TIM1_CH4", "motor"), ("jaune", "wire-ye")],
    ("EN", "B"): [("CN10-31 · PB3", "motor"), ("ENB (GPIO)", "motor"), ("orange", "wire-og")],
    ("CT", "A"): [("0,155 V/A", "sense")] + NC,
    ("CT", "B"): [("0,155 V/A", "sense")] + NC,
}

# Bornier de puissance, de haut en bas selon le schéma constructeur (à vérifier sur la carte).
TERMINALS = [
    ("A 1", "motor", [("Moteur gauche", "motor")]),
    ("A 2", "motor", [("Moteur gauche", "motor")]),
    ("PWR+", "pwr", [("Batterie + 5…12 V", "pwr")]),
    ("PWR−", "gnd", [("Batterie −", "gnd")]),
    ("B 1", "motor", [("Moteur droit", "motor")]),
    ("B 2", "motor", [("Moteur droit", "motor")]),
]

LEGEND = [
    ("Moteurs", "motor"),
    ("Masse", "gnd"),
    ("Batterie", "pwr"),
    ("Mesure courant", "sense"),
    ("Non relié", "nc"),
]

# Géométrie (px)
PITCH = 34
COL = 28
WIDTHS = (52, 124, 96, 62)  # signal, broche Nucleo, rôle, couleur du fil
LABEL_START = 26
LABEL_SPAN = LABEL_START + sum(w + 6 for w in WIDTHS)
TOP = 84
MARGIN = 16

B_X = MARGIN + LABEL_SPAN + 10
A_X = B_X + COL
TERM_X = A_X + LABEL_SPAN + 90
TERM_WIDTHS = (52, 150)
WIDTH = TERM_X + 26 + sum(w + 6 for w in TERM_WIDTHS) + 2 * MARGIN
PANEL_01_H = TOP + len(SIGNALS) * PITCH + 40
HEIGHT = PANEL_01_H + 150


def label_group(x, y, direction, row, widths):
    return PinLabelGroup(
        x=x,
        y=y,
        pin_pitch=(0, PITCH),
        label_start=(LABEL_START, 0),
        label_pitch=(0, PITCH),
        scale=(direction, 1),
        labels=[[(c, t, {"body": {"width": widths[i]}}) for i, (c, t) in enumerate(row)]],
    )


def signal_tag(signal):
    return {"V+": "pwr", "GND": "gnd", "CT": "sense"}.get(signal, "motor")


graphic = Group(0, 12)

# Carte
bottom = TOP + (len(SIGNALS) - 1) * PITCH
graphic.add(
    Rect(
        x=B_X - 40,
        y=TOP - 56,
        width=TERM_X + 40 - (B_X - 40),
        height=bottom - TOP + 92,
        corner_radius=10,
        tag="driver-outline",
    )
)
graphic.add(Text("DBH-12V · Dual DC Motor Drive", x=(B_X + TERM_X) / 2, y=bottom + 26, tag="driver-ref"))

# Connecteur logique 2×6
graphic.add(Rect(x=B_X - 12, y=TOP - 16, width=COL + 24, height=bottom - TOP + 32, corner_radius=2, tag="header"))
graphic.add(Text("logique", x=B_X + COL / 2, y=TOP - 44, tag="driver-title"))
graphic.add(Text("B", x=B_X, y=TOP - 26, tag="driver-row"))
graphic.add(Text("A", x=A_X, y=TOP - 26, tag="driver-row"))
for i, signal in enumerate(SIGNALS):
    y = TOP + i * PITCH
    for side, x, direction in (("B", B_X, -1), ("A", A_X, 1)):
        graphic.add(Rect(x=x - 6, y=y - 6, width=12, height=12, tag="pad"))
        row = [(f"{signal}{side if signal not in ('V+', 'GND') else ''}", signal_tag(signal))]
        row += HEADER[(signal, side)]
        graphic.add(label_group(x, y, direction, row, WIDTHS))

# Bornier de puissance
term_top = TOP + (len(SIGNALS) - len(TERMINALS)) * PITCH / 2
graphic.add(
    Rect(
        x=TERM_X - 14,
        y=term_top - 16,
        width=28,
        height=(len(TERMINALS) - 1) * PITCH + 32,
        corner_radius=3,
        tag="terminal-block",
    )
)
graphic.add(Text("bornier", x=TERM_X, y=TOP - 44, tag="driver-title"))
for i, (name, tag, usage) in enumerate(TERMINALS):
    y = term_top + i * PITCH
    graphic.add(Rect(x=TERM_X - 7, y=y - 7, width=14, height=14, corner_radius=7, tag="screw"))
    graphic.add(label_group(TERM_X, y, 1, [(name, tag)] + usage, TERM_WIDTHS))

# Diagramme
diagram = Diagram_2Rows(WIDTH, HEIGHT, PANEL_01_H, "diagram")
diagram.add_stylesheet("styles.css", embed=True)
diagram.panel_01.add(graphic)

title_block = diagram.panel_02.add(
    TextBlock(
        "<tspan class='h1'>Driver DBH-12V (DBH-1A) · modelidar</tspan>",
        x=20,
        y=34,
        line_height=20,
        tag="panel title_block",
    )
)
diagram.panel_02.add(
    TextBlock(
        "\n".join(
            [
                "Voie A = moteur gauche, voie B = moteur droit. Fils : câble W1 de wires/driver.yml.",
                "V+ sort la tension batterie : ne jamais la relier à la Nucleo. Entrées compatibles 3,3 V.",
                "PWM : 20 kHz, rapport cyclique max 98 % selon le constructeur.",
                "CT : 0,155 V/A, dépasse 3,3 V au-delà de 21 A (pont diviseur avant un ADC).",
            ]
        ),
        x=20,
        y=68,
        width=title_block.width,
        line_height=20,
        tag="panel text_block",
    )
)
diagram.panel_02.add(Legend(LEGEND, x=WIDTH - 2 * 159 - 40, y=14, max_height=3 * 28))
