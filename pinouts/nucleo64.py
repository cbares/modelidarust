"""Dessin commun des cartes Nucleo-64 (connecteurs morpho CN7 / CN10) avec pinout.

Chaque script de carte (f446.py, f411.py, l476.py) appelle `build()` avec le modèle
de carte de base (MB1136...), la liste des broches utilisées par le firmware, et
expose la variable `diagram`.

Les signaux sont repérés par le nom de la broche MCU (« PA8 ») : leur position sur
CN7 / CN10 est déduite du brochage du modèle de carte. Seules les alimentations et
masses, présentes plusieurs fois, sont repérées par leur position (« CN7 », 18).

Doc de la bibliothèque : https://pinout.readthedocs.io/
"""

from pinout.core import Group, Rect, Text
from pinout.components.layout import Diagram_2Rows
from pinout.components.legend import Legend
from pinout.components.pinlabel import PinLabelGroup
from pinout.components.text import TextBlock

# Connecteurs morpho par modèle de carte.
# Index = numéro de broche - 1. Broches impaires à gauche, paires à droite.

# MB1136 : Nucleo-F446RE, F411RE, L476RG (manuel UM1724).
MB1136 = {
    "CN7": [
        "PC10", "PC11", "PC12", "PD2", "VDD", "E5V", "BOOT0", "GND", "NC", "NC",
        "NC", "IOREF", "PA13", "RESET", "PA14", "+3V3", "PA15", "+5V", "GND", "GND",
        "PB7", "GND", "PC13", "VIN", "PC14", "NC", "PC15", "PA0", "PH0", "PA1",
        "PH1", "PA4", "VBAT", "PB0", "PC2", "PC1", "PC3", "PC0",
    ],
    "CN10": [
        "PC9", "PC8", "PB8", "PC6", "PB9", "PC5", "AVDD", "U5V", "GND", "NC",
        "PA5", "PA12", "PA6", "PA11", "PA7", "PB12", "PB6", "NC", "PC7", "GND",
        "PA9", "PB2", "PA8", "PB1", "PB10", "PB15", "PB4", "PB14", "PB5", "PB13",
        "PB3", "AGND", "PA10", "PC4", "PA2", "NC", "PA3", "NC",
    ],
}

# MB1367 : Nucleo-G431RB, G474RE... (manuel UM2505). À remplir depuis le manuel
# avant d'ajouter g431.py : le brochage morpho diffère de la MB1136 par endroits.
BASES = {
    "MB1136": MB1136,
}

LEGEND = [
    ("Broche MCU", "pin"),
    ("Moteurs", "motor"),
    ("Codeurs", "encoder"),
    ("Radio SBUS", "rc"),
    ("USB série", "uart"),
    ("Carte", "board"),
    ("Alim. 5 V", "pwr"),
    ("Masse", "gnd"),
]

# Usages communs aux cartes : broche -> [(fonction, tag), (rôle, tag)].
# Clé = nom de broche MCU, ou (connecteur, numéro) pour une alimentation / masse.
COMMON = {
    ("CN7", 18): [("Codeur G", "pwr")],
    ("CN7", 20): [("Codeur G", "gnd")],
    ("CN10", 8): [("Codeur D", "pwr")],
    ("CN10", 9): [("Codeur D", "gnd")],
    ("CN10", 20): [("Driver", "gnd")],
    "PC13": [("GPIO", "board"), ("Bouton B1", "board")],
    "PA5": [("GPIO", "board"), ("LED LD2", "board")],
    "PA0": [("TIM5_CH1", "encoder"), ("Codeur G · A", "encoder")],
    "PA1": [("TIM5_CH2", "encoder"), ("Codeur G · B", "encoder")],
    "PA8": [("TIM1_CH1", "motor"), ("IN1A · mot. D", "motor")],
    "PA9": [("TIM1_CH2", "motor"), ("IN2A · mot. D", "motor")],
    "PA10": [("TIM1_CH3", "motor"), ("IN1B · mot. G", "motor")],
    "PA11": [("TIM1_CH4", "motor"), ("IN2B · mot. G", "motor")],
    "PC4": [("GPIO", "motor"), ("ENA · mot. D", "motor")],
    "PB3": [("GPIO", "motor"), ("ENB · mot. G", "motor")],
    "PA2": [("USART2_TX", "uart"), ("VCP ST-Link", "uart")],
    "PA3": [("USART2_RX", "uart"), ("VCP ST-Link", "uart")],
}

POWER_PINS = {"GND", "+5V", "U5V", "E5V", "+3V3", "VIN", "AVDD", "AGND", "IOREF"}

# Géométrie (px)
PITCH = 30  # pas entre rangées de broches
COL = 28  # écart entre les deux colonnes d'un connecteur
WIDTHS = (52, 92, 128)  # largeur des étiquettes : broche, fonction, rôle
LABEL_START = 26  # longueur du trait entre la broche et la 1re étiquette
LABEL_SPAN = LABEL_START + sum(w + 6 for w in WIDTHS)  # place prise par une rangée d'étiquettes
MARGIN = 16
TOP = 64  # place au-dessus des connecteurs (titres CN7 / CN10)
ROWS = 19

CN7_X = MARGIN + LABEL_SPAN + 14
CN10_X = CN7_X + COL + 2 * LABEL_SPAN + 40
WIDTH = CN10_X + COL + LABEL_SPAN + MARGIN + 14
PANEL_01_H = TOP + ROWS * PITCH + 44
HEIGHT = PANEL_01_H + 160


def pin_xy(connector, number):
    """Coordonnées d'une broche dans le groupe graphique."""
    x = CN7_X if connector == "CN7" else CN10_X
    if number % 2 == 0:
        x += COL
    y = TOP + ((number - 1) // 2) * PITCH
    return x, y


def label_direction(connector, number):
    """Les étiquettes partent vers l'extérieur pour la colonne externe,
    vers le centre de la carte pour la colonne interne."""
    odd = number % 2 == 1
    if connector == "CN7":
        return -1 if odd else 1
    return -1 if odd else 1


def draw_board(graphic, base):
    top = TOP - 30
    bottom = TOP + (ROWS - 1) * PITCH + 26
    # Carte
    graphic.add(
        Rect(
            x=CN7_X - 34,
            y=top - 34,
            width=CN10_X + COL + 34 - (CN7_X - 34),
            height=bottom - top + 44,
            corner_radius=10,
            tag="board-outline",
        )
    )
    # Référence de la carte, en bas entre les connecteurs
    graphic.add(
        Text(f"Nucleo-64 · {base}", x=(CN7_X + CN10_X + COL) / 2, y=bottom + 6, tag="board-ref")
    )
    # Connecteurs et broches
    for name, x in (("CN7", CN7_X), ("CN10", CN10_X)):
        graphic.add(
            Rect(
                x=x - 12,
                y=TOP - 14,
                width=COL + 24,
                height=(ROWS - 1) * PITCH + 28,
                corner_radius=2,
                tag="header",
            )
        )
        graphic.add(Text(name, x=x + COL / 2, y=top - 18, tag="connector-name"))
        graphic.add(Text("1", x=x, y=TOP - 22, tag="pin-number"))
        graphic.add(Text("2", x=x + COL, y=TOP - 22, tag="pin-number"))
        graphic.add(Text("37", x=x, y=bottom + 6, tag="pin-number"))
        graphic.add(Text("38", x=x + COL, y=bottom + 6, tag="pin-number"))
        for number in range(1, 2 * ROWS + 1):
            px, py = pin_xy(name, number)
            graphic.add(Rect(x=px - 6, y=py - 6, width=12, height=12, tag="pad"))


def resolve(morpho, key):
    """Position (connecteur, numéro) d'une broche MCU ou d'une alimentation."""
    if isinstance(key, tuple):
        connector, number = key
        name = morpho[connector][number - 1]
        assert name in POWER_PINS, f"{connector}-{number} est {name}, pas une alimentation"
        return key
    found = [(c, i + 1) for c, pins in morpho.items() for i, n in enumerate(pins) if n == key]
    assert len(found) == 1, f"{key} absente ou en double sur les connecteurs morpho"
    return found[0]


def add_label(graphic, morpho, connector, number, usage):
    pin_name = morpho[connector][number - 1]
    pin_tag = {"GND": "gnd", "+5V": "pwr", "U5V": "pwr"}.get(pin_name, "pin")
    row = [(pin_name, pin_tag)] + list(usage)
    labels = [
        (content, tag, {"body": {"width": WIDTHS[i]}})
        for i, (content, tag) in enumerate(row)
    ]
    x, y = pin_xy(connector, number)
    graphic.add(
        PinLabelGroup(
            x=x,
            y=y,
            pin_pitch=(0, PITCH),
            label_start=(LABEL_START, 0),
            label_pitch=(0, PITCH),
            scale=(label_direction(connector, number), 1),
            labels=[labels],
        )
    )


def build(board, base, specific, notes):
    """Construit le diagramme d'une carte.

    board    : nom de la carte (« Nucleo-F446RE »)
    base     : modèle de carte de base, clé de BASES (« MB1136 »)
    specific : usages propres à la carte, même format que COMMON
    notes    : lignes de texte affichées sous le schéma
    """
    morpho = BASES[base]
    usages = {resolve(morpho, k): v for k, v in {**COMMON, **specific}.items()}

    diagram = Diagram_2Rows(WIDTH, HEIGHT, PANEL_01_H, "diagram")
    diagram.add_stylesheet("styles.css", embed=True)

    graphic = diagram.panel_01.add(Group(0, 12))
    draw_board(graphic, base)
    for (connector, number), usage in sorted(usages.items()):
        add_label(graphic, morpho, connector, number, usage)

    title_block = diagram.panel_02.add(
        TextBlock(
            f"<tspan class='h1'>{board} ({base}) · modelidar</tspan>",
            x=20,
            y=34,
            line_height=20,
            tag="panel title_block",
        )
    )
    diagram.panel_02.add(
        TextBlock(
            "\n".join(notes),
            x=20,
            y=68,
            width=title_block.width,
            line_height=20,
            tag="panel text_block",
        )
    )
    diagram.panel_02.add(Legend(LEGEND, x=WIDTH - 2 * 159 - 50, y=14, max_height=4 * 28))
    return diagram
