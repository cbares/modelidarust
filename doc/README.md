# Documentation de référence

Liens vers les documents officiels des composants utilisés. Les photos (`2026*.jpg`)
et le schéma Fritzing (`modelidar.fzz`) sont propres au projet.

## Microcontrôleurs et cartes

| Document | Lien |
|----------|------|
| STM32F446RE, datasheet (DS10693) | <https://www.st.com/resource/en/datasheet/stm32f446re.pdf> |
| STM32F446, manuel de référence (RM0390) | <https://www.st.com/resource/en/reference_manual/dm00135183.pdf> |
| STM32F411RE, datasheet (DS10314) | <https://www.st.com/resource/en/datasheet/stm32f411re.pdf> |
| STM32F411, manuel de référence (RM0383) | <https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf> |
| STM32L476RG, datasheet (DS10198) | <https://www.st.com/resource/en/datasheet/stm32l476rg.pdf> |
| Nucleo-64 MB1136, manuel (UM1724) | <https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf> |
| Nucleo-64 MB1136, schéma (rév. C-04) | <https://www.st.com/resource/en/schematic_pack/mb1136-default-c04_schematic.pdf> |
| Nucleo-64 G4 MB1367, manuel (UM2505), pour la future G431 | <https://www.st.com/resource/en/user_manual/dm00556337.pdf> |

Pages produit, avec tous les documents (errata, notes d'application) :
[STM32F446RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html),
[STM32F411RE](https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html),
[STM32L476RG](https://www.st.com/en/microcontrollers-microprocessors/stm32l476rg.html).

## Driver moteurs DBH-12V / DBH-1A

Module générique sans datasheet constructeur. Le revendeur (RAM Electronics) a retiré
sa fiche produit ; ses caractéristiques utiles sont reprises dans le brochage
[`pinouts/driver.py`](../pinouts/driver.py) : 5–12 V (14,6 V max), 30 A par voie,
PWM 0–98 %, 16 kHz recommandé pour un moteur classique, sortie CT = 0,155 V/A.

Composants du module :

| Composant | Rôle | Lien |
|-----------|------|------|
| IR2104(S) | driver de demi-pont | <https://www.infineon.com/assets/row/public/documents/24/49/infineon-ir2104-ds-en.pdf> ([page produit](https://www.infineon.com/part/IR2104S)) |
| LM393 | conversion des niveaux logiques | <https://www.ti.com/lit/ds/symlink/lm393.pdf> |
| LM358 | ampli de mesure de courant (CT) | <https://www.ti.com/lit/ds/symlink/lm358.pdf> |
| MC34063A | élévateur 13,8 V pour les drivers | <https://www.ti.com/lit/ds/symlink/mc34063a.pdf> |
| STD70N03L | MOSFET du pont en H | composant ST obsolète, plus de datasheet en ligne |
