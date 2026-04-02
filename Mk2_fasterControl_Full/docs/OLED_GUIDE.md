# Affichage OLED & encodeur rotatif — Guide d'utilisation

Lorsque `DisplayType::OLED` est sélectionné, le système utilise un **écran OLED I2C SSD1306 128x64** (sur A4/A5) couplé à un **encodeur rotatif avec bouton poussoir** pour naviguer entre les pages et modifier les paramètres en temps réel.

## Carrousel de pages

L'affichage est organisé sous forme d'un **carrousel circulaire de pages**, construit dynamiquement au démarrage en fonction de la configuration à la compilation. Tournez l'encodeur pour faire défiler les pages.

| Page | Condition | Contenu |
|------|-----------|---------|
| **ENERGIE** | toujours | Énergie déviée en kWh (grande police) |
| **INFO RESEAU** | toujours | Tension, puissance réseau instantanée, puissance moyenne relais, puissance déviée |
| **TEMPERATURES** | `TEMP_SENSOR_PRESENT` | Jusqu'à 6 capteurs de température |
| **ROUTAGE** | toujours | Interrupteurs ON/OFF par groupe de déviation |
| **BOOST** | un par entrée visible | Interrupteur ON/OFF par canal de boost |
| **RELAIS CFG** | un par relais, si `RELAY_DIVERSION` | Modification des seuils surplus/import et minuteries ON/OFF |
| **REQ / DIV** | `OLED_ENABLE_RUNTIME_SETTINGS` | Modification de `REQUIRED_EXPORT_IN_WATTS` et `DIVERSION_START_THRESHOLD_WATTS` |
| **RESTART** | `OLED_ENABLE_RESTART_PAGE` | Redémarrage logiciel via watchdog |

Seules les pages pertinentes pour la configuration actuelle sont affichées. Par exemple, les pages TEMPERATURES n'apparaissent que si des capteurs sont configurés, les pages RELAIS CFG uniquement si la déviation par relais est activée, et les pages BOOST uniquement pour les entrées avec `visibleOnOLED: true`.

## Modèle d'interaction

L'interface dispose de trois modes, pilotés par la rotation de l'encodeur et les appuis sur le bouton :

### 1. Mode AFFICHAGE (par défaut)
- **Tourner** : défiler entre les pages (boucle circulaire)
- **Appui court** : entrer dans l'action de la page (dépend du type de page)
- **Appui long (3 s)** : retour à la page d'accueil (ENERGIE)

### 2. Mode NAVIGATION (pages ROUTAGE, RELAIS CFG, REQ / DIV)
- **Tourner** : déplacer le curseur (`>`) entre les éléments de la liste
- **Appui court sur la page ROUTAGE** : basculer le groupe de déviation sélectionné ON/OFF
- **Appui court sur la page RELAIS CFG / REQ / DIV** :
  - Sur un élément de valeur : passer en mode MODIFICATION
  - Sur « VALIDER » : sauvegarder toutes les valeurs en EEPROM et revenir en mode AFFICHAGE
- **Appui long (3 s)** : annuler toute modification non sauvegardée et revenir à l'accueil

### 3. Mode MODIFICATION (pages RELAIS CFG, REQ / DIV)
- **Tourner** : ajuster la valeur sélectionnée
  - Seuils en watts : pas de **10 W** par cran
  - Minuteries en minutes : pas de **1 min** par cran
- **Appui court** : confirmer la modification, revenir en mode NAVIGATION
- **Appui long (3 s)** : **annuler** la modification (restaurer la valeur précédente) et revenir à l'accueil

### Indicateurs visuels

- Préfixe `>` : le curseur pointe vers cet élément (mode NAVIGATION)
- Préfixe `*` : cet élément est en cours de modification (mode MODIFICATION)

Chaque ligne OLED fait 16 caractères de large (police 8x8 pixels).

```
  Page ROUTAGE             Page RELAIS CFG          Page REQ / DIV

+----------------+      +----------------+      +----------------+
|    ROUTAGE     |      |   RELAIS 1     |      |   REQ / DIV    |
|                |      |                |      |                |
| >Div 1  ON     |      | > SM  : 1000W  |      | > REQ :    0W  |
|  Div 2  OFF    |      |   SA  :  200W  |      |   DIV :    0W  |
|  Div 3  ON     |      |   Ton :   10m  |      |                |
|                |      |   Toff:   10m  |      |   VALIDER      |
|                |      |   VALIDER      |      | Act. / 3s back |
| Act. / 3s bk   |      | Act. / 3s back |      +----------------+
+----------------+      +----------------+


  Page ENERGIE             Page INFO RESEAU         Page BOOST

+----------------+      +----------------+      +----------------+
|    ENERGIE     |      |  INFO RESEAU   |      |   BOOST 1      |
|                |      |                |      |                |
|                |      |  U  230.15 V   |      | Sortie TRIAC 1 |
|    1 . 2 3 4   |      |  Pinst  -120W  |      |                |
|                |      |                |      | Etat : OFF     |
|      kWh       |      |  PmRel   850W  |      |                |
|                |      |  Pdiv    950W  |      | Clic=action    |
+----------------+      +----------------+      +----------------+
```

## Pages Boost

Chaque entrée boost avec `visibleOnOLED: true` possède sa propre page BOOST. La page affiche :
- Le numéro du canal boost
- La sortie cible (TRIAC ou relais)
- L'état actuel (ON/OFF)

Un **appui court** bascule le boost ON/OFF immédiatement. L'état du boost est combiné en OU avec l'état de la broche physique : si le basculeur OLED ou la broche matérielle est actif, le boost est ON.

## Page Routage

Chaque groupe de déviation avec `visibleOnOLED: true` apparaît comme une ligne sur la page ROUTAGE. Un appui court entre en mode navigation, puis un second appui bascule le groupe sélectionné. L'état de déviation est combiné en ET avec l'état de la broche physique : le basculeur OLED et la broche matérielle doivent tous deux être actifs pour que le routage soit autorisé.

## Persistance EEPROM

Lorsque `OLED_ENABLE_RUNTIME_SETTINGS` est `true`, les valeurs modifiables (seuils/minuteries des relais, paramètres système en watts) sont sauvegardées en EEPROM lorsque l'utilisateur confirme via l'élément « VALIDER ». Les données EEPROM incluent une signature, un octet de version et une somme de contrôle. Au démarrage, les valeurs sauvegardées sont chargées si elles sont valides ; sinon, les valeurs par défaut définies à la compilation dans `config.h` sont utilisées.

> [!NOTE]
> Les états des basculeurs boost et déviation ne sont **pas** sauvegardés en EEPROM. Ils sont réinitialisés à leurs valeurs par défaut à chaque redémarrage (tous les boosts OFF, toutes les déviations autorisées).
