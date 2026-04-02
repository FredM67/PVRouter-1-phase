[![en](https://img.shields.io/badge/lang-en-red.svg)](README.en.md)

Ce programme est conçu pour être utilisé avec l'IDE Arduino et/ou d'autres IDE de développement comme VSCode + PlatformIO.

- [Utilisation avec Arduino IDE](#utilisation-avec-arduino-ide)
  - [Bibliothèques requises](#bibliothèques-requises)
- [Utilisation avec Visual Studio Code](#utilisation-avec-visual-studio-code)
- [Aperçu rapide des fichiers](#aperçu-rapide-des-fichiers)
- [Documentation de développement](#documentation-de-développement)
- [Étalonnage du routeur](#étalonnage-du-routeur)
- [Configuration du programme](#configuration-du-programme)
  - [Configuration de la version du PCB](#configuration-de-la-version-du-pcb)
  - [Type de sortie série](#type-de-sortie-série)
  - [Configuration de l'affichage](#configuration-de-laffichage)
    - [Affichage OLED avec encodeur rotatif](#affichage-oled-avec-encodeur-rotatif)
  - [Configuration des sorties TRIAC](#configuration-des-sorties-triac)
  - [Configuration des sorties relais tout-ou-rien](#configuration-des-sorties-relais-tout-ou-rien)
    - [Principe de fonctionnement](#principe-de-fonctionnement)
  - [Configuration du Watchdog](#configuration-du-watchdog)
  - [Configuration du ou des capteurs de température](#configuration-du-ou-des-capteurs-de-température)
    - [Activation de la fonctionnalité](#activation-de-la-fonctionnalité)
      - [Avec l'Arduino IDE](#avec-larduino-ide)
      - [Avec Visual Studio Code et PlatformIO](#avec-visual-studio-code-et-platformio)
    - [Configuration du ou des capteurs (commun aux 2 cas précédents)](#configuration-du-ou-des-capteurs-commun-aux-2-cas-précédents)
  - [Gestion des Heures Creuses et boost programmé (dual tariff)](#gestion-des-heures-creuses-et-boost-programmé-dual-tariff)
    - [Configuration matérielle](#configuration-matérielle)
    - [Configuration logicielle](#configuration-logicielle)
    - [Configuration du boost programmé (rg_ForceLoad)](#configuration-du-boost-programmé-rg_forceload)
    - [Exemples visuels](#exemples-visuels)
    - [Configuration pour plusieurs charges](#configuration-pour-plusieurs-charges)
    - [Aide-mémoire](#aide-mémoire)
  - [Rotation des priorités](#rotation-des-priorités)
- [Configuration avancée du programme](#configuration-avancée-du-programme)
  - [Paramètre `DIVERSION_START_THRESHOLD_WATTS`](#paramètre-diversion_start_threshold_watts)
  - [Paramètre `REQUIRED_EXPORT_IN_WATTS`](#paramètre-required_export_in_watts)
- [Dépannage](#dépannage)
- [Contribuer](#contribuer)

# Utilisation avec Arduino IDE

Pour utiliser ce programme avec l'IDE Arduino, vous devez télécharger et installer la dernière version de l'IDE Arduino. Choisissez la version "standard", PAS la version du Microsoft Store. Optez pour la version "Win 10 et plus récent, 64 bits" ou la version "MSI installer".

Comme le code est optimisé avec l'une des dernières normes C++, vous devez modifier un fichier de configuration pour activer C++17. Vous trouverez le fichier '**platform.txt**' dans le chemin d'installation de l'IDE Arduino.

Pour **Windows**, vous trouverez généralement le fichier dans '**C:\Program Files (x86)\Arduino\hardware\arduino\avr**' et/ou dans '**%LOCALAPPDATA%\Arduino15\packages\arduino\hardware\avr\x.y.z**' où **'x.y.z**' est la version du package Arduino AVR Boards.

Vous pouvez également exécuter cette commande dans Powershell : `Get-Childitem –Path C:\ -Include platform.txt -Recurse -ErrorAction SilentlyContinue`. Cela peut prendre quelques secondes/minutes jusqu'à ce que le fichier soit trouvé.

Pour **Linux**, si vous utilisez le package AppImage, vous trouverez ce fichier dans '~/.arduino15/packages/arduino/hardware/avr/1.8.6'. Vous pouvez exécuter `find / -name platform.txt 2>/dev/null` au cas où l'emplacement aurait changé.

Pour **MacOSX**, ce fichier se trouve dans '/Users/[user]/Library/Arduino15/packages/arduino/hardware/avr/1.8.6'.

Ouvrez le fichier dans n'importe quel éditeur de texte (vous aurez besoin des droits d'administrateur) et remplacez le paramètre '**-std=gnu++11**' par '**-std=gnu++17**'. C'est tout !

Si votre IDE Arduino était ouvert, veuillez fermer toutes les instances et le rouvrir.

## Bibliothèques requises

Pour utiliser le projet, et selon la configuration du programme, vous aurez besoin des bibliothèques :
- ArduinoJson : il faudra rester avec la version 6.*. La version 7 n'est pas appropriée pour l'Atmega328P (Arduino Uno).
- U8g2
- OneWire

# Utilisation avec Visual Studio Code

Vous devrez installer des extensions supplémentaires. Les extensions les plus populaires et les plus utilisées pour ce travail sont '*Arduino*' et '*Platform IO*'.  
L'ensemble du projet a été conçu pour être utilisé de façon optimale avec *Platform IO*.

# Aperçu rapide des fichiers

- **Mk2_fasterControl_Full.ino** : Ce fichier est nécessaire pour l’IDE Arduino
- **calibration.h** : contient les paramètres d’étalonnage
- **config.h** : les préférences de l’utilisateur sont stockées ici (affectation des broches, fonctionnalités …)
- **config_system.h** : constantes système rarement modifiées
- **constants.h** : quelques constantes — *ne pas modifier*
- **debug.h** : Quelques macros pour la sortie série et le débogage
- **dualtariff.h** : définitions de la fonction double tarif
- **ewma_avg.h** : fonctions de calcul de moyenne EWMA modifiée
- **main.cpp** : code source principal
- **movingAvg.h** : code source pour la moyenne glissante
- **processing.cpp** : code source du moteur de traitement
- **processing.h** : prototypes de fonctions du moteur de traitement
- **README.md** : ce fichier
- **teleinfo.h**: code source de la fonctionnalité *Télémétrie IoT*
- **types.h** : définitions des types …
- **type_traits.h** : quelques trucs STL qui ne sont pas encore disponibles dans le paquet avr
- **type_traits** : contient des patrons STL manquants
- **utils_display.h** : code source de la fonctionnalité *afficheur 7-segments*
- **utils_dualtariff.h** : code source de la fonctionnalité *gestion Heures Creuses*
- **utils_oled.h** : code source de la fonctionnalité *afficheur OLED I2C*
- **utils_pins.h** : quelques fonctions d'accès direct aux entrées/sorties du micro-contrôleur
- **utils_relay.h** : code source de la fonctionnalité *diversion par relais*
- **utils_temp.h** : code source de la fonctionnalité *Température*
- **utils.h** : fonctions d’aide et trucs divers
- **validation.h** : validation des paramètres, ce code n’est exécuté qu’au moment de la compilation !
- **platformio.ini** : paramètres PlatformIO
- **inject_sketch_name.py** : script d'aide pour PlatformIO
- **Doxyfile** : paramètre pour Doxygen (documentation du code)

L’utilisateur final ne doit éditer QUE les fichiers **calibration.h** et **config.h**.

# Documentation de développement

Vous pouvez commencer à lire la documentation ici [1-phase routeur](https://fredm67.github.io/PVRouter-1-phase/) (en anglais).

# Étalonnage du routeur
Les valeurs d'étalonnage se trouvent dans le fichier **calibration.h**.
Il s'agit des lignes :
```cpp
inline constexpr float powerCal_grid{ 0.0435f };
inline constexpr float powerCal_diverted{ 0.0435f };
```

Ces valeurs par défaut n'entrent pas en jeux dans le fonctionnement du routeur.

Par contre elles doivent être déterminées précisément si on souhaite avoir un affichage cohérent avec la réalité.

# Configuration du programme

La configuration d'une fonctionnalité suit généralement deux étapes :
- Activation de la fonctionnalité
- Configuration des paramètres de la fonctionnalité

La cohérence de la configuration est vérifiée lors de la compilation. Par exemple, si une *pin* est allouée deux fois par erreur, le compilateur générera une erreur.

## Configuration de la version du PCB

Le routeur existe en deux versions de PCB (circuit imprimé), qui utilisent des broches analogiques différentes pour les capteurs de tension et de courant. Il est important de configurer correctement cette option selon la version du PCB que vous possédez.

Pour activer l'ancien PCB (par défaut) :
```cpp
inline constexpr bool OLD_PCB{ true };
```

Pour utiliser le nouveau PCB :
```cpp
inline constexpr bool OLD_PCB{ false };
```

Cette configuration modifie automatiquement l'affectation des broches analogiques :
- **Ancien PCB** : capteur de tension sur A3, CT1 (réseau) sur A5, CT2 (déviation) sur A4
- **Nouveau PCB** : capteur de tension sur A0, CT1 (réseau) sur A1, CT2 (déviation) sur A3

---
> [!WARNING]
> Une mauvaise configuration de ce paramètre empêchera le routeur de fonctionner correctement, car les capteurs ne seront pas lus sur les bonnes broches analogiques.
---

---
> [!IMPORTANT]
> **Utilisateurs de PCB triphasé en mode monophasé** : Si vous utilisez un PCB triphasé en mode monophasé (par exemple, si vous avez changé votre raccordement de triphasé à monophasé), vous **devez** configurer `OLD_PCB` sur `false`. Cela garantit que les bonnes broches analogiques sont utilisées pour les capteurs.
---

## Type de sortie série

Le type de sortie série peut être configuré pour s'adapter à différents besoins. Trois options sont disponibles :

- **HumanReadable** : Sortie lisible par un humain, idéale pour le débogage ou la mise en service.
- **IoT** : Sortie formatée pour des plateformes IoT comme HomeAssistant.
- **JSON** : Sortie formatée pour les plateformes comme EmonCMS (JSON).

Pour configurer le type de sortie série, modifiez la constante suivante dans le fichier **config.h** :
```cpp
inline constexpr SerialOutputType SERIAL_OUTPUT_TYPE = SerialOutputType::HumanReadable;
```
Remplacez `HumanReadable` par `IoT` ou `JSON` selon vos besoins.

## Configuration de l'affichage

Configurez le type d'affichage dans `config.h` :
```cpp
inline constexpr DisplayType TYPE_OF_DISPLAY{ DisplayType::OLED };
```

Les options possibles sont :
- **DisplayType::NONE** : Aucun affichage n'est utilisé.
- **DisplayType::OLED** : Utilise un écran OLED 128x64 SSD1306 avec un encodeur rotatif pour la navigation et les réglages en temps réel.
- **DisplayType::SEG** : Utilise un afficheur à segments pour afficher les informations.
- **DisplayType::SEG_HW** : Utilise un afficheur à segments avec une interface matérielle spécifique pour afficher les informations (présence des circuits **IC3** et **IC4**).

### Affichage OLED avec encodeur rotatif

Lorsque `DisplayType::OLED` est sélectionné, le système utilise un **écran OLED I2C SSD1306 128x64** (sur A4/A5) couplé à un **encodeur rotatif avec bouton poussoir** pour naviguer entre les pages et modifier les paramètres en temps réel.

L'encodeur rotatif possède 3 broches configurées dans `config.h` :
```cpp
inline constexpr OledEncoderConfig oledEncoder{ 11, 12, 13 };
```
- **Broche 1 (CLK)** : signal d'horloge de l'encodeur
- **Broche 2 (DT)** : signal de données de l'encodeur
- **Broche 3 (SW)** : bouton poussoir (INPUT_PULLUP, actif à l'état bas)

De plus, deux fonctionnalités optionnelles peuvent être activées :
```cpp
inline constexpr bool OLED_ENABLE_RUNTIME_SETTINGS{ true };
inline constexpr bool OLED_ENABLE_RESTART_PAGE{ true };
```
- **OLED_ENABLE_RUNTIME_SETTINGS** : si `true`, les seuils des relais et les paramètres système sont modifiables depuis l'OLED et sauvegardés en EEPROM
- **OLED_ENABLE_RESTART_PAGE** : si `true`, ajoute une page dédiée pour redémarrer le routeur par logiciel

Pour tous les détails sur la navigation, la description des pages, les modes d'interaction et la persistance EEPROM, consultez le **[Guide d'utilisation OLED](docs/OLED_GUIDE.md)**.

## Configuration des sorties TRIAC

La première étape consiste à définir le nombre de sorties TRIAC :

```cpp
inline constexpr uint8_t NO_OF_DUMPLOADS{ 2 };
```

Ensuite, il faudra assigner les *pins* correspondantes ainsi que l'ordre des priorités au démarrage.
```cpp
inline constexpr uint8_t physicalLoadPin[NO_OF_DUMPLOADS]{ 5, 7 };
inline constexpr uint8_t loadPrioritiesAtStartup[NO_OF_DUMPLOADS]{ 0, 1 };
```

## Configuration des sorties relais tout-ou-rien
Les sorties relais tout-ou-rien permettent d'alimenter des appareils qui contiennent de l'électronique (pompe à chaleur …).

Il faudra activer la fonctionnalité comme ceci :
```cpp
inline constexpr bool RELAY_DIVERSION{ true };
```

Chaque relais nécessite la définition de cinq paramètres :
- le numéro de **pin** sur laquelle est branché le relais
- le **seuil de surplus** avant mise en route (par défaut **1000 W**)
- le **seuil d'import** avant arrêt (par défaut **200 W**)
- la **durée de fonctionnement minimale** en minutes (par défaut **5 min**)
- la **durée d'arrêt minimale** en minutes (par défaut **5 min**).

Exemple de configuration d'un relais :
```cpp
inline constexpr RelayEngine relays{ { { 4, 1000, 200, 10, 10 } } };
```
Dans cet exemple, le relais est connecté sur la *pin* **4**, il se déclenchera à partir de **1000 W** de surplus, s'arrêtera à partir de **200 W** d'import, et a une durée minimale de fonctionnement et d'arrêt de **10 min**.

Pour configurer plusieurs relais, listez simplement les configurations de chaque relais :
```cpp
inline constexpr RelayEngine relays{ { { 4, 1000, 200, 10, 10 },
                                       { 3, 1500, 250, 5, 15 } } };
```
Les relais sont activés dans l'ordre de la liste, et désactivés dans l'ordre inverse.  
Dans tous les cas, les durées minimales de fonctionnement et d'arrêt sont toujours respectées.

### Principe de fonctionnement
Les seuils de surplus et d'import sont calculés en utilisant une moyenne mobile pondérée exponentiellement (EWMA), dans notre cas précis, il s'agit d'une modification d'une moyenne mobile triple exponentiellement pondérée (TEMA).  
Par défaut, cette moyenne est calculée sur une fenêtre d'environ **10 min**. Vous pouvez ajuster cette durée pour l'adapter à vos besoins.  
Il est possible de la rallonger mais aussi de la raccourcir.  
Pour des raisons de performances de l'Arduino, la durée choisie sera arrondie à une durée proche qui permettra de faire les calculs sans impacter les performances du routeur.

Si l'utilisateur souhaite plutôt une fenêtre de 15 min, il suffira d'écrire :
```cpp
inline constexpr RelayEngine relays{ MINUTES(15), { { 3, 1000, 200, 1, 1 } } };
```
___
> [!NOTE]
> La macro `MINUTES()` convertit automatiquement la valeur en paramètre template. Aucun suffixe spécial n'est nécessaire !
___

Les relais configurés dans le système sont gérés par un système similaire à une machine à états.
Chaque seconde, le système augmente la durée de l'état actuel de chaque relais et procède avec tous les relais en fonction de la puissance moyenne actuelle :
- si la puissance moyenne actuelle est supérieure au seuil d'import, elle essaie d'éteindre certains relais.
- si la puissance moyenne actuelle est supérieure au seuil de surplus, elle essaie d'allumer plus de relais.

Les relais sont traités dans l'ordre croissant pour le surplus et dans l'ordre décroissant pour l'importation.

Pour chaque relais, la transition ou le changement d'état est géré de la manière suivante :
- si le relais est *OFF* et que la puissance moyenne actuelle est inférieure au seuil de surplus, le relais essaie de passer à l'état *ON*. Cette transition est soumise à la condition que le relais ait été *OFF* pendant au moins la durée *minOFF*.
- si le relais est *ON* et que la puissance moyenne actuelle est supérieure au seuil d'importation, le relais essaie de passer à l'état *OFF*. Cette transition est soumise à la condition que le relais ait été *ON* pendant au moins la durée *minON*.

## Configuration du Watchdog
Un chien de garde, en anglais *watchdog*, est un circuit électronique ou un logiciel utilisé en électronique numérique pour s'assurer qu'un automate ou un ordinateur ne reste pas bloqué à une étape particulière du traitement qu'il effectue.

Ceci est réalisé à l'aide d'une LED qui clignote à la fréquence de 1 Hz, soit toutes les secondes.  
Ainsi, l'utilisateur sait d'une part si son routeur est allumé, et si jamais cette LED ne clignote plus, c'est que l'Arduino s'est bloqué (cas encore jamais rencontré !).  
Un simple appui sur le bouton *Reset* permettra de redémarrage le système sans rien débrancher.

Il faudra activer la fonctionnalité comme ceci :
```cpp
inline constexpr bool WATCHDOG_PIN_PRESENT{ true };
```
et définir la *pin* utilisée, dans l'exemple la *9* :
```cpp
inline constexpr uint8_t watchDogPin{ 9 };
```

## Configuration du ou des capteurs de température
Il est possible de brancher un ou plusieurs capteurs de température Dallas DS18B20.  
Ces capteurs peuvent servir à des fins informatives ou pour contrôler le mode de fonctionnement forcé.

Pour activer cette fonctionnalité, il faudra procéder différemment selon que l'on utilise l'Arduino IDE ou Visual Studio Code avec l'extension PlatformIO.

### Activation de la fonctionnalité

Pour activer cette fonctionnalité, la procédure diffère selon que vous utilisez l'Arduino IDE ou Visual Studio Code avec l'extension PlatformIO.

#### Avec l'Arduino IDE
Activez la ligne suivante en supprimant le commentaire :
```cpp
#define TEMP_ENABLED
```

Si la bibliothèque *OneWire* n'est pas installée, installez-la via le menu **Outils** => **Gérer les bibliothèques…**.  
Recherchez "Onewire" et installez "**OneWire** par Jim Studt, …" en version **2.3.7** ou plus récente.

#### Avec Visual Studio Code et PlatformIO
Il n'y a pas d'environnement PlatformIO dédié pour la température.
Définissez simplement `TEMP_SENSOR_PRESENT` à `true` dans `config.h` et assurez-vous que la bibliothèque *OneWire* est disponible.

### Configuration du ou des capteurs (commun aux 2 cas précédents)
Pour configurer les capteurs, vous devez entrer leurs adresses.  
Utilisez un programme pour scanner les capteurs connectés.  
Vous pouvez trouver de tels programmes sur Internet ou parmi les exemples fournis avec l'Arduino IDE.  
Il est recommandé de coller une étiquette avec l'adresse de chaque capteur sur son câble.

Entrez les adresses comme suit :
```cpp
inline constexpr TemperatureSensing temperatureSensing{ 4,
                                                        { { 0x28, 0xBE, 0x41, 0x6B, 0x09, 0x00, 0x00, 0xA4 },
                                                          { 0x28, 0x1B, 0xD7, 0x6A, 0x09, 0x00, 0x00, 0xB7 } } };
```
Le nombre *4* en premier paramètre est la *pin* que l'utilisateur aura choisi pour le bus *OneWire*.

___
> [!NOTE]
> Plusieurs capteurs peuvent être branchés sur le même câble.  
> Sur Internet vous trouverez tous les détails concernant la topologie utilisable avec ce genre de capteurs.
___

## Gestion des Heures Creuses et boost programmé (dual tariff)

Cette fonctionnalité permet au routeur de gérer automatiquement le chauffage pendant les périodes d'Heures Creuses. Elle est utile pour :
- Chauffer l'eau la nuit quand l'électricité est moins chère
- Garantir de l'eau chaude le matin si le surplus solaire a été insuffisant pendant la journée
- Limiter la durée de chauffe pour éviter la surchauffe (optionnellement avec un capteur de température)

### Configuration matérielle

Décâblez la commande du contacteur Jour/Nuit, qui n'est plus nécessaire.
Reliez directement une *pin* choisie au contact sec du compteur (bornes *C1* et *C2*).

> [!WARNING]
> Il faut relier **directement**, une paire *pin/masse* avec les bornes *C1/C2* du compteur.
> Il NE doit PAS y avoir de 230 V sur ce circuit !

### Configuration logicielle

**Étape 1 :** Activez la fonctionnalité :
```cpp
inline constexpr bool DUAL_TARIFF{ true };
```

**Étape 2 :** Configurez la *pin* sur laquelle est relié le compteur :
```cpp
inline constexpr uint8_t dualTariffPin{ 3 };
```

**Étape 3 :** Configurez la durée en heures de la période d'Heures Creuses (pour l'instant, une seule période est supportée par jour) :
```cpp
inline constexpr uint8_t ul_OFF_PEAK_DURATION{ 8 };
```

**Étape 4 :** Configurez le timing du boost programmé pour chaque charge.

### Configuration du boost programmé (rg_ForceLoad)

Le tableau `rg_ForceLoad` définit **quand** et **combien de temps** chaque charge doit être en boost pendant la période d'Heures Creuses.

```cpp
inline constexpr pairForceLoad rg_ForceLoad[NO_OF_DUMPLOADS]{ { HEURE_DEBUT, DUREE } };
```

Chaque charge a deux paramètres : `{ HEURE_DEBUT, DUREE }`

#### Comprendre la ligne du temps

```
Exemple de période HC : 23:00 à 07:00 (8 heures)

        23:00                                           07:00
          |================== HEURES CREUSES =============|
          |                                              |
     DEBUT ──────────────────────────────────────────► FIN
          │                                              │
          │  Les valeurs positives                       │
          │  comptent à partir d'ici ───►                │
          │                                              │
          │                      ◄─── Les valeurs        │
          │                           négatives comptent │
          │                           à partir d'ici     │
```

#### Paramètre 1 : HEURE_DEBUT (quand démarrer)

| Valeur | Signification | Exemple (HC 23:00-07:00) |
|--------|---------------|--------------------------|
| `0` | **Désactivé** – pas de boost pour cette charge | - |
| `1` à `23` | Heures **après** le DÉBUT des HC | `3` = démarrage à 02:00 (23:00 + 3 h) |
| `-1` à `-23` | Heures **avant** la FIN des HC | `-3` = démarrage à 04:00 (07:00 - 3 h) |
| `24` ou plus | Minutes **après** le DÉBUT des HC | `90` = démarrage à 00:30 (23:00 + 90 min) |
| `-24` ou moins | Minutes **avant** la FIN des HC | `-90` = démarrage à 05:30 (07:00 - 90 min) |

> [!NOTE]
> **Pourquoi 24 ?** La valeur 24 sert de seuil pour distinguer les heures des minutes.
> Les valeurs de 1 à 23 sont interprétées comme des heures, les valeurs 24+ sont interprétées comme des minutes.

#### Paramètre 2 : DUREE (combien de temps)

| Valeur | Signification |
|--------|---------------|
| `0` | **Désactivé** - pas de boost |
| `1` à `23` | Durée en **heures** |
| `24` ou plus | Durée en **minutes** |
| `UINT16_MAX` | Jusqu'à la **fin** de la période HC |

> [!IMPORTANT]
> **Le boost s'arrête toujours à la fin de la période d'Heures Creuses**, quelle que soit la durée configurée.
> Si vous définissez une durée qui dépasse la fin des HC, le boost sera interrompu.

### Exemples visuels

Tous les exemples supposent une période HC de **23:00 à 07:00** (8 heures) :

**Exemple 1 :** `{ -3, 2 }` - Démarrage 3 heures avant la fin, durée 2 heures
```
23:00                              04:00    06:00    07:00
  |====================================|======|========|
                                       |BOOST |
                                       └─2 h──┘
```
Résultat : Le boost fonctionne de **04:00 à 06:00**

**Exemple 2 :** `{ 2, 3 }` - Démarrage 2 heures après le début, durée 3 heures
```
23:00    01:00          04:00                        07:00
  |========|=============|==============================|
           |────BOOST────|
           └────3 h──────┘
```
Résultat : Le boost fonctionne de **01:00 à 04:00**

**Exemple 3 :** `{ -90, 120 }` - Démarrage 90 minutes avant la fin, durée de 120 minutes (mais limitée)
```
23:00                              05:30    07:00
  |====================================|======|
                                       |BOOST | ← s'arrête ici (fin des HC)
                                       └90 min┘
```
Résultat : Le boost fonctionne de **05:30 à 07:00** (s'arrête à la fin des HC, pas à 07:30)

> [!NOTE]
> **Le boost s'arrête toujours à la fin de la période d'Heures Creuses**, même si la durée configurée est plus longue.
> Dans cet exemple, seules 90 minutes de boost ont lieu au lieu des 120 minutes configurées.

**Exemple 4 :** `{ 1, UINT16_MAX }` - Démarrage 1 heure après le début, jusqu'à la fin
```
23:00    00:00                                       07:00
  |========|=========================================|
           |──────────────BOOST──────────────────────|
```
Résultat : Le boost fonctionne de **00:00 à 07:00**

### Configuration pour plusieurs charges

Chaque charge peut avoir sa propre programmation de boost. Utilisez `{ 0, 0 }` pour désactiver le boost d'une charge spécifique.

**Exemple :** 2 charges, boost uniquement sur la deuxième :
```cpp
inline constexpr pairForceLoad rg_ForceLoad[NO_OF_DUMPLOADS]{
    { 0, 0 },      // Charge #1 : pas de boost programmé
    { -3, 2 }      // Charge #2 : boost 3h avant la fin, pendant 2h
};
```

### Aide-mémoire

| Vous voulez…   | Utilisez ceci |
|----------------|---------------|
| Désactiver le boost | `{ 0, 0 }` |
| Démarrer 2 h après le début des HC, pendant 3 h | `{ 2, 3 }` |
| Démarrer 3 h avant la fin des HC, pendant 2 h | `{ -3, 2 }` |
| Boost jusqu'à la fin des HC | `{ 1, UINT16_MAX }` |

## Rotation des priorités
La rotation des priorités est utile lors de l'alimentation d'un chauffe-eau triphasé.  
Elle permet d'équilibrer la durée de fonctionnement des différentes résistances sur une période prolongée.

Mais elle peut aussi être intéressante si on veut permuter les priorités de deux appareils chaque jour (deux chauffe-eau, …).

Une fois n'est pas coutume, l'activation de cette fonction possède 2 modes :
- **automatique**, on spécifiera alors
```cpp
inline constexpr RotationModes PRIORITY_ROTATION{ RotationModes::AUTO };
```
- **manuel**, on écrira alors
```cpp
inline constexpr RotationModes PRIORITY_ROTATION{ RotationModes::PIN };
```
En mode **automatique**, la rotation se fait automatiquement toutes les 24 h.  
Em mode **manuel**, vous devez également définir la *pin* qui déclenchera la rotation :
```cpp
inline constexpr uint8_t rotationPin{ 10 };
```

## Commandes Boost (Marche forcée)
Chaque entrée dans la table boost définit une source indépendante pouvant forcer une sortie spécifique à ON.
Un boost peut être déclenché par une pin physique (micro-interrupteur, minuterie — ATTENTION, PAS de 230 V sur cette ligne), depuis l'interface OLED, ou les deux.

Chaque entrée = `{ inputPin, outputIndex, visibleOnOLED }`
- `inputPin` : pin physique (INPUT_PULLUP) ou `unused_pin` pour OLED uniquement
- `outputIndex` : sortie cible — utiliser `LOAD(n)` ou `RELAY(n)`
- `visibleOnOLED` : `true` pour créer une page BOOST dédiée sur l'OLED

Exemple :
```cpp
inline constexpr BoostControlConfig boostControls{
  { { 7, LOAD(0), true },
    { unused_pin, RELAY(1), true },
    { 8, LOAD(0), false } }
};
```

## Groupes d'autorisation de routage
Chaque entrée définit un groupe de sorties dont le routage peut être bloqué indépendamment.
C'est utile pour désactiver le routage lors d'une absence prolongée.
La commande peut provenir d'une pin physique (contact sec, contrôlable à distance via une routine Alexa ou similaire), de l'interface OLED, ou les deux.

Chaque entrée = `{ inputPin, outputMask, visibleOnOLED }`
- `inputPin` : pin physique (INPUT_PULLUP, LOW = bloquer) ou `unused_pin` pour OLED uniquement
- `outputMask` : sorties concernées — utiliser `LOAD(n)`, `RELAY(n)`, `ALL_LOADS()`, `ALL_RELAYS()`, `ALL_LOADS_AND_RELAYS()`, ou combiner avec `|`
- `visibleOnOLED` : `true` pour afficher un interrupteur sur la page routage de l'OLED

Exemple :
```cpp
inline constexpr DiversionGroupConfig diversionGroups{
  { { 10, ALL_LOADS_AND_RELAYS(), true },
    { unused_pin, LOAD(0), true },
    { unused_pin, RELAY(0) | RELAY(1), true },
    { 9, ALL_LOADS(), false } }
};
```

# Configuration avancée du programme

Ces paramètres se trouvent dans le fichier `config_system.h`.

## Paramètre `DIVERSION_START_THRESHOLD_WATTS`
Le paramètre `DIVERSION_START_THRESHOLD_WATTS` définit un seuil de surplus avant tout routage vers les charges configurées sur le routeur. Elle est principalement destinée aux installations avec batteries de stockage.   
Par défaut, cette valeur est réglée à 0 W.  
En réglant ce paramètre à 50 W par exemple, le routeur ne démarrera le routage qu'à partir du moment où 50 W de surplus sera disponible. Une fois le routage démarré, la totalité du surplus sera routé.  
Cette fonctionnalité permet d'établir une hiérarchie claire dans l'utilisation de l'énergie produite, en privilégiant le stockage d'énergie sur la consommation immédiate. Vous pouvez ajuster cette valeur selon la réactivité du système de charge des batteries et vos priorités d'utilisation de l'énergie.

> [!IMPORTANT]
> Ce paramètre concerne uniquement la condition de démarrage du routage.
> Une fois le seuil atteint et le routage démarré, la **totalité** du surplus devient disponible pour les charges.

## Paramètre `REQUIRED_EXPORT_IN_WATTS`
Le paramètre `REQUIRED_EXPORT_IN_WATTS` détermine la quantité minimale d'énergie que le système doit réserver pour l'exportation ou l'importation vers le réseau électrique avant de dévier le surplus vers les charges contrôlées.  
Par défaut réglé à 0 W, ce paramètre peut être utilisé pour garantir une exportation constante vers le réseau, par exemple pour respecter des accords de revente d'électricité.  
Une valeur négative obligera le routeur à consommer cette puissance depuis le réseau. Cela peut être utile voire nécessaire pour les installations configurées en *zéro injection* afin d'amorcer la production solaire.

> [!IMPORTANT]
> Contrairement au premier paramètre, celui-ci représente un décalage permanent qui est continuellement soustrait du surplus disponible.
> Si réglé à 20 W par exemple, le système réservera **toujours** 20 W pour l'exportation, indépendamment des autres conditions.

# Dépannage
- Assurez-vous que toutes les bibliothèques requises sont installées.
- Vérifiez la configuration correcte des pins et des paramètres.
- Consultez la sortie série pour les messages d'erreur.

# Contribuer
Les contributions sont les bienvenues ! Veuillez soumettre des problèmes, des demandes de fonctionnalités et des pull requests via GitHub.

*doc non finie*
