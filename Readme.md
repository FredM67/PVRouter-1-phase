<div align = center>

[![CI](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/build.yml?branch=dev&label=CI&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/build.yml)
[![relay simulation](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/relay-sim.yml?branch=dev&label=relay%20simulation&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/relay-sim.yml)
[![CodeQL](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/codeql.yml?branch=dev&label=CodeQL&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/codeql.yml)
[![doc](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/doxygen-gh-pages.yml?branch=dev&label=doc&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/doxygen-gh-pages.yml)
<br/>
[![Arduino Uno](https://img.shields.io/badge/Arduino%20Uno-ATmega328P-00878F?logo=arduino&logoColor=white)](https://docs.arduino.cc/hardware/uno-rev3/)
[![flash](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Ffredm67.github.io%2FPVRouter-1-phase%2Fbench%2Fhistory.json&query=%24.entries%5B-1%3A%5D.sizes.basic.flash&label=flash&suffix=%20%2F%2032256%20B&color=blue)](https://fredm67.github.io/PVRouter-1-phase/bench/)
[![RAM](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Ffredm67.github.io%2FPVRouter-1-phase%2Fbench%2Fhistory.json&query=%24.entries%5B-1%3A%5D.sizes.basic.ram&label=RAM&suffix=%20%2F%202048%20B&color=blue)](https://fredm67.github.io/PVRouter-1-phase/bench/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ready-F5822A?logo=platformio&logoColor=white)](https://platformio.org/)
<br/>
[![License](https://img.shields.io/github/license/FredM67/PVRouter-1-phase)](https://github.com/FredM67/PVRouter-1-phase/blob/main/LICENSE)
[![Last commit](https://img.shields.io/github/last-commit/FredM67/PVRouter-1-phase/dev?label=last%20commit%20%28dev%29)](https://github.com/FredM67/PVRouter-1-phase/commits/dev)
[![Issues](https://img.shields.io/github/issues/FredM67/PVRouter-1-phase)](https://github.com/FredM67/PVRouter-1-phase/issues)
[![Stars](https://img.shields.io/github/stars/FredM67/PVRouter-1-phase?style=flat)](https://github.com/FredM67/PVRouter-1-phase/stargazers)
[![Forks](https://img.shields.io/github/forks/FredM67/PVRouter-1-phase?style=flat)](https://github.com/FredM67/PVRouter-1-phase/network)
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)
<br/>
<br/>
  [![en](https://img.shields.io/badge/lang-en-red.svg)](Readme.en.md)
  [![fr](https://img.shields.io/badge/lang-fr-blue.svg)](Readme.md)
</div>

# PVRouter (version monophasée)

Firmware d’un routeur photovoltaïque monophasé pour l’Arduino Uno (ATmega328P). Il mesure la puissance échangée avec le réseau, et envoie le surplus de la production solaire vers des charges résistives, typiquement la résistance d’un chauffe-eau, au lieu de l’injecter.

Il est issu du [Mk2PVRouter](https://web.archive.org/web/20231115123017/https://mk2pvrouter.co.uk/) monophasé (archive) de Robin Emley : il a le cœur de régulation de son `Mk2_fasterControl_twoLoads_5`, et les fonctionnalités et les réglages du firmware triphasé, `Mk2_3phase_RFdatalog_temp`. Tout le traitement critique en temps se fait dans l’interruption de l’ADC, en arithmétique entière uniquement, et toute la configuration est vérifiée à la compilation.

> [!NOTE]
> Vous construisez ou installez un routeur ? La **[documentation utilisateur](https://fredm67.github.io/Mk2PVRouter/)** couvre le matériel, l’installation, l’étalonnage et le dépannage.
> Pour une installation triphasée, voir [PVRouter-3-phase](https://github.com/FredM67/PVRouter-3-phase).

> [!WARNING]
> **Pas encore testé sur une vraie carte.** Ce firmware passe ses tests natifs, ses vérifications à la compilation et le simulateur de réseau sur les trois cartes, mais il n’a pas encore tourné sur du vrai matériel. En attendant, utilisez `Mk2_fasterControl_Full` sur une installation en service.

- [PVRouter (version monophasée)](#pvrouter-version-monophasée)
  - [Fonctionnalités](#fonctionnalités)
  - [Pour commencer](#pour-commencer)
  - [Organisation du dépôt](#organisation-du-dépôt)
  - [Documentation de développement](#documentation-de-développement)
  - [Installations en zéro injection (Enphase)](#installations-en-zéro-injection-enphase)
  - [Les anciens sketches, figés](#les-anciens-sketches-figés)
  - [Prochainement](#prochainement)
  - [Feuille de route](#feuille-de-route)
- [Support](#support)
- [Contribuer](#contribuer)
- [Auteurs et remerciements](#auteurs-et-remerciements)

## Fonctionnalités

Ce que le routeur fait pour vous :

- **Chauffer l’eau avec votre surplus solaire.** Dès que vos panneaux produisent plus que ce que la maison consomme, le routeur envoie l’excédent vers le chauffe-eau au lieu de le céder au réseau. Il dose au plus juste : 300 W de surplus, ce sont environ 300 W dans le chauffe-eau, sans acheter de courant au réseau ([réglage des sorties](Mk2_1phase_RFdatalog_temp/Readme.md#configuration-des-sorties-triac)).
- **Plusieurs résistances, dans l’ordre que vous choisissez.** Le routeur alimente la première ; quand elle est à pleine puissance, il passe à la suivante. Pour qu’elles s’usent de la même façon, l’ordre peut aussi changer chaque jour ([rotation des priorités](Mk2_1phase_RFdatalog_temp/Readme.md#rotation-des-priorités)).
- **Faire tourner une pompe à chaleur, la climatisation, la pompe de piscine... sur le surplus.** Ces appareils ne se dosent pas : ils sont soit en marche, soit à l’arrêt. Le routeur les met en marche par un relais quand le surplus suffit, et les arrête quand la maison se remet à acheter du courant, sans s’affoler au passage d’un nuage. Des durées minimales de marche et d’arrêt ménagent leur compresseur ou leur moteur. Fonctionne aussi avec une batterie domestique ([relais](Mk2_1phase_RFdatalog_temp/Readme.md#configuration-des-sorties-relais-tout-ou-rien)).
- **Piloter un chauffe-eau éloigné, sans fil.** Si le chauffe-eau est loin du tableau électrique, un petit boîtier placé près de lui reçoit les ordres du routeur par radio ([charges distantes](Mk2_1phase_RFdatalog_temp/Readme.md#configuration-du-module-rf-et-des-charges-distantes)).
- **Avoir de l’eau chaude même sans soleil.** Si la journée n’a pas suffi, le routeur chauffe pendant les heures creuses, à l’heure et pour la durée que vous choisissez, et s’arrête dès que l’eau est assez chaude ([heures creuses](Mk2_1phase_RFdatalog_temp/Readme.md#gestion-des-heures-creuses-et-boost-programmé-dual-tariff)). Un simple interrupteur peut aussi forcer la pleine puissance à tout moment ([boost manuel](Mk2_1phase_RFdatalog_temp/Readme.md#boost-manuel-déclenché-par-pin)).
- **Mesurer ce qui a été routé**, avec une deuxième sonde de courant (sonde 2), affiché sur toutes les sorties série et sur l’afficheur ([énergie routée](Mk2_1phase_RFdatalog_temp/Readme.md#énergie-routée-sonde-2), [afficheur](Mk2_1phase_RFdatalog_temp/Readme.md#afficheur)).
- **Connaître la température de l’eau** avec une ou plusieurs sondes ([sondes de température](Mk2_1phase_RFdatalog_temp/Readme.md#configuration-du-ou-des-capteurs-de-température)).
- **Mettre en pause pendant une absence** : soit arrêter le routage (le surplus repart sur le réseau, les heures creuses chauffent encore), soit tout éteindre, heures creuses comprises ([arrêt du routage](Mk2_1phase_RFdatalog_temp/Readme.md#arrêt-du-routage), [routeur à l’arrêt](Mk2_1phase_RFdatalog_temp/Readme.md#routeur-à-larrêt)).
- **Tout suivre dans Home Assistant** : puissances, tension, températures, état des charges, via le module Wi-Fi mk2Wifi ([mk2Wifi](Mk2_1phase_RFdatalog_temp/Readme.md#configuration-avec-le-module-mk2wifi)), ou sur un ordinateur relié en USB ([sortie série](Mk2_1phase_RFdatalog_temp/Readme.md#type-de-sortie-série)).
- **Un étalonnage facile**, grâce à un mode dédié dans le firmware ([étalonnage](Mk2_1phase_RFdatalog_temp/Readme.md#étalonnage-du-routeur)).
- **Trois cartes** : la carte Mk2 de Robin avec son afficheur 7 segments, la seconde ancienne carte monophasée avec un écran OLED, et la carte universelle ([carte mère](Mk2_1phase_RFdatalog_temp/Readme.md#carte-mère)).
- **Fonctionner aussi en zéro injection**, comme avec les micro-onduleurs Enphase réglés pour ne rien injecter ([plus bas](#installations-en-zéro-injection-enphase)).

## Pour commencer

1. **Matériel et câblage** : la [documentation utilisateur](https://fredm67.github.io/Mk2PVRouter/), la [notice d’installation](docs/HowToInstall.pdf), et les schémas de câblage dans [docs](docs/).
2. **Configuration** : les réglages sont dans `config.h`, `config_system.h` et `config_rf.h`, l’étalonnage dans `calibration.h`. Le [configurateur](configurator/) écrit les réglages depuis une page web : ouvrez `configurator/index.html` dans un navigateur.
3. **Compilation et téléversement** : avec [Visual Studio Code et PlatformIO](Mk2_1phase_RFdatalog_temp/Readme.md#utilisation-avec-visual-studio-code-recommandé) (recommandé) ou l’[Arduino IDE](Mk2_1phase_RFdatalog_temp/Readme.md#utilisation-avec-arduino-ide).
4. **Étalonnage** : voir l’[étalonnage du routeur](Mk2_1phase_RFdatalog_temp/Readme.md#étalonnage-du-routeur).

## Organisation du dépôt

| Dossier                                                    | Contenu                                                                                     |
| ---------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| [`Mk2_1phase_RFdatalog_temp`](Mk2_1phase_RFdatalog_temp/)  | le firmware du routeur, sa configuration, ses tests et sa documentation technique           |
| [`RemoteLoadReceiver`](RemoteLoadReceiver/)                | le firmware des unités de charges distantes (RFM69)                                         |
| [`configurator`](configurator/)                            | la page web qui écrit les fichiers de configuration                                         |
| [`sim`](sim/)                                              | le simulateur de réseau : le firmware tourne sur une installation monophasée simulée (simavr) |
| [`dev`](dev/)                                              | programmes de test et d’étalonnage : échantillons bruts, test de transformateur, test des afficheurs, ... |
| [`Robin`](Robin/)                                          | deux sketches de Robin Emley                                                                |
| `Mk2_fasterControl_*`                                      | les anciens sketches, figés ([plus bas](#les-anciens-sketches-figés))                        |
| [`docs`](docs/)                                            | notice d’installation, schémas de câblage                                                   |

## Documentation de développement

- [Documentation du code](https://fredm67.github.io/PVRouter-1-phase/doc/) (Doxygen, en anglais) et [historique de la taille et de la vitesse du firmware](https://fredm67.github.io/PVRouter-1-phase/bench/), sur le [site du projet](https://fredm67.github.io/PVRouter-1-phase/).
- [Journal des modifications](CHANGELOG.md) (en anglais) des changements sur `dev`, pas encore sur `main`.
- [Documentation technique](Mk2_1phase_RFdatalog_temp/docs/README.md) (en anglais) : architecture, interruptions, calcul de la puissance, valeurs d’étalonnage, performances.
- [Tests](Mk2_1phase_RFdatalog_temp/test/README.md) (en anglais) : tests unitaires natifs, tests embarqués et mesures de performance dans simavr.
- [Simulateur de réseau](sim/README.md) (en anglais) : régulation, timing de l’ISR, instants de commutation, radio et relais, vérifiés en CI.
- [Outils d’analyse](Mk2_1phase_RFdatalog_temp/Readme.md#documentation-danalyse-et-outils) : filtrage des sorties relais, réglage pour les climats nuageux et les systèmes avec batterie.

## Installations en zéro injection (Enphase)

Avec un profil zéro injection, l’installation photovoltaïque bride sa production dès qu’elle dépasse la consommation de la maison : rien n’est jamais injecté. Le routeur ne voit donc jamais de surplus, et n’a rien à router.

La solution est un `REQUIRED_EXPORT_IN_WATTS` négatif (dans `config_system.h`, voir [sa description](Mk2_1phase_RFdatalog_temp/Readme.md#paramètre-required_export_in_watts)) : le routeur se comporte alors comme un petit générateur photovoltaïque. Avec *-20*, il ajoute -20 W à chaque mesure :

- quand la maison **importe** de l’énergie, la mesure reste positive : le routeur ne fait rien ;
- quand la mesure est **autour de zéro**, la limitation de l’injection est active. Avec -20 W ajoutés, le routeur voit un surplus, et commence à router vers le chauffe-eau. L’Envoy voit plus de consommation et augmente la production ; le routeur revoit zéro, ajoute à nouveau -20 W, et route davantage. Cela continue jusqu’à ce que la production atteigne son maximum : la mesure reste alors juste au-dessus de zéro, et le système est stable.

Testé sur une installation réelle avec le firmware triphasé, qui traite ce réglage de la même façon. Selon l’installation, il faudra peut-être augmenter ou diminuer *-20*.

## Les anciens sketches, figés

Ces sketches restent tels quels, pour référence ; ils n’évoluent plus :

- ***Mk2_fasterControl_Full*** est une version remaniée du sketch d’origine ***Mk2_fasterControl_twoLoads_x*** de Robin Emley : traitements critiques dans l’interruption, journal série, sonde de température, relais, télémétrie pour Home Assistant, carte universelle, démarrage différé du routage, et affichage de l’énergie routée (la température sur l’afficheur est une nouveauté de `Mk2_1phase_RFdatalog_temp`).
- ***Mk2_fasterControl_twoLoads_temp_1*** et ***Mk2_fasterControl_threeLoads_temp_1*** sont des sketches en un seul fichier. Ils portent la logique deux charges de la version 4 de Robin, qu’il a archivée comme problématique : préférez `Mk2_1phase_RFdatalog_temp`, qui a la logique de sa version 5.
- ***Robin/*** contient deux sketches de Robin Emley, ***dev/*** ses outils de test et d’étalonnage.

## Prochainement

- schéma de câblage pour thermostat mécanique
- schéma de câblage pour thermostat électronique (ACI)

## Feuille de route

- le YAML ESPHome du module mk2Wifi, écrit par le configurateur ;
- les valeurs du filtre de la sonde 1 mesurées pour chaque type de sonde (celles livrées sont provisoires) ;
- les tests sur de vraies cartes.

# Support

Ce projet est maintenu par [@FredM67](https://github.com/FredM67). Il n’est pas possible d’assurer un support individuel par e-mail ; une aide partagée publiquement est d’ailleurs bien plus utile, puisque tout le monde peut en profiter.

| Type                                   | Où                                                                            |
| -------------------------------------- | ----------------------------------------------------------------------------- |
| 🚨 **Signaler un bug**                 | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 📚 **Problème de documentation**       | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 🎁 **Demande de fonctionnalité**       | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 🛡 **Signaler une faille de sécurité** | Voir [SECURITY.md](SECURITY.md)                                               |
| 💬 **Questions générales**             | [GitHub Discussions](https://github.com/FredM67/PVRouter-1-phase/discussions) |

# Contribuer

Lisez [CONTRIBUTING.md](CONTRIBUTING.md) (en anglais) pour le code de conduite et la façon de proposer des *pull requests*.

# Auteurs et remerciements

- **Robin Emley** - le Mk2PVRouter, ses cartes et ses sketches - [mk2pvrouter.co.uk](https://web.archive.org/web/20231115123017/https://mk2pvrouter.co.uk/) (archive)
- **Frédéric Metrich** - cette version - [FredM67](https://github.com/FredM67)

Voir aussi la liste des [contributeurs](https://github.com/FredM67/PVRouter-1-phase/graphs/contributors) qui ont participé à ce projet.

# Licence

GPL-3.0-or-later, voir [LICENSE](LICENSE). [NOTICE](NOTICE) (en anglais) précise l’origine du code : le firmware est issu du firmware triphasé et des sketches Mk2PVRouter de Robin Emley, qu’il a publiés sans mention de licence ; les modifications sont sous GPL.

Les fichiers qui contiennent vos réglages (`config.h`, `config_system.h`, `config_rf.h`, `calibration.h`) et tous les fichiers qu’écrit le configurateur sont à vous : utilisez-les, modifiez-les et partagez-les comme vous l’entendez.
