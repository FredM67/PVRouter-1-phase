<div align = center>

[![CI](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/build.yml?branch=dev&label=CI&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/build.yml)
[![relay simulation](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/relay-sim.yml?branch=dev&label=relay%20simulation&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/relay-sim.yml)
[![CodeQL](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/codeql.yml?branch=dev&label=CodeQL&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/codeql.yml)
[![doc](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/doxygen-gh-pages.yml?branch=main&label=doc&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/doxygen-gh-pages.yml)
<br/>
[![Arduino Uno](https://img.shields.io/badge/Arduino%20Uno-ATmega328P-00878F?logo=arduino&logoColor=white)](https://docs.arduino.cc/hardware/uno-rev3/)
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

Ma version du firmware Mk2PVRouter monophasé (voir <http://www.mk2pvrouter.co.uk>)  il envoie le surplus d’une installation photovoltaïque vers des charges résistives, comme un chauffe-eau, au lieu de l’injecter sur le réseau.

---
**_NOTE :_**

Pour la version triphasée, voir [PVRouter-3-phase](https://github.com/FredM67/PVRouter-3-phase).

---

## Mk2_1phase_RFdatalog_temp, le firmware

***[Mk2_1phase_RFdatalog_temp](Mk2_1phase_RFdatalog_temp/)*** est le firmware monophasé, celui qui continue d’évoluer. Il a les fonctionnalités et les réglages du firmware triphasé, `Mk2_3phase_RFdatalog_temp`, dont il est issu, avec le cœur de régulation de `Mk2_fasterControl_twoLoads_5` de Robin Emley 

- trois cartes  la carte Mk2 de Robin avec son afficheur 7 segments (`OLD_SEG`), la seconde ancienne carte monophasée avec un écran OLED (`OLD_OLED`), et la carte universelle (`NEW`) 
- l’énergie routée, mesurée par une deuxième sonde (sonde 2), affichée sur toutes les sorties série et sur l’afficheur 
- sorties TRIAC, sorties relais avec leur filtre et leurs durées minimales (mode batterie compris), charges distantes par radio (RFM69) 
- double tarif avec marches forcées programmées, rotation des priorités avec groupes de rotation, entrées de marche forcée, entrées d’arrêt du routage et d’arrêt du routeur 
- sondes de température DS18B20, LED témoin, afficheur 7 segments ou OLED 
- sortie série pour le moniteur série, pour Home Assistant via le module mk2Wifi (ESPHome), ou en JSON 
- chaque réglage vérifié à la compilation  tests natifs, tests dans un ATmega328P simulé, et un simulateur de réseau qui fait tourner le firmware sur une installation monophasée simulée, le tout lancé par la CI.

> [!WARNING]
> **Pas encore testé sur une vraie carte.** Ce firmware passe ses tests natifs, ses vérifications à la compilation et le simulateur de réseau sur les trois cartes, mais il n’a pas encore tourné sur du vrai matériel. En attendant, utilisez `Mk2_fasterControl_Full` sur une installation en service.

Documentation  [français](Mk2_1phase_RFdatalog_temp/Readme.md), [English](Mk2_1phase_RFdatalog_temp/Readme.en.md). Changements  [CHANGELOG.md](CHANGELOG.md) (en anglais).

## Les anciens sketches, figés

Ces sketches restent tels quels, pour référence  ils n’évoluent plus 

- ***Mk2_fasterControl_Full*** est une version remaniée du sketch d’origine ***Mk2_fasterControl_twoLoads_x*** de Robin Emley  traitements critiques dans l’interruption, journal série, sonde de température, relais, télémétrie pour Home Assistant, carte universelle, démarrage différé du routage, et affichage de l’énergie routée (la température sur l’afficheur est une nouveauté de `Mk2_1phase_RFdatalog_temp`).
- ***Mk2_fasterControl_twoLoads_temp_1*** et ***Mk2_fasterControl_threeLoads_temp_1*** sont des sketches en un seul fichier. Ils portent la logique deux charges de la version 4 de Robin, qu’il a archivée comme problématique  préférez `Mk2_1phase_RFdatalog_temp`, qui a la logique de sa version 5.
- ***Robin/*** contient deux sketches de Robin Emley, ***dev/*** ses outils de test et d’étalonnage.

## Prochainement
- schéma de câblage pour thermostat mécanique
- schéma de câblage pour thermostat électronique (ACI)

## Support

Ce projet est maintenu par [@FredM67](https://github.com/FredM67). Il n’est pas possible d’assurer un support individuel par e-mail  une aide partagée publiquement est d’ailleurs bien plus utile, puisque tout le monde peut en profiter.

| Type                                         | Où                                                                                  |
| -------------------------------------------- | ----------------------------------------------------------------------------------- |
| 🚨 **Signaler un bug**                       | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)          |
| 📚 **Problème de documentation**             | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)          |
| 🎁 **Demande de fonctionnalité**             | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)          |
| 🛡 **Signaler une faille de sécurité**       | Voir [SECURITY.md](SECURITY.md)                                                     |
| 💬 **Questions générales**                   | [GitHub Discussions](https://github.com/FredM67/PVRouter-1-phase/discussions)       |

## Feuille de route

- le mode monophasé du configurateur, et les paquets ESPHome du module mk2Wifi 
- les valeurs du filtre de la sonde 1 mesurées pour chaque type de sonde (celles livrées sont provisoires) 
- les tests sur de vraies cartes.

## Contribuer

Lisez [CONTRIBUTING.md](CONTRIBUTING.md) (en anglais) pour le code de conduite et la façon de proposer des *pull requests*.

## Auteurs et remerciements

- **Robin Emley** - le Mk2PVRouter, ses cartes et ses sketches - [www.mk2pvrouter.co.uk](http://www.mk2pvrouter.co.uk)
- **Frédéric Metrich** - cette version - [FredM67](https://github.com/FredM67)

Voir aussi la liste des [contributeurs](https://github.com/FredM67/PVRouter-1-phase/graphs/contributors) qui ont participé à ce projet.

## Licence

GPL-3.0-or-later, voir [LICENSE](LICENSE). [NOTICE](NOTICE) (en anglais) précise l’origine du code  le firmware est issu du firmware triphasé et des sketches Mk2PVRouter de Robin Emley, qu’il a publiés sans mention de licence  les modifications sont sous GPL.

Les fichiers qui contiennent vos réglages (`config.h`, `config_system.h`, `config_rf.h`, `calibration.h`) sont à vous  utilisez-les, modifiez-les et partagez-les comme vous l’entendez.
