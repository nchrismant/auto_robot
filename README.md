# Auto Robot – Suivi de Ligne avec Raspberry Pi, OpenCV & Arduino

Robot autonome capable de suivre une ligne grâce à un système de vision embarqué (OpenCV sur Raspberry Pi), une communication série et un contrôle moteur via Arduino.

---

## 📌 Sommaire

- [Auto Robot – Suivi de Ligne avec Raspberry Pi, OpenCV \& Arduino](#auto-robot--suivi-de-ligne-avec-raspberry-pi-opencv--arduino)
  - [📌 Sommaire](#-sommaire)
  - [🎯 Objectif du projet](#-objectif-du-projet)
  - [✨ Fonctionnalités principales](#-fonctionnalités-principales)
  - [🧩 Structure du projet / Architecture](#-structure-du-projet--architecture)
  - [🤖 Architecture Matérielle](#-architecture-matérielle)
  - [🧠 Architecture Logicielle](#-architecture-logicielle)
    - [Traitement d'image (Raspberry Pi – C++ / OpenCV)](#traitement-dimage-raspberry-pi--c--opencv)
    - [Programme de contrôle](#programme-de-contrôle)
    - [Firmware Arduino](#firmware-arduino)
  - [📊 Optimisations \& Performances](#-optimisations--performances)
    - [Parallélisation (threads)](#parallélisation-threads)
    - [Réduction du nombre d’angles](#réduction-du-nombre-dangles)
    - [Réduction de la fenêtre de traitement](#réduction-de-la-fenêtre-de-traitement)
    - [Mise en place de métriques](#mise-en-place-de-métriques)
  - [🚀 Installation \& Déploiement](#-installation--déploiement)
    - [Prérequis](#prérequis)
    - [Compilation](#compilation)
    - [Chargement du firmware Arduino](#chargement-du-firmware-arduino)
  - [🛠️ Technologies \& Outils utilisés](#️-technologies--outils-utilisés)
  - [👥 Auteurs \& Licence](#-auteurs--licence)

---

## 🎯 Objectif du projet

L’objectif de ce projet est de concevoir un **robot autonome** capable de suivre une ligne en utilisant :

- une **caméra** pour analyser son environnement,
- une **Raspberry Pi** pour effectuer le traitement d'image,
- un **Arduino** pour exécuter les commandes moteur.

Le robot détecte les lignes via OpenCV, calcule un **angle de correction**, puis ajuste sa trajectoire grâce au microcontrôleur Arduino.

---

## ✨ Fonctionnalités principales

- **Capture d'image en temps réel** via la caméra Pi.  
- **Détection de contours** (Canny + filtre Sobel).  
- **Détection de lignes** via Transformée de Hough simplifiée.  
- **Calcul automatique de l’angle de correction du robot**.  
- **Communication série Raspberry Pi → Arduino** (UART).  
- **Contrôle moteur dynamique** sur Arduino.  
- **Optimisations CPU** :
  - parallélisation des calculs,
  - réduction du nombre d'angles,
  - réduction de la zone d’analyse.  
- **Métriques en temps réel** : FPS & taux CPU.

---

## 🧩 Structure du projet / Architecture

```text
/ (racine)
├── Makefile
├── README.md
├── main.cpp # Traitement d'image + Hough + communication série
└── route_low.jpg # Image test utilisée pour les benchmarks
```

---

## 🤖 Architecture Matérielle

- **Raspberry Pi 3 Model B+** : unité principale de traitement (vision + prise de décision)  
- **Caméra Raspberry Pi** : capture les images analysées par OpenCV  
- **Arduino (Robotics Shield Kit)** : reçoit les commandes via UART et pilote les moteurs  
- **Robot + moteurs** : plateforme mobile autonome  
- **Câbles UART** : communication série bidirectionnelle entre Raspberry Pi et Arduino

---

## 🧠 Architecture Logicielle

### Traitement d'image (Raspberry Pi – C++ / OpenCV)

- **Filtre Sobel** → détection des bords  
- **Canny** → amélioration de la précision des contours  
- **Transformée de Hough simplifiée** → détection des droites dans l’image  
- **Calcul de l’angle** : moyenne des 4 meilleurs angles détectés (référence axe Y)

### Programme de contrôle

- Envoie l’angle au microcontrôleur via UART  
- Ajuste la trajectoire du robot selon l’inclinaison de la ligne

### Firmware Arduino

- Réception des commandes série  
- Pilotage des moteurs selon l’angle reçu  
- Gestion fine des vitesses & corrections

---

## 📊 Optimisations & Performances

### Parallélisation (threads)

- Division de l’image en 4 zones  
- Chaque thread traite une section avec sa propre matrice d'accumulation  
- Utilisation des 4 cœurs de la Pi pour :
  - accélérer la Transformée de Hough  
  - réduire le temps de traitement global

### Réduction du nombre d’angles

- Passer de 1° à 10° réduit x10 les itérations :
  - **174 420 → 17 442** opérations pour une image de test

### Réduction de la fenêtre de traitement

- Par exemple : analyse uniquement des **50% inférieurs** de l’image  
- Justification : les lignes suivies par le robot se trouvent toujours au sol

### Mise en place de métriques

- **FPS** : nombre d’images traitées/seconde  
- **% CPU** : charge processeur pendant le traitement  

Ces métriques permettent de comparer objectivement les optimisations.

---

## 🚀 Installation & Déploiement

### Prérequis

- Raspberry Pi OS  
- OpenCV C++  
- Arduino IDE  
- Câble UART (TX/RX croisés)

### Compilation

```bash
make 
./auto_robot
```

### Chargement du firmware Arduino

Téléverser le code via l’Arduino IDE sur le module robotique.

---

## 🛠️ Technologies & Outils utilisés

| Technologie      | Rôle              |
| ---------------- | ----------------- |
| **C++**         | Langage principal |
| **OpenCV**      | Traiter les images de la caméra et détecter les lignes |
| **Raspberry Pi 3 Model B+**         | Unité principale de traitement |
| **UART / Communication série**      | Communication série bidirectionnelle entre Raspberry Pi et Arduino    |

---

## 👥 Auteurs & Licence

- **CHRISMANT Nathan** — Étudiant M1 Informatique, Cergy Paris Université.
- **LEMARCHAND Jonathan** — Étudiant M1 Informatique, Cergy Paris Université.

Projet distribué sous licence **Open Source**.
